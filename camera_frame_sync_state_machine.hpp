#pragma once

/*
 * CameraFrameSync 状态机只在图像提交时推进。
 *
 * 输入图像本体已经通过 LinuxSharedTopic 发布，这里只处理图像 timestamp 事件。
 * 原始 IMU 回调只入队，CameraSync 回执回调只写当前 probe 的邮箱；所有匹配、
 * 重同步、offset 等状态更新都在本文件中串行完成。
 *
 * ImageDecision 约定：
 * - WAIT：当前图像还不能发布，保留 pending_frame_，等待后续 IMU 或回执。
 * - DONE：当前图像已经处理完成，可以丢弃 pending_frame_。
 * - RESET：当前图像证明同步关系不可用，需要丢弃它并回到 OBSERVING。
 */

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ProcessImageEvents()
{
  while (pending_frame_.valid || !image_events_.Empty())
  {
    if (!pending_frame_.valid)
    {
      if (!image_events_.Front(pending_frame_.image))
      {
        break;
      }
      // 图像队列只保存 timestamp 事件；同一时刻最多处理一个未完成图像。
      pending_frame_.valid = true;
      image_events_.PopFront();
    }

    const ImageDecision decision =
        sync_mode_ == SyncMode::LATEST_IMU ? ProcessLatestImage(pending_frame_)
                                           : ProcessRawProbeImage(pending_frame_);
    if (decision == ImageDecision::WAIT)
    {
      break;
    }
    if (decision == ImageDecision::RESET)
    {
      ClearPendingFrame();
      ResetLock("image-rejected", StateName(state_));
      continue;
    }

    ClearPendingFrame();
  }
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::ProcessLatestImage(
    typename CameraFrameSync<CameraInfoV>::PendingFrame& frame)
{
  // LATEST_IMU 模式假设数据源已经同步；乱序图像直接丢弃并重看图像时间轴。
  if (last_image_valid_ &&
      frame.image.sensor_timestamp_us <= last_image_timestamp_us_)
  {
    ResetImageObservation();
    return ImageDecision::DONE;
  }

  // offset 可能指向未来 IMU。已经锁定过 match 的图像必须沿用原 match，
  // 否则等待期间新 IMU 进入历史后会导致同一张图像重新选择同步点。
  const ImageDecision decision = frame.match.valid
                                     ? ResumePendingMatch(frame)
                                     : TryLatestImuMatch(frame);
  if (decision == ImageDecision::WAIT)
  {
    return decision;
  }

  RememberImage(frame.image.sensor_timestamp_us);
  return decision;
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::ProcessRawProbeImage(
    typename CameraFrameSync<CameraInfoV>::PendingFrame& frame)
{
  // 这张图像已经找到同步基准 IMU，只是在等 offset 后的最终 IMU。
  if (frame.match.valid)
  {
    return ResumePendingMatch(frame);
  }

  // 第一张图像只能建立图像时间轴起点，不能计算 gap。
  if (!last_image_valid_)
  {
    if (!frame.cadence_consumed)
    {
      ObserveNormalImageCadence(frame.image.sensor_timestamp_us);
      frame.cadence_consumed = true;
    }
    RememberImage(frame.image.sensor_timestamp_us);
    MaybeStartProbe();
    return ImageDecision::DONE;
  }

  const uint64_t image_ts = frame.image.sensor_timestamp_us;
  if (image_ts <= last_image_timestamp_us_)
  {
    // 图像时间戳倒退说明当前图像节拍不可继续信任；清掉图像观察并重新开始。
    ResetImageObservation();
    ResetLock("image-out-of-order", "RAW_PROBE");
    ObserveNormalImageCadence(image_ts);
    RememberImage(image_ts);
    return ImageDecision::DONE;
  }

  const uint64_t image_gap_us = image_ts - last_image_timestamp_us_;
  if (state_ == SyncState::PROBE_SENT && IsProbeImageGap(image_gap_us))
  {
    // probe gap 是 CameraSync 命令主动拉长出来的，不能拿来重估正常图像周期。
    // 但它仍然是下一张图像的前一帧，所以需要推进 cadence 的最后 timestamp。
    if (!frame.cadence_consumed)
    {
      image_cadence_.last_timestamp_us = image_ts;
      frame.cadence_consumed = true;
    }
    return TryProbeImage(frame);
  }

  auto cadence_update = CameraFrameSyncCore::CadenceUpdate::STABLE;
  if (!frame.cadence_consumed)
  {
    cadence_update = ObserveNormalImageCadence(image_ts);
    frame.cadence_consumed = true;
  }
  if (cadence_update == CameraFrameSyncCore::CadenceUpdate::BROKEN)
  {
    // 周期稳定后又被打破，说明同步关系可能失效。保留新图像作为下一轮起点。
    ResetLock("image-cadence-broken", "RAW_PROBE");
    RememberImage(image_ts);
    return ImageDecision::DONE;
  }

  switch (state_)
  {
    case SyncState::OBSERVING:
      // 两路周期都稳定后，MaybeStartProbe() 会发送一次 CameraSync 命令。
      RememberImage(image_ts);
      MaybeStartProbe();
      return ImageDecision::DONE;

    case SyncState::PROBE_SENT:
      // probe 命令发出后，仍可能先看到若干正常图像；继续等真正的 probe gap。
      if (IsNormalImageGap(image_gap_us))
      {
        RememberImage(image_ts);
        return ImageDecision::DONE;
      }
      return ImageDecision::RESET;

    case SyncState::SYNCED:
      // 锁定后图像必须保持正常周期；否则丢弃当前锁并重同步。
      if (!IsNormalImageGap(image_gap_us))
      {
        return ImageDecision::RESET;
      }
      return frame.match.valid ? ResumePendingMatch(frame) : TrySyncedImage(frame);
  }

  return ImageDecision::RESET;
}

template <CameraTypes::CameraInfo CameraInfoV>
CameraFrameSyncCore::CadenceUpdate
CameraFrameSync<CameraInfoV>::ObserveNormalImageCadence(uint64_t image_ts)
{
  // 只观察正常图像 gap。probe gap 会绕过这里，避免污染 baseline 周期。
  const auto update = CameraFrameSyncCore::ObserveCadence(
      image_cadence_, image_ts, cadence_stable_gaps, image_cadence_tolerance_us);
  if (image_cadence_.stable)
  {
    periods_.image_us = image_cadence_.period_us;
  }
  return update;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::MaybeStartProbe()
{
  // RAW_PROBE 只允许一个 outstanding probe；必须先看到稳定图像周期和 IMU 周期。
  if (sync_mode_ != SyncMode::RAW_PROBE || state_ != SyncState::OBSERVING)
  {
    return;
  }
  if (!image_cadence_.stable || !imu_cadence_.stable || !last_image_valid_)
  {
    return;
  }

  const uint64_t sync_period_us = EstimatedSyncPeriodUs();
  if (sync_period_us == 0)
  {
    return;
  }

  CameraSync::SyncCommand cmd{};
  cmd.div = sync_probe_div_;
  cmd.active_level = sync_active_level_;
  cmd.seq = next_sync_seq_++;

  // seq 是 CameraSync 跨端协议的一部分，用来过滤旧回执或串扰回执。
  state_ = SyncState::PROBE_SENT;
  pending_probe_seq_ = cmd.seq;
  pending_probe_ack_valid_ = false;
  pending_probe_imu_timestamp_us_ = 0;
  // 先启用当前 probe 再发布命令，避免同步回环里的早到回执丢失。
  probe_ack_timestamp_us_.store(0);
  probe_ack_seq_.store(0);
  active_probe_seq_.store(cmd.seq);

  topics_.sync_command.Publish(cmd);

  XR_LOG_INFO(
      "CameraFrameSync: sync command sent seq=%u div=%u active=%u image_period_us=%u imu_period_us=%u sync_period_us=%u",
      static_cast<unsigned>(cmd.seq), static_cast<unsigned>(cmd.div),
      static_cast<unsigned>(cmd.active_level),
      static_cast<unsigned>(periods_.image_us),
      static_cast<unsigned>(periods_.imu_us),
      static_cast<unsigned>(sync_period_us));
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::TryProbeImage(
    typename CameraFrameSync<CameraInfoV>::PendingFrame& frame)
{
  // 当前图像已经表现为 probe gap；还需要等同 seq 的 CameraSync 回执。
  const uint64_t sync_period_us = EstimatedSyncPeriodUs();
  if (!pending_probe_ack_valid_ && probe_ack_seq_.load() == pending_probe_seq_)
  {
    pending_probe_ack_valid_ = true;
    pending_probe_imu_timestamp_us_ = probe_ack_timestamp_us_.load();
  }
  if (sync_period_us == 0 || !pending_probe_ack_valid_)
  {
    return ImageDecision::WAIT;
  }

  const uint64_t ack_ts = pending_probe_imu_timestamp_us_;
  if (!ImuHistoryReached(ack_ts))
  {
    // 回执 timestamp 属于 IMU 时间轴。历史还没覆盖到该点时，保留当前图像等待。
    return ImageDecision::WAIT;
  }

  const AssembledImu* sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
      imu_history_, ack_ts,
      CameraFrameSyncCore::ImuTimestampToleranceUs(periods_.imu_us));
  if (sync_imu == nullptr)
  {
    // 历史已经越过容差窗口仍找不到 ack 对应的完整 IMU，说明这次 probe 无效。
    return ImuHistoryReached(ack_ts + CameraFrameSyncCore::ImuTimestampToleranceUs(
                                          periods_.imu_us))
               ? ImageDecision::RESET
               : ImageDecision::WAIT;
  }

  return PublishOrRememberMatch(frame,
                                SyncMatch{.imu = sync_imu, .period_us = sync_period_us});
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::TryLatestImuMatch(
    typename CameraFrameSync<CameraInfoV>::PendingFrame& frame)
{
  // 已同步数据源的短路径：直接拿最新完整 IMU 作为当前图像的同步基准。
  if (imu_history_.Empty())
  {
    return ImageDecision::WAIT;
  }

  const uint64_t period = periods_.imu_us != 0 ? periods_.imu_us : 1ULL;
  return PublishOrRememberMatch(
      frame, SyncMatch{.imu = &imu_history_.Back(), .period_us = period});
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::TrySyncedImage(
    typename CameraFrameSync<CameraInfoV>::PendingFrame& frame)
{
  // 已锁定后不再比较相机时间和 IMU 时间的绝对值，只沿 IMU 时间轴递推。
  if (locked_sync_.last_imu_timestamp_us == 0 || locked_sync_.period_us == 0)
  {
    return ImageDecision::RESET;
  }

  const uint64_t expected_sync_ts =
      locked_sync_.last_imu_timestamp_us + locked_sync_.period_us;
  if (!ImuHistoryReached(expected_sync_ts))
  {
    // 图像先于对应 IMU 到达时，挂住这张图像等待后续 IMU 回调入队。
    return ImageDecision::WAIT;
  }

  const AssembledImu* sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
      imu_history_, expected_sync_ts,
      CameraFrameSyncCore::ImuTimestampToleranceUs(periods_.imu_us));
  if (sync_imu == nullptr)
  {
    return ImageDecision::RESET;
  }

  return PublishOrRememberMatch(
      frame, SyncMatch{.imu = sync_imu, .period_us = locked_sync_.period_us});
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::ResumePendingMatch(
    typename CameraFrameSync<CameraInfoV>::PendingFrame& frame)
{
  // pending match 只保存 IMU timestamp，不保存指针；历史数组移动后仍然安全。
  if (!frame.match.valid)
  {
    return ImageDecision::RESET;
  }

  const AssembledImu* sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
      imu_history_, frame.match.imu_timestamp_us, 0);
  if (sync_imu == nullptr)
  {
    // 已经锁住的同步点被历史窗口挤掉，当前图像无法再正确发布。
    return ImageDecision::RESET;
  }

  return PublishOrRememberMatch(
      frame,
      SyncMatch{.imu = sync_imu, .period_us = frame.match.period_us});
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::PublishOrRememberMatch(
    typename CameraFrameSync<CameraInfoV>::PendingFrame& frame,
    const typename CameraFrameSync<CameraInfoV>::SyncMatch& match)
{
  // 先尝试发布；如果只是 offset 后的最终 IMU 还没到，就记录同步基准继续等。
  const ImageDecision decision = PublishMatchedImage(frame.image, match);
  if (decision == ImageDecision::WAIT)
  {
    frame.match.valid = true;
    frame.match.imu_timestamp_us = match.imu->sensor_timestamp_us;
    frame.match.period_us = match.period_us;
    return decision;
  }

  frame.match = {};
  return decision;
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::PublishMatchedImage(
    const typename CameraFrameSync<CameraInfoV>::ImageSample& image,
    const typename CameraFrameSync<CameraInfoV>::SyncMatch& match)
{
  if (match.imu == nullptr || match.period_us == 0)
  {
    return ImageDecision::RESET;
  }

  const int32_t offset_us = offset_us_;
  const uint64_t final_ts =
      CameraFrameSyncCore::ApplyOffsetUs(match.imu->sensor_timestamp_us, offset_us);
  if (!ImuHistoryReached(final_ts))
  {
    // offset_us 在 IMU 时间轴内生效；正 offset 可能需要等待未来 IMU。
    return ImageDecision::WAIT;
  }

  const AssembledImu* final_imu = CameraFrameSyncCore::FindBySensorTimestamp(
      imu_history_, final_ts,
      CameraFrameSyncCore::ImuTimestampToleranceUs(periods_.imu_us));
  if (final_imu == nullptr)
  {
    // 已经越过目标时间但找不到容差内 IMU，说明当前锁定关系不可继续使用。
    return ImageDecision::RESET;
  }

  // 输出 IMU 的 timestamp 对齐图像 timestamp；姿态内容来自 offset 后 IMU。
  PublishSyncedImu(image.sensor_timestamp_us, *final_imu);
  RememberImage(image.sensor_timestamp_us);

  // 成功发布后更新锁定关系。后续图像从这个同步基准继续按 IMU 周期递推。
  const SyncState old_state = state_;
  state_ = SyncState::SYNCED;
  locked_sync_.period_us = match.period_us;
  locked_sync_.last_imu_timestamp_us = match.imu->sensor_timestamp_us;
  ClearPendingProbe();

  if (old_state != SyncState::SYNCED)
  {
    XR_LOG_PASS(
        "CameraFrameSync: state %s -> SYNCED mode=%s image_period_us=%u imu_period_us=%u sync_period_us=%u offset_us=%d",
        StateName(old_state), SyncModeName(sync_mode_),
        static_cast<unsigned>(periods_.image_us),
        static_cast<unsigned>(periods_.imu_us),
        static_cast<unsigned>(locked_sync_.period_us), static_cast<int>(offset_us));
  }
  return ImageDecision::DONE;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::PublishSyncedImu(
    uint64_t image_timestamp_us,
    const typename CameraFrameSync<CameraInfoV>::AssembledImu& imu)
{
  ImuStamped synced{
      .timestamp_us = image_timestamp_us,
      .rotation_wxyz = imu.rotation_wxyz,
      .translation_xyz = {0.0f, 0.0f, 0.0f},
      .angular_velocity_xyz = imu.angular_velocity_xyz,
      .linear_acceleration_xyz = imu.linear_acceleration_xyz,
  };
  topics_.synced_imu.Publish(synced);
}

template <CameraTypes::CameraInfo CameraInfoV>
uint64_t CameraFrameSync<CameraInfoV>::EstimatedSyncPeriodUs() const
{
  // 把图像周期换算成 IMU 时间轴上的整数步长，避免比较两个时间轴绝对值。
  if (periods_.image_us == 0 || periods_.imu_us == 0)
  {
    return 0;
  }
  const uint32_t stride = CameraFrameSyncCore::EstimateStrideSamples(
      periods_.image_us, periods_.imu_us);
  return stride == 0 ? 0 : periods_.imu_us * static_cast<uint64_t>(stride);
}

template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::IsNormalImageGap(uint64_t image_gap_us) const
{
  // 这里只检查相邻图像 gap 是否仍符合相机自身周期。
  return periods_.image_us != 0 &&
         CameraFrameSyncCore::AbsDiffUs(image_gap_us, periods_.image_us) <=
             CameraFrameSyncCore::ImageGapToleranceUs(periods_.image_us);
}

template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::IsProbeImageGap(uint64_t image_gap_us) const
{
  // CameraSync 只拉长一次反向半周期，Host 侧应看到一个可预测的长图像 gap。
  const uint64_t expected_gap =
      CameraFrameSyncCore::ProbeImageGapUs(periods_.image_us, sync_probe_div_);
  return expected_gap != 0 &&
         CameraFrameSyncCore::AbsDiffUs(image_gap_us, expected_gap) <=
             CameraFrameSyncCore::ImageGapToleranceUs(periods_.image_us);
}

template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::ImuHistoryReached(
    uint64_t target_timestamp_us) const
{
  return !imu_history_.Empty() &&
         imu_history_.Back().sensor_timestamp_us >= target_timestamp_us;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::RememberImage(uint64_t image_timestamp_us)
{
  last_image_valid_ = true;
  last_image_timestamp_us_ = image_timestamp_us;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ResetImageObservation()
{
  // 只重置图像节拍观察；IMU 周期仍可继续观察。
  image_cadence_ = {};
  periods_.image_us = 0;
  last_image_valid_ = false;
  last_image_timestamp_us_ = 0;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ClearPendingProbe()
{
  // 清掉 outstanding probe 与回执邮箱，下一轮 probe 必须重新发命令。
  active_probe_seq_.store(0);
  probe_ack_seq_.store(0);
  probe_ack_timestamp_us_.store(0);
  pending_probe_seq_ = 0;
  pending_probe_ack_valid_ = false;
  pending_probe_imu_timestamp_us_ = 0;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ClearPendingFrame()
{
  pending_frame_ = {};
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ResetLock(const char* reason, const char* detail)
{
  // 丢弃已经锁定的同步关系，但保留已观察到的周期，便于尽快重新发 probe。
  const SyncState old_state = state_;
  state_ = SyncState::OBSERVING;
  locked_sync_ = {};
  ClearPendingProbe();

  if (old_state != SyncState::OBSERVING)
  {
    XR_LOG_WARN(
        "CameraFrameSync: state %s -> OBSERVING reason=%s detail=%s image_period_us=%u imu_period_us=%u",
        StateName(old_state), reason, detail,
        static_cast<unsigned>(periods_.image_us),
        static_cast<unsigned>(periods_.imu_us));
  }
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ResetRuntimeState()
{
  pending_gyros_.Clear();
  pending_accls_.Clear();
  pending_quats_.Clear();
  image_events_.Clear();
  imu_history_.Clear();
  pending_frame_ = {};
  image_cadence_ = {};
  imu_cadence_ = {};
  periods_ = {};
  locked_sync_ = {};
  last_image_valid_ = false;
  last_image_timestamp_us_ = 0;
  state_ = SyncState::OBSERVING;
  ClearPendingProbe();
  overflowed_.store(false, std::memory_order_relaxed);
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::HandleOverflowRecovery()
{
  if (!overflowed_.exchange(false, std::memory_order_relaxed))
  {
    return;
  }

  XR_LOG_WARN("CameraFrameSync: ingress overflow, runtime state reset");
  ResetRuntimeState();
}
