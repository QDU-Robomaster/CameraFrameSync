#pragma once

// 图像提交是唯一的状态机时钟。IMU 回调和 CameraSync 回执回调只搬运数据，
// 所有跨队列匹配、等待和重同步都在这里串行完成。

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
      pending_frame_.valid = true;
      image_events_.PopFront();
    }

    const ImageDecision decision =
        sync_mode_ == SyncMode::LATEST_IMU ? ProcessLatestImage(pending_frame_)
                                           : ProcessRawProbeImage(pending_frame_);
    if (decision == ImageDecision::WAIT)
    {
      // 当前图像已经成为同步基线，不能越过它消费下一帧。
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
  if (last_image_valid_ &&
      frame.image.sensor_timestamp_us <= last_image_timestamp_us_)
  {
    ResetImageObservation();
    return ImageDecision::DONE;
  }

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
  if (frame.match.valid)
  {
    // 同步基准已经锁定，只是在等 offset 后的最终 IMU 到达。
    return ResumePendingMatch(frame);
  }

  if (!last_image_valid_)
  {
    // 第一帧只能建立图像时间基线；是否发 probe 取决于 IMU 周期是否也稳定。
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
    // 图像时间倒退说明相机侧时间轴不可连续使用，只保留新的观察起点。
    ResetImageObservation();
    ResetLock("image-out-of-order", "RAW_PROBE");
    ObserveNormalImageCadence(image_ts);
    RememberImage(image_ts);
    return ImageDecision::DONE;
  }

  const uint64_t image_gap_us = image_ts - last_image_timestamp_us_;
  if (state_ == SyncState::PROBE_SENT &&
      (IsProbeImageGap(image_gap_us) ||
       (probe_ack_seq_.load() == pending_probe_seq_ && !IsNormalImageGap(image_gap_us))))
  {
    // 这次 gap 是同步探针或切换运行分频制造的异常间隔，不能拿来重估正常图像周期。
    if (!frame.cadence_consumed)
    {
      image_cadence_.last_timestamp_us = image_ts;
      frame.cadence_consumed = true;
    }
    return TryProbeImage(frame);
  }

  const uint32_t image_gap_stride = state_ == SyncState::SYNCED
                                        ? MatchImageGapStride(image_gap_us)
                                        : 0;
  auto cadence_update = CameraFrameSyncCore::CadenceUpdate::STABLE;
  if (!frame.cadence_consumed)
  {
    if (image_gap_stride > 1)
    {
      image_cadence_.last_timestamp_us = image_ts;
      frame.cadence_consumed = true;
    }
    else
    {
      cadence_update = ObserveNormalImageCadence(image_ts);
      frame.cadence_consumed = true;
    }
  }
  if (cadence_update == CameraFrameSyncCore::CadenceUpdate::BROKEN)
  {
    ResetLock("image-cadence-broken", "RAW_PROBE");
    RememberImage(image_ts);
    return ImageDecision::DONE;
  }

  switch (state_)
  {
    case SyncState::OBSERVING:
      RememberImage(image_ts);
      MaybeStartProbe();
      return ImageDecision::DONE;

    case SyncState::PROBE_SENT:
      if (IsNormalImageGap(image_gap_us))
      {
        // probe 尚未体现到图像间隔上，继续等真正被拉长的那一帧。
        if (ProbeTimedOut())
        {
          ResetLock("probe-timeout", "normal image gaps without sync result");
          RememberImage(image_ts);
          MaybeStartProbe();
          return ImageDecision::DONE;
        }
        RememberImage(image_ts);
        return ImageDecision::DONE;
      }
      return ImageDecision::RESET;

    case SyncState::SYNCED:
      if (image_gap_stride == 0)
      {
        return ImageDecision::RESET;
      }
      return frame.match.valid ? ResumePendingMatch(frame)
                               : TrySyncedImage(frame, image_gap_stride);
  }

  return ImageDecision::RESET;
}

template <CameraTypes::CameraInfo CameraInfoV>
CameraFrameSyncCore::CadenceUpdate
CameraFrameSync<CameraInfoV>::ObserveNormalImageCadence(uint64_t image_ts)
{
  const auto update = CameraFrameSyncCore::ObserveCadence(
      image_cadence_, image_ts, cadence_stable_gaps, image_cadence_tolerance_us);
  if (image_cadence_.stable)
  {
    periods_.image_us = image_cadence_.period_us;
  }
  return update;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ObserveDroppedImage(uint64_t image_ts)
{
  if (!last_image_valid_)
  {
    ObserveNormalImageCadence(image_ts);
    RememberImage(image_ts);
    return;
  }

  if (image_ts <= last_image_timestamp_us_)
  {
    ResetImageObservation();
    ObserveNormalImageCadence(image_ts);
    RememberImage(image_ts);
    return;
  }

  const uint64_t image_gap_us = image_ts - last_image_timestamp_us_;
  const SyncState old_state = state_;
  const bool normal_gap = IsNormalImageGap(image_gap_us);
  const bool dropped_probe_gap =
      old_state == SyncState::PROBE_SENT &&
      (IsProbeImageGap(image_gap_us) ||
       (probe_ack_seq_.load() == pending_probe_seq_ && !normal_gap));

  if (dropped_probe_gap)
  {
    ResetLock("dropped-probe-image", "publish failed");
    ResetImageObservation();
    ObserveNormalImageCadence(image_ts);
    RememberImage(image_ts);
    return;
  }

  const uint32_t image_gap_stride =
      old_state == SyncState::SYNCED ? MatchImageGapStride(image_gap_us) : 0;
  auto update = CameraFrameSyncCore::CadenceUpdate::STABLE;
  if (image_gap_stride > 1)
  {
    image_cadence_.last_timestamp_us = image_ts;
  }
  else
  {
    update = ObserveNormalImageCadence(image_ts);
  }

  if (old_state == SyncState::SYNCED && image_gap_stride != 0)
  {
    if (locked_sync_.period_us != 0 && locked_sync_.last_imu_timestamp_us != 0)
    {
      locked_sync_.last_imu_timestamp_us +=
          locked_sync_.period_us * static_cast<uint64_t>(image_gap_stride);
    }
    RememberImage(image_ts);
    return;
  }

  if (update == CameraFrameSyncCore::CadenceUpdate::BROKEN)
  {
    ResetLock("dropped-image-cadence-broken", "publish failed");
  }
  RememberImage(image_ts);
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::MaybeStartProbe()
{
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
  cmd.version = 1;
  cmd.seq = next_sync_seq_++;
  if (next_sync_seq_ == 0)
  {
    next_sync_seq_ = 1;
  }
  cmd.sync_probe_div = static_cast<uint8_t>(sync_probe_div_);
  cmd.run_trigger_div = TargetRunTriggerDiv();
  cmd.active_level = static_cast<uint8_t>(sync_active_level_);

  state_ = SyncState::PROBE_SENT;
  pending_probe_seq_ = cmd.seq;
  pending_probe_ack_valid_ = false;
  pending_probe_imu_timestamp_us_ = 0;
  pending_run_period_us_ = periods_.imu_us * static_cast<uint64_t>(cmd.run_trigger_div);
  pending_probe_start_imu_timestamp_us_ =
      imu_history_.Empty() ? 0 : imu_history_.Back().sensor_timestamp_us;
  // 回环很短时回执可能早于 Publish 返回，因此先打开当前 probe 邮箱。
  probe_ack_timestamp_us_.store(0);
  probe_ack_seq_.store(0);
  probe_ack_run_div_.store(0);
  active_probe_seq_.store(cmd.seq);

  topics_.sync_command.Publish(cmd);

  XR_LOG_INFO(
      "CameraFrameSync: sync command sent seq=%u probe_div=%u run_div=%u active=%u target_hz=%.3f image_period_us=%u imu_period_us=%u sync_period_us=%u",
      static_cast<unsigned>(cmd.seq), static_cast<unsigned>(cmd.sync_probe_div),
      static_cast<unsigned>(cmd.run_trigger_div),
      static_cast<unsigned>(cmd.active_level), static_cast<double>(target_trigger_hz_),
      static_cast<unsigned>(periods_.image_us),
      static_cast<unsigned>(periods_.imu_us),
      static_cast<unsigned>(sync_period_us));
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::TryProbeImage(
    typename CameraFrameSync<CameraInfoV>::PendingFrame& frame)
{
  const uint64_t sync_period_us = EstimatedSyncPeriodUs();
  if (!pending_probe_ack_valid_ && probe_ack_seq_.load() == pending_probe_seq_)
  {
    // 回执 timestamp 来自 IMU 时间轴，是 RAW_PROBE 的唯一跨端同步锚点。
    pending_probe_ack_valid_ = true;
    pending_probe_imu_timestamp_us_ = probe_ack_timestamp_us_.load();
    const uint32_t ack_run_div = probe_ack_run_div_.load();
    if (ack_run_div == 0 || ack_run_div > UINT8_MAX)
    {
      return ImageDecision::RESET;
    }
    pending_run_period_us_ = periods_.imu_us * static_cast<uint64_t>(ack_run_div);
  }
  if (sync_period_us == 0 || !pending_probe_ack_valid_)
  {
    if (!pending_probe_ack_valid_ && ProbeTimedOut())
    {
      ResetLock("probe-timeout", "probe gap without sync result");
      ResetImageObservation();
      ObserveNormalImageCadence(frame.image.sensor_timestamp_us);
      RememberImage(frame.image.sensor_timestamp_us);
      return ImageDecision::DONE;
    }
    return ImageDecision::WAIT;
  }

  const uint64_t ack_ts = pending_probe_imu_timestamp_us_;
  if (!ImuHistoryReached(ack_ts))
  {
    return ImageDecision::WAIT;
  }

  const AssembledImu* sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
      imu_history_, ack_ts,
      CameraFrameSyncCore::ImuTimestampToleranceUs(periods_.imu_us));
  if (sync_imu == nullptr)
  {
    return ImuHistoryReached(ack_ts + CameraFrameSyncCore::ImuTimestampToleranceUs(
                                          periods_.imu_us))
               ? ImageDecision::RESET
               : ImageDecision::WAIT;
  }

  return PublishOrRememberMatch(frame,
                                SyncMatch{.imu = sync_imu, .period_us = sync_period_us});
}

template <CameraTypes::CameraInfo CameraInfoV>
uint64_t CameraFrameSync<CameraInfoV>::ProbeTimeoutUs() const
{
  uint64_t image_period_us = periods_.image_us;
  if (image_period_us == 0)
  {
    image_period_us = EstimatedSyncPeriodUs();
  }
  if (image_period_us == 0)
  {
    return 0;
  }

  const uint64_t probe_gap_us =
      CameraFrameSyncCore::ProbeImageGapUs(image_period_us, sync_probe_div_);
  uint64_t timeout_us = probe_gap_us + image_period_us * 4ULL;
  if (timeout_us < probe_ack_timeout_min_us)
  {
    timeout_us = probe_ack_timeout_min_us;
  }
  return timeout_us;
}

template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::ProbeTimedOut() const
{
  if (state_ != SyncState::PROBE_SENT ||
      pending_probe_start_imu_timestamp_us_ == 0 || imu_history_.Empty())
  {
    return false;
  }

  const uint64_t timeout_us = ProbeTimeoutUs();
  if (timeout_us == 0 ||
      imu_history_.Back().sensor_timestamp_us <= pending_probe_start_imu_timestamp_us_)
  {
    return false;
  }

  return imu_history_.Back().sensor_timestamp_us -
             pending_probe_start_imu_timestamp_us_ >=
         timeout_us;
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::TryLatestImuMatch(
    typename CameraFrameSync<CameraInfoV>::PendingFrame& frame)
{
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
    typename CameraFrameSync<CameraInfoV>::PendingFrame& frame,
    uint32_t image_gap_stride)
{
  if (locked_sync_.last_imu_timestamp_us == 0 || locked_sync_.period_us == 0 ||
      image_gap_stride == 0)
  {
    return ImageDecision::RESET;
  }

  const uint64_t expected_sync_ts =
      locked_sync_.last_imu_timestamp_us +
      locked_sync_.period_us * static_cast<uint64_t>(image_gap_stride);
  if (!ImuHistoryReached(expected_sync_ts))
  {
    // 图像已到达，但对应的 IMU 时间点可能还没从 SharedTopic 上来。
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
  if (!frame.match.valid)
  {
    return ImageDecision::RESET;
  }

  const AssembledImu* sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
      imu_history_, frame.match.imu_timestamp_us, 0);
  if (sync_imu == nullptr)
  {
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
    // offset 只在 IMU 时间轴内生效；正 offset 会等待未来 IMU。
    return ImageDecision::WAIT;
  }

  const AssembledImu* final_imu = CameraFrameSyncCore::FindBySensorTimestamp(
      imu_history_, final_ts,
      CameraFrameSyncCore::ImuTimestampToleranceUs(periods_.imu_us));
  if (final_imu == nullptr)
  {
    return ImageDecision::RESET;
  }

  recording_.Record(image.sensor_timestamp_us, match.imu->sensor_timestamp_us,
                    final_imu->sensor_timestamp_us, final_imu->rotation_wxyz,
                    final_imu->angular_velocity_xyz,
                    final_imu->linear_acceleration_xyz, SyncModeName(sync_mode_));
  PublishSyncedImu(image.sensor_timestamp_us, *final_imu);

  const SyncState old_state = state_;
  state_ = SyncState::SYNCED;
  locked_sync_.period_us = match.period_us;
  locked_sync_.last_imu_timestamp_us = match.imu->sensor_timestamp_us;
  if (old_state == SyncState::PROBE_SENT && pending_run_period_us_ != 0)
  {
    locked_sync_.period_us = pending_run_period_us_;
    LockImageCadence(image.sensor_timestamp_us, pending_run_period_us_);
  }
  else
  {
    RememberImage(image.sensor_timestamp_us);
  }
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
  monitor_synced_output_count_.fetch_add(1, std::memory_order_relaxed);
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::LockImageCadence(uint64_t image_timestamp_us,
                                                    uint64_t image_period_us)
{
  image_cadence_.has_last_timestamp = true;
  image_cadence_.stable = true;
  image_cadence_.last_timestamp_us = image_timestamp_us;
  image_cadence_.period_us = image_period_us;
  image_cadence_.stable_count = cadence_stable_gaps;
  periods_.image_us = image_period_us;
  RememberImage(image_timestamp_us);
}

template <CameraTypes::CameraInfo CameraInfoV>
uint64_t CameraFrameSync<CameraInfoV>::EstimatedSyncPeriodUs() const
{
  if (periods_.image_us == 0 || periods_.imu_us == 0)
  {
    return 0;
  }
  const uint32_t stride = CameraFrameSyncCore::EstimateStrideSamples(
      periods_.image_us, periods_.imu_us);
  return stride == 0 ? 0 : periods_.imu_us * static_cast<uint64_t>(stride);
}

template <CameraTypes::CameraInfo CameraInfoV>
uint8_t CameraFrameSync<CameraInfoV>::TargetRunTriggerDiv() const
{
  if (periods_.imu_us == 0 || target_trigger_hz_ <= 0.0F)
  {
    return 1;
  }

  const double imu_hz = 1000000.0 / static_cast<double>(periods_.imu_us);
  const double raw_div = imu_hz / static_cast<double>(target_trigger_hz_);
  uint32_t div = static_cast<uint32_t>(raw_div + 0.5);
  if (div == 0)
  {
    div = 1;
  }
  if (div > UINT8_MAX)
  {
    div = UINT8_MAX;
  }
  return static_cast<uint8_t>(div);
}

template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::IsNormalImageGap(uint64_t image_gap_us) const
{
  return periods_.image_us != 0 &&
         CameraFrameSyncCore::AbsDiffUs(image_gap_us, periods_.image_us) <=
             CameraFrameSyncCore::ImageGapToleranceUs(periods_.image_us);
}

template <CameraTypes::CameraInfo CameraInfoV>
uint32_t CameraFrameSync<CameraInfoV>::MatchImageGapStride(uint64_t image_gap_us) const
{
  return CameraFrameSyncCore::MatchImageGapStride(
      image_gap_us, periods_.image_us, max_synced_image_gap_stride);
}

template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::IsProbeImageGap(uint64_t image_gap_us) const
{
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
  image_cadence_ = {};
  periods_.image_us = 0;
  last_image_valid_ = false;
  last_image_timestamp_us_ = 0;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ClearPendingProbe()
{
  active_probe_seq_.store(0);
  probe_ack_seq_.store(0);
  probe_ack_run_div_.store(0);
  probe_ack_timestamp_us_.store(0);
  pending_probe_seq_ = 0;
  pending_probe_ack_valid_ = false;
  pending_probe_imu_timestamp_us_ = 0;
  pending_run_period_us_ = 0;
  pending_probe_start_imu_timestamp_us_ = 0;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ClearPendingFrame()
{
  pending_frame_ = {};
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ResetLock(const char* reason, const char* detail)
{
  const SyncState old_state = state_;
  state_ = SyncState::OBSERVING;
  locked_sync_ = {};
  ClearPendingProbe();
  monitor_reset_count_.fetch_add(1, std::memory_order_relaxed);

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
  monitor_reset_count_.fetch_add(1, std::memory_order_relaxed);
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
