#pragma once

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ProcessImageEvents()
{
  while (pending_image_.valid || !image_events_.Empty())
  {
    if (!pending_image_.valid)
    {
      pending_image_.valid = true;
      pending_image_.sample = image_events_.Front();
      image_events_.PopFront();
    }

    const ImageDecision decision =
        sync_mode_ == SyncMode::LATEST_IMU ? ProcessLatestImage(pending_image_)
                                           : ProcessRawProbeImage(pending_image_);
    if (decision == ImageDecision::WAIT)
    {
      break;
    }
    if (decision == ImageDecision::RESET)
    {
      ClearPendingImage();
      ResetLock("image-rejected", StateName(state_));
      continue;
    }

    ClearPendingImage();
  }
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::ProcessLatestImage(
    typename CameraFrameSync<CameraInfoV>::PendingImage& image)
{
  if (last_image_valid_ &&
      image.sample.sensor_timestamp_us <= last_image_timestamp_us_)
  {
    ResetImageObservation();
    return ImageDecision::DONE;
  }

  const ImageDecision decision = image.sync_candidate_valid
                                     ? ResumePendingMatch(image)
                                     : TryLatestImuMatch(image);
  if (decision == ImageDecision::WAIT)
  {
    return decision;
  }

  RememberImage(image.sample.sensor_timestamp_us);
  return decision;
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::ProcessRawProbeImage(
    typename CameraFrameSync<CameraInfoV>::PendingImage& image)
{
  if (image.sync_candidate_valid)
  {
    return ResumePendingMatch(image);
  }

  if (!last_image_valid_)
  {
    if (!image.cadence_observed)
    {
      ObserveNormalImageCadence(image.sample.sensor_timestamp_us);
      image.cadence_observed = true;
    }
    RememberImage(image.sample.sensor_timestamp_us);
    MaybeStartProbe();
    return ImageDecision::DONE;
  }

  const uint64_t image_ts = image.sample.sensor_timestamp_us;
  if (image_ts <= last_image_timestamp_us_)
  {
    ResetImageObservation();
    ResetLock("image-out-of-order", "RAW_PROBE");
    ObserveNormalImageCadence(image_ts);
    RememberImage(image_ts);
    return ImageDecision::DONE;
  }

  const uint64_t image_gap_us = image_ts - last_image_timestamp_us_;
  if (state_ == SyncState::PROBE_SENT && IsProbeImageGap(image_gap_us))
  {
    // 主动探针 gap 不参与基线重估，只推进图像 cadence 的最后时间戳。
    if (!image.cadence_observed)
    {
      image_cadence_.last_timestamp_us = image_ts;
      image.cadence_observed = true;
    }
    return TryProbeImage(image);
  }

  auto cadence_update = CameraFrameSyncCore::CadenceUpdate::STABLE;
  if (!image.cadence_observed)
  {
    cadence_update = ObserveNormalImageCadence(image_ts);
    image.cadence_observed = true;
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
        RememberImage(image_ts);
        return ImageDecision::DONE;
      }
      return ImageDecision::RESET;

    case SyncState::SYNCED:
      if (!IsNormalImageGap(image_gap_us))
      {
        return ImageDecision::RESET;
      }
      return image.sync_candidate_valid ? ResumePendingMatch(image)
                                        : TrySyncedImage(image);
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
    relation_.image_period_us = image_cadence_.period_us;
  }
  return update;
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
  cmd.div = sync_probe_div_;
  cmd.active_level = sync_active_level_;
  cmd.seq = next_sync_seq_++;

  state_ = SyncState::PROBE_SENT;
  pending_probe_seq_ = cmd.seq;
  pending_probe_ack_valid_ = false;
  pending_probe_imu_timestamp_us_ = 0;
  relation_.sync_period_us = sync_period_us;
  // 先启用当前 probe 再发布命令，避免同步回环里的早到回执丢失。
  probe_ack_timestamp_us_.store(0);
  probe_ack_seq_.store(0);
  active_probe_seq_.store(cmd.seq);

  sync_command_topic_.Publish(cmd);

  XR_LOG_INFO(
      "CameraFrameSync: sync command sent seq=%u div=%u active=%u image_period_us=%u imu_period_us=%u sync_period_us=%u",
      static_cast<unsigned>(cmd.seq), static_cast<unsigned>(cmd.div),
      static_cast<unsigned>(cmd.active_level),
      static_cast<unsigned>(relation_.image_period_us),
      static_cast<unsigned>(relation_.imu_period_us),
      static_cast<unsigned>(sync_period_us));
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::TryProbeImage(
    typename CameraFrameSync<CameraInfoV>::PendingImage& image)
{
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
    return ImageDecision::WAIT;
  }

  const AssembledImu* sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
      imu_history_, ack_ts,
      CameraFrameSyncCore::ImuTimestampToleranceUs(relation_.imu_period_us));
  if (sync_imu == nullptr)
  {
    return ImuHistoryReached(ack_ts + CameraFrameSyncCore::ImuTimestampToleranceUs(
                                          relation_.imu_period_us))
               ? ImageDecision::RESET
               : ImageDecision::WAIT;
  }

  return PublishOrRememberMatch(image, SyncMatch{.sync_imu = sync_imu,
                                                 .sync_period_us = sync_period_us});
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::TryLatestImuMatch(
    typename CameraFrameSync<CameraInfoV>::PendingImage& image)
{
  if (imu_history_.Empty())
  {
    return ImageDecision::WAIT;
  }

  const uint64_t period =
      relation_.imu_period_us != 0 ? relation_.imu_period_us : 1ULL;
  return PublishOrRememberMatch(
      image, SyncMatch{.sync_imu = &imu_history_.Back(), .sync_period_us = period});
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::TrySyncedImage(
    typename CameraFrameSync<CameraInfoV>::PendingImage& image)
{
  if (relation_.last_sync_imu_timestamp_us == 0 || relation_.sync_period_us == 0)
  {
    return ImageDecision::RESET;
  }

  const uint64_t expected_sync_ts =
      relation_.last_sync_imu_timestamp_us + relation_.sync_period_us;
  if (!ImuHistoryReached(expected_sync_ts))
  {
    return ImageDecision::WAIT;
  }

  const AssembledImu* sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
      imu_history_, expected_sync_ts,
      CameraFrameSyncCore::ImuTimestampToleranceUs(relation_.imu_period_us));
  if (sync_imu == nullptr)
  {
    return ImageDecision::RESET;
  }

  return PublishOrRememberMatch(
      image,
      SyncMatch{.sync_imu = sync_imu, .sync_period_us = relation_.sync_period_us});
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::ResumePendingMatch(
    typename CameraFrameSync<CameraInfoV>::PendingImage& image)
{
  if (!image.sync_candidate_valid)
  {
    return ImageDecision::RESET;
  }

  const AssembledImu* sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
      imu_history_, image.sync_candidate_sensor_timestamp_us, 0);
  if (sync_imu == nullptr)
  {
    return ImageDecision::RESET;
  }

  return PublishOrRememberMatch(
      image, SyncMatch{.sync_imu = sync_imu,
                       .sync_period_us = image.sync_candidate_period_us});
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::PublishOrRememberMatch(
    typename CameraFrameSync<CameraInfoV>::PendingImage& image,
    const typename CameraFrameSync<CameraInfoV>::SyncMatch& match)
{
  const ImageDecision decision = PublishMatchedImage(image.sample, match);
  if (decision == ImageDecision::WAIT)
  {
    image.sync_candidate_valid = true;
    image.sync_candidate_sensor_timestamp_us = match.sync_imu->sensor_timestamp_us;
    image.sync_candidate_period_us = match.sync_period_us;
    return decision;
  }

  image.sync_candidate_valid = false;
  image.sync_candidate_sensor_timestamp_us = 0;
  image.sync_candidate_period_us = 0;
  return decision;
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageDecision
CameraFrameSync<CameraInfoV>::PublishMatchedImage(
    const typename CameraFrameSync<CameraInfoV>::ImageSample& image,
    const typename CameraFrameSync<CameraInfoV>::SyncMatch& match)
{
  if (match.sync_imu == nullptr || match.sync_period_us == 0)
  {
    return ImageDecision::RESET;
  }

  const int32_t offset_us = offset_us_;
  const uint64_t final_ts =
      CameraFrameSyncCore::ApplyOffsetUs(match.sync_imu->sensor_timestamp_us, offset_us);
  if (!ImuHistoryReached(final_ts))
  {
    return ImageDecision::WAIT;
  }

  const AssembledImu* final_imu = CameraFrameSyncCore::FindBySensorTimestamp(
      imu_history_, final_ts,
      CameraFrameSyncCore::ImuTimestampToleranceUs(relation_.imu_period_us));
  if (final_imu == nullptr)
  {
    return ImageDecision::RESET;
  }

  PublishSyncedImu(image.sensor_timestamp_us, *final_imu);
  RememberImage(image.sensor_timestamp_us);

  const SyncState old_state = state_;
  state_ = SyncState::SYNCED;
  relation_.sync_period_us = match.sync_period_us;
  relation_.last_sync_imu_timestamp_us = match.sync_imu->sensor_timestamp_us;
  ClearPendingProbe();

  if (old_state != SyncState::SYNCED)
  {
    XR_LOG_PASS(
        "CameraFrameSync: state %s -> SYNCED mode=%s image_period_us=%u imu_period_us=%u sync_period_us=%u offset_us=%d",
        StateName(old_state), SyncModeName(sync_mode_),
        static_cast<unsigned>(relation_.image_period_us),
        static_cast<unsigned>(relation_.imu_period_us),
        static_cast<unsigned>(relation_.sync_period_us), static_cast<int>(offset_us));
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
  synced_imu_topic_.Publish(synced);
}

template <CameraTypes::CameraInfo CameraInfoV>
uint64_t CameraFrameSync<CameraInfoV>::EstimatedSyncPeriodUs() const
{
  if (relation_.image_period_us == 0 || relation_.imu_period_us == 0)
  {
    return 0;
  }
  const uint32_t stride = CameraFrameSyncCore::EstimateStrideSamples(
      relation_.image_period_us, relation_.imu_period_us);
  return stride == 0 ? 0 : relation_.imu_period_us * static_cast<uint64_t>(stride);
}

template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::IsNormalImageGap(uint64_t image_gap_us) const
{
  return relation_.image_period_us != 0 &&
         CameraFrameSyncCore::AbsDiffUs(image_gap_us, relation_.image_period_us) <=
             CameraFrameSyncCore::ImageGapToleranceUs(relation_.image_period_us);
}

template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::IsProbeImageGap(uint64_t image_gap_us) const
{
  const uint64_t expected_gap =
      CameraFrameSyncCore::ProbeImageGapUs(relation_.image_period_us, sync_probe_div_);
  return expected_gap != 0 &&
         CameraFrameSyncCore::AbsDiffUs(image_gap_us, expected_gap) <=
             CameraFrameSyncCore::ImageGapToleranceUs(relation_.image_period_us);
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
  relation_.image_period_us = 0;
  last_image_valid_ = false;
  last_image_timestamp_us_ = 0;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ClearPendingProbe()
{
  active_probe_seq_.store(0);
  probe_ack_seq_.store(0);
  probe_ack_timestamp_us_.store(0);
  pending_probe_seq_ = 0;
  pending_probe_ack_valid_ = false;
  pending_probe_imu_timestamp_us_ = 0;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ClearPendingImage()
{
  pending_image_ = {};
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ResetLock(const char* reason, const char* detail)
{
  const SyncState old_state = state_;
  state_ = SyncState::OBSERVING;
  relation_.sync_period_us = 0;
  relation_.last_sync_imu_timestamp_us = 0;
  ClearPendingProbe();

  if (old_state != SyncState::OBSERVING)
  {
    XR_LOG_WARN(
        "CameraFrameSync: state %s -> OBSERVING reason=%s detail=%s image_period_us=%u imu_period_us=%u",
        StateName(old_state), reason, detail,
        static_cast<unsigned>(relation_.image_period_us),
        static_cast<unsigned>(relation_.imu_period_us));
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
  pending_image_ = {};
  image_cadence_ = {};
  imu_cadence_ = {};
  relation_ = {};
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
