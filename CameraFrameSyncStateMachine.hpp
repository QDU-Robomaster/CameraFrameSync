#pragma once

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::ProcessPendingLocked()
{
  auto pending_processing_measurement = pending_processing_duration_.Measure();
  if (sync_mode_ == SyncMode::LATEST_IMU)
  {
    ProcessLatestMatchesLocked();
    return;
  }
  if (control_state_ == ControlState::RUNNING)
  {
    ProcessTriggerMatchesLocked();
  }
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::ProcessTriggerMatchesLocked()
{
  while (!images_.Empty() && !triggers_.Empty())
  {
    ImageSample* image = images_.Front();
    ASSERT(image != nullptr);

    uint64_t target_sequence = 0U;
    if (!matched_pair_valid_)
    {
      const TriggerSample* first_trigger = triggers_.Front();
      ASSERT(first_trigger != nullptr);
      target_sequence = first_trigger->sequence;
    }
    else
    {
      if (image->camera_timestamp_us <= last_matched_camera_timestamp_us_)
      {
        RestartForMismatchLocked();
        return;
      }
      const uint64_t camera_gap_us =
          image->camera_timestamp_us - last_matched_camera_timestamp_us_;
      const uint32_t stride = CameraFrameSyncCore::MatchImageGapStride(
          camera_gap_us, active_trigger_period_us_, max_camera_gap_stride);
      if (stride == 0U)
      {
        RestartForMismatchLocked();
        return;
      }
      target_sequence = static_cast<uint64_t>(last_matched_trigger_sequence_) + stride;
      if (target_sequence > UINT32_MAX)
      {
        RestartForMismatchLocked();
        return;
      }
    }

    while (const TriggerSample* skipped = triggers_.Front())
    {
      if (skipped->sequence >= target_sequence)
      {
        break;
      }
      const bool popped = triggers_.PopFront();
      ASSERT(popped);
    }

    const TriggerSample* trigger = triggers_.Front();
    if (trigger == nullptr)
    {
      return;
    }
    if (trigger->sequence != target_sequence)
    {
      RestartForMismatchLocked();
      return;
    }

    const AssembledImu* imu = nullptr;
    const ImuLookup lookup = FindImuLocked(trigger->imu_timestamp_us, imu);
    if (lookup == ImuLookup::WAIT)
    {
      return;
    }
    if (lookup == ImuLookup::MISSING || imu == nullptr)
    {
      RestartForMismatchLocked();
      return;
    }

    const uint64_t camera_timestamp_us = image->camera_timestamp_us;
    const uint32_t trigger_sequence = trigger->sequence;
    if (!QueueSyncedFrameLocked(*image, *imu, trigger->imu_timestamp_us))
    {
      monitor_image_drop_count_.fetch_add(1U, std::memory_order_relaxed);
      AutoAimReplayBenchmark::RecordSyncDrop();
    }
    CompleteMatchedImageLocked(camera_timestamp_us, trigger_sequence);
    const bool image_popped = images_.PopFront();
    const bool trigger_popped = triggers_.PopFront();
    ASSERT(image_popped && trigger_popped);
  }
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::ProcessLatestMatchesLocked()
{
  while (!images_.Empty() && !imu_history_->Empty())
  {
    ImageSample* image = images_.Front();
    ASSERT(image != nullptr);
    const AssembledImu& imu = imu_history_->Back();
    if (!QueueSyncedFrameLocked(*image, imu, imu.sensor_timestamp_us))
    {
      monitor_image_drop_count_.fetch_add(1U, std::memory_order_relaxed);
      AutoAimReplayBenchmark::RecordSyncDrop();
    }
    const bool popped = images_.PopFront();
    ASSERT(popped);
  }
}

template <CameraTypes::FrameLayout FrameLayoutV>
typename CameraFrameSync<FrameLayoutV>::ImuLookup
CameraFrameSync<FrameLayoutV>::FindImuLocked(uint64_t trigger_timestamp_us,
                                             const AssembledImu*& imu) const
{
  imu = nullptr;
  if (imu_history_->Empty())
  {
    return ImuLookup::WAIT;
  }

  const uint64_t target_timestamp_us =
      CameraFrameSyncCore::ApplyOffsetUs(trigger_timestamp_us, offset_us_);
  const uint64_t tolerance_us = offset_us_ == 0 ? 0U : offset_lookup_tolerance_us;
  if (imu_history_->Back().sensor_timestamp_us < target_timestamp_us)
  {
    return ImuLookup::WAIT;
  }

  imu = CameraFrameSyncCore::FindBySensorTimestamp(*imu_history_, target_timestamp_us,
                                                   tolerance_us);
  return imu == nullptr ? ImuLookup::MISSING : ImuLookup::FOUND;
}

template <CameraTypes::FrameLayout FrameLayoutV>
bool CameraFrameSync<FrameLayoutV>::QueueSyncedFrameLocked(
    ImageSample& image, const AssembledImu& imu, uint64_t authoritative_timestamp_us)
{
  if (!image.frame.Valid() || (last_output_timestamp_valid_ &&
                               authoritative_timestamp_us <= last_output_timestamp_us_))
  {
    return false;
  }

  ImuStamped stamped{
      .timestamp_us = authoritative_timestamp_us,
      .rotation_wxyz = imu.rotation_wxyz,
      .translation_xyz = {0.0F, 0.0F, 0.0F},
      .angular_velocity_xyz = imu.angular_velocity_xyz,
      .linear_acceleration_xyz = imu.linear_acceleration_xyz,
  };
  OutboundItem output{};
  output.kind = OutboundKind::SYNCED_FRAME;
  output.frame = {
      .sequence = image.sequence, .image = std::move(image.frame), .imu = stamped};
  if (!QueueOutboundLocked(std::move(output)))
  {
    return false;
  }

  last_output_timestamp_valid_ = true;
  last_output_timestamp_us_ = authoritative_timestamp_us;
  return true;
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::CompleteMatchedImageLocked(
    uint64_t camera_timestamp_us, uint32_t trigger_sequence)
{
  matched_pair_valid_ = true;
  last_matched_camera_timestamp_us_ = camera_timestamp_us;
  last_matched_trigger_sequence_ = trigger_sequence;
}
