#pragma once

template <CameraTypes::FrameLayout FrameLayoutV>
CameraFrameSync<FrameLayoutV>::CameraFrameSync(LibXR::HardwareContainer& hw,
                                               LibXR::ApplicationManager& app,
                                               Base* camera)
    : CameraFrameSync(hw, app, camera, RuntimeParam{})
{
}

template <CameraTypes::FrameLayout FrameLayoutV>
CameraFrameSync<FrameLayoutV>::CameraFrameSync(LibXR::HardwareContainer& hw,
                                               LibXR::ApplicationManager& app,
                                               Base& camera)
    : CameraFrameSync(hw, app, &camera, RuntimeParam{})
{
}

template <CameraTypes::FrameLayout FrameLayoutV>
CameraFrameSync<FrameLayoutV>::CameraFrameSync(LibXR::HardwareContainer&,
                                               LibXR::ApplicationManager& app,
                                               Base* camera, RuntimeParam runtime)
{
  REQUIRE(camera != nullptr);
  camera_ = camera;
  REQUIRE(!camera_->NameView().empty());
  REQUIRE(!camera_->ImageTopicNameView().empty());
  REQUIRE(!runtime.host_topic_domain_name.empty());
  REQUIRE(!runtime.sync_command_topic_name.empty());
  REQUIRE(!runtime.sync_result_topic_name.empty());
  calibration_ = camera_->Calibration();
  topics_.emplace(*camera_, runtime);
  callbacks_.emplace(this);
  pending_gyros_.emplace(pending_limit);
  pending_accls_.emplace(pending_limit);
  pending_quats_.emplace(pending_limit);
  imu_history_.emplace();

  sync_mode_ = runtime.mode;
  offset_us_ = runtime.offset_us;
  sync_active_level_ = runtime.sync_active_level == 0U ? 0U : 1U;
  camera_settle_us_ = runtime.camera_settle_us;
  raw_imu_frame_ = runtime.raw_imu_frame;

  const auto profiles = camera_->Profiles();
  REQUIRE(!profiles.empty());
  REQUIRE(profiles.size() <= 2U);
  for (std::size_t index = 0U; index < profiles.size(); ++index)
  {
    REQUIRE(profiles[index].trigger_period_us != 0U);
    REQUIRE(CameraTypes::ValidateFrameGeometry(frame_layout, calibration_,
                                               profiles[index].geometry));
    for (std::size_t other = index + 1U; other < profiles.size(); ++other)
    {
      REQUIRE(profiles[index].id != profiles[other].id);
    }
  }

  active_profile_ = profiles.front().id;
  requested_profile_ = active_profile_;
  active_trigger_period_us_ = profiles.front().trigger_period_us;
  requested_trigger_period_us_ = active_trigger_period_us_;

  topics_->gyro.RegisterCallback(callbacks_->gyro);
  topics_->accl.RegisterCallback(callbacks_->accl);
  topics_->quat.RegisterCallback(callbacks_->quat);
  topics_->sync_result.RegisterCallback(callbacks_->sync_result);

  if (sync_mode_ == SyncMode::TRIGGER)
  {
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    BeginRestartLocked(active_profile_, false);
  }
  else
  {
    control_state_ = ControlState::BYPASS;
  }
  DispatchOutbound();

  // Image delivery is registered last so every callback sees complete state.
  topics_->camera_image.RegisterCallback(callbacks_->image);
  app.Register(*this);

  XR_LOG_INFO(
      XR_PRINTF("CameraFrameSync: input=%s output=%s mode=%s profile=%u period_us=%u "
                "settle_us=%llu"),
      topics_->camera_image_name.c_str(), topics_->synced_frame_name.c_str(),
      SyncModeName(sync_mode_), static_cast<unsigned>(active_profile_),
      static_cast<unsigned>(active_trigger_period_us_),
      static_cast<unsigned long long>(camera_settle_us_));
}

template <CameraTypes::FrameLayout FrameLayoutV>
CameraFrameSync<FrameLayoutV>::CameraFrameSync(LibXR::HardwareContainer& hw,
                                               LibXR::ApplicationManager& app,
                                               Base& camera, RuntimeParam runtime)
    : CameraFrameSync(hw, app, &camera, runtime)
{
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::OnMonitor()
{
  const uint64_t raw_gyro =
      monitor_raw_gyro_count_.exchange(0U, std::memory_order_relaxed);
  const uint64_t raw_accl =
      monitor_raw_accl_count_.exchange(0U, std::memory_order_relaxed);
  const uint64_t raw_quat =
      monitor_raw_quat_count_.exchange(0U, std::memory_order_relaxed);
  const uint64_t assembled =
      monitor_assembled_imu_count_.exchange(0U, std::memory_order_relaxed);
  const uint64_t trigger = monitor_trigger_count_.exchange(0U, std::memory_order_relaxed);
  const uint64_t image =
      monitor_image_input_count_.exchange(0U, std::memory_order_relaxed);
  const uint64_t retained =
      monitor_image_retained_count_.exchange(0U, std::memory_order_relaxed);
  const uint64_t dropped =
      monitor_image_drop_count_.exchange(0U, std::memory_order_relaxed);
  const uint64_t synced =
      monitor_synced_output_count_.exchange(0U, std::memory_order_relaxed);
  const uint64_t resets = monitor_reset_count_.exchange(0U, std::memory_order_relaxed);
  const uint64_t overflows =
      monitor_overflow_count_.exchange(0U, std::memory_order_relaxed);
  const auto pending_processing = pending_processing_duration_.GetSummary();

  ControlState state{};
  ProfileId profile{};
  uint32_t period_us{};
  std::size_t held_images{};
  std::size_t pending_triggers{};
  {
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    state = control_state_;
    profile = active_profile_;
    period_us = active_trigger_period_us_;
    held_images = images_.Size() + outbound_.Size();
    pending_triggers = triggers_.Size();
  }

  XR_LOG_INFO(
      XR_PRINTF("CameraFrameSync monitor: mode=%s state=%s profile=%u period_us=%u "
                "raw=%llu/%llu/%llu assembled=%llu trigger=%llu image=%llu retained=%llu "
                "held=%llu pending_trigger=%llu drop=%llu synced=%llu reset=%llu overflow=%llu"),
      SyncModeName(sync_mode_), ControlStateName(state), static_cast<unsigned>(profile),
      static_cast<unsigned>(period_us), static_cast<unsigned long long>(raw_gyro),
      static_cast<unsigned long long>(raw_accl),
      static_cast<unsigned long long>(raw_quat),
      static_cast<unsigned long long>(assembled),
      static_cast<unsigned long long>(trigger), static_cast<unsigned long long>(image),
      static_cast<unsigned long long>(retained),
      static_cast<unsigned long long>(held_images),
      static_cast<unsigned long long>(pending_triggers),
      static_cast<unsigned long long>(dropped), static_cast<unsigned long long>(synced),
      static_cast<unsigned long long>(resets),
      static_cast<unsigned long long>(overflows));
  XR_LOG_INFO(
      XR_PRINTF("CameraFrameSync pending_processing count=%llu average_us=%llu "
                "minimum_us=%llu maximum_us=%llu"),
      static_cast<unsigned long long>(pending_processing.sample_count),
      static_cast<unsigned long long>(pending_processing.average_us),
      static_cast<unsigned long long>(pending_processing.minimum_us),
      static_cast<unsigned long long>(pending_processing.maximum_us));
}

template <CameraTypes::FrameLayout FrameLayoutV>
const char* CameraFrameSync<FrameLayoutV>::SyncedFrameTopicName() const
{
  return topics_.has_value() ? topics_->synced_frame_name.c_str() : "";
}

template <CameraTypes::FrameLayout FrameLayoutV>
const char* CameraFrameSync<FrameLayoutV>::RawTopicDomainName() const
{
  return topics_.has_value() ? topics_->host_domain_name.c_str() : "";
}

template <CameraTypes::FrameLayout FrameLayoutV>
std::span<const typename CameraFrameSync<FrameLayoutV>::CameraProfile>
CameraFrameSync<FrameLayoutV>::Profiles() const noexcept
{
  return camera_->Profiles();
}

template <CameraTypes::FrameLayout FrameLayoutV>
typename CameraFrameSync<FrameLayoutV>::ProfileId
CameraFrameSync<FrameLayoutV>::ActiveProfile() const
{
  LibXR::Mutex::LockGuard lock(sync_state_mutex_);
  return active_profile_;
}

template <CameraTypes::FrameLayout FrameLayoutV>
typename CameraFrameSync<FrameLayoutV>::SyncMode
CameraFrameSync<FrameLayoutV>::GetSyncMode() const
{
  LibXR::Mutex::LockGuard lock(sync_state_mutex_);
  return sync_mode_;
}

template <CameraTypes::FrameLayout FrameLayoutV>
const typename CameraFrameSync<FrameLayoutV>::CameraProfile*
CameraFrameSync<FrameLayoutV>::FindProfile(ProfileId profile) const noexcept
{
  for (const auto& candidate : camera_->Profiles())
  {
    if (candidate.id == profile)
    {
      return &candidate;
    }
  }
  return nullptr;
}

template <CameraTypes::FrameLayout FrameLayoutV>
LibXR::ErrorCode CameraFrameSync<FrameLayoutV>::RequestProfile(ProfileId profile)
{
  LibXR::ErrorCode result = LibXR::ErrorCode::OK;
  {
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    const CameraProfile* requested = FindProfile(profile);
    if (requested == nullptr)
    {
      result = LibXR::ErrorCode::NOT_SUPPORT;
    }
    else if (sync_mode_ != SyncMode::TRIGGER)
    {
      result = profile == active_profile_ ? LibXR::ErrorCode::OK
                                          : LibXR::ErrorCode::NOT_SUPPORT;
    }
    else if (control_state_ != ControlState::RUNNING)
    {
      result = LibXR::ErrorCode::STATE_ERR;
    }
    else if (profile == active_profile_)
    {
      result = LibXR::ErrorCode::OK;
    }
    else
    {
      BeginRestartLocked(profile, true);
      if (control_state_ == ControlState::FAILED)
      {
        result = LibXR::ErrorCode::STATE_ERR;
      }
    }
  }
  DispatchOutbound();
  return result;
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::SetOffsetUs(int32_t offset_us)
{
  LibXR::Mutex::LockGuard lock(sync_state_mutex_);
  offset_us_ = offset_us;
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::FlushPendingFrames()
{
  {
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    AssembleImuHistoryLocked();
    ProcessPendingLocked();
    images_.Clear();
    triggers_.Clear();
  }
  DispatchOutbound();
}

template <CameraTypes::FrameLayout FrameLayoutV>
const char* CameraFrameSync<FrameLayoutV>::SyncModeName(SyncMode mode)
{
  switch (mode)
  {
    case SyncMode::TRIGGER:
      return "TRIGGER";
    case SyncMode::LATEST_IMU:
      return "LATEST_IMU";
  }
  return "UNKNOWN";
}

template <CameraTypes::FrameLayout FrameLayoutV>
const char* CameraFrameSync<FrameLayoutV>::ControlStateName(ControlState state)
{
  switch (state)
  {
    case ControlState::BYPASS:
      return "BYPASS";
    case ControlState::WAIT_STOP_ACK:
      return "WAIT_STOP_ACK";
    case ControlState::SETTLING:
      return "SETTLING";
    case ControlState::SWITCHING_CAMERA:
      return "SWITCHING_CAMERA";
    case ControlState::WAIT_START_ACK:
      return "WAIT_START_ACK";
    case ControlState::RUNNING:
      return "RUNNING";
    case ControlState::FAILED:
      return "FAILED";
  }
  return "UNKNOWN";
}

template <CameraTypes::FrameLayout FrameLayoutV>
const char* CameraFrameSync<FrameLayoutV>::RawImuFrameName(RawImuFrame frame)
{
  switch (frame)
  {
    case RawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP:
      return "BODY_X_RIGHT_Y_FORWARD_Z_UP";
    case RawImuFrame::X_FORWARD_Y_LEFT_Z_UP_TO_BODY:
      return "X_FORWARD_Y_LEFT_Z_UP_TO_BODY";
  }
  return "UNKNOWN";
}

template <CameraTypes::FrameLayout FrameLayoutV>
typename CameraFrameSync<FrameLayoutV>::ImuVector
CameraFrameSync<FrameLayoutV>::ToImuVector(const RawImuVector& data, RawImuFrame frame)
{
  switch (frame)
  {
    case RawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP:
      return {data.x(), data.y(), data.z()};
    case RawImuFrame::X_FORWARD_Y_LEFT_Z_UP_TO_BODY:
      return {-data.y(), data.x(), data.z()};
  }
  return {data.x(), data.y(), data.z()};
}

template <CameraTypes::FrameLayout FrameLayoutV>
typename CameraFrameSync<FrameLayoutV>::QuatSample
CameraFrameSync<FrameLayoutV>::ToQuatSample(const RawQuatSample& data, RawImuFrame frame)
{
  switch (frame)
  {
    case RawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP:
      return {data.w(), data.x(), data.y(), data.z()};
    case RawImuFrame::X_FORWARD_Y_LEFT_Z_UP_TO_BODY:
      return {data.w(), -data.y(), data.x(), data.z()};
  }
  return {data.w(), data.x(), data.y(), data.z()};
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::OnImageStatic(bool, Self* self,
                                                  CameraImageTopicPayload borrowed)
{
  if (borrowed == nullptr || !borrowed->Valid())
  {
    return;
  }
  self->HandleImage(*borrowed);
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::HandleImage(SharedFrame image)
{
  const ImageFrame* committed = image.Get();
  if (committed == nullptr)
  {
    return;
  }

  monitor_image_input_count_.fetch_add(1U, std::memory_order_relaxed);
  const uint64_t camera_timestamp_us = static_cast<uint64_t>(committed->timestamp_us);
  const FrameGeometry geometry = committed->geometry;
  const bool geometry_valid =
      CameraTypes::ValidateFrameGeometry(frame_layout, calibration_, geometry);
  bool log_invalid_geometry = false;

  {
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    const bool accepting =
        sync_mode_ == SyncMode::LATEST_IMU || control_state_ == ControlState::RUNNING;
    if (!accepting || !geometry_valid)
    {
      if (!geometry_valid && !geometry_reject_logged_)
      {
        geometry_reject_logged_ = true;
        log_invalid_geometry = true;
      }
      monitor_image_drop_count_.fetch_add(1U, std::memory_order_relaxed);
      AutoAimReplayBenchmark::RecordSyncDrop();
    }
    else
    {
      const uint64_t sequence = next_frame_sequence_++;
      if (next_frame_sequence_ == 0U)
      {
        next_frame_sequence_ = 1U;
      }

      ImageSample sample{.camera_timestamp_us = camera_timestamp_us,
                         .sequence = sequence,
                         .frame = std::move(image)};
      if (!images_.Push(std::move(sample)))
      {
        monitor_image_drop_count_.fetch_add(1U, std::memory_order_relaxed);
        monitor_overflow_count_.fetch_add(1U, std::memory_order_relaxed);
        AutoAimReplayBenchmark::RecordSyncDrop();
      }
      else
      {
        monitor_image_retained_count_.fetch_add(1U, std::memory_order_relaxed);
        ProcessPendingLocked();
      }
    }
  }

  if (log_invalid_geometry)
  {
    XR_LOG_ERROR(
        "CameraFrameSync: invalid geometry size=%ux%u step=%u offset=%u,%u "
        "decimation=%u,%u flags=0x%x",
        static_cast<unsigned>(geometry.width), static_cast<unsigned>(geometry.height),
        static_cast<unsigned>(geometry.step),
        static_cast<unsigned>(geometry.roi_offset_x_native),
        static_cast<unsigned>(geometry.roi_offset_y_native),
        static_cast<unsigned>(geometry.decimation_x),
        static_cast<unsigned>(geometry.decimation_y),
        static_cast<unsigned>(geometry.flags));
  }
  DispatchOutbound();
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::OnGyroStatic(bool, Self* self,
                                                 LibXR::MicrosecondTimestamp timestamp,
                                                 const RawImuVector& data)
{
  const uint64_t sensor_timestamp_us = static_cast<uint64_t>(timestamp);
  const GyroSample sample{
      .sensor_timestamp_us = sensor_timestamp_us,
      .angular_velocity_xyz = ToImuVector(data, self->raw_imu_frame_)};
  self->monitor_raw_gyro_count_.fetch_add(1U, std::memory_order_relaxed);
  if (self->sync_mode_ == SyncMode::LATEST_IMU)
  {
    return;
  }

  std::optional<ProfileId> switch_profile;
  {
    LibXR::Mutex::LockGuard lock(self->sync_state_mutex_);
    const bool timestamp_backwards =
        self->latest_raw_imu_timestamp_us_ != 0U &&
        sensor_timestamp_us <= self->latest_raw_imu_timestamp_us_;
    if (timestamp_backwards)
    {
      self->ResetRawImuLocked();
      self->RestartForMismatchLocked();
    }
    self->latest_raw_imu_timestamp_us_ = sensor_timestamp_us;
    self->MaybeRetryCommandLocked(sensor_timestamp_us);
    switch_profile = self->AdvanceSettlingLocked(sensor_timestamp_us);

    if (self->pending_gyros_->PushBackDropOldest(sample))
    {
      self->monitor_overflow_count_.fetch_add(1U, std::memory_order_relaxed);
      self->ResetRawImuLocked();
      self->RestartForMismatchLocked();
    }
    else
    {
      self->AssembleImuHistoryLocked();
      self->ProcessPendingLocked();
    }
  }

  if (switch_profile.has_value())
  {
    self->ApplyProfileSwitch(*switch_profile);
  }
  self->DispatchOutbound();
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::OnAcclStatic(bool, Self* self,
                                                 LibXR::MicrosecondTimestamp timestamp,
                                                 const RawImuVector& data)
{
  const AcclSample sample{
      .sensor_timestamp_us = static_cast<uint64_t>(timestamp),
      .linear_acceleration_xyz = ToImuVector(data, self->raw_imu_frame_)};
  self->monitor_raw_accl_count_.fetch_add(1U, std::memory_order_relaxed);
  if (self->sync_mode_ == SyncMode::LATEST_IMU)
  {
    return;
  }
  {
    LibXR::Mutex::LockGuard lock(self->sync_state_mutex_);
    if (self->pending_accls_->PushBackDropOldest(sample))
    {
      self->monitor_overflow_count_.fetch_add(1U, std::memory_order_relaxed);
      self->ResetRawImuLocked();
      self->RestartForMismatchLocked();
    }
    else
    {
      self->AssembleImuHistoryLocked();
      self->ProcessPendingLocked();
    }
  }
  self->DispatchOutbound();
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::OnQuatStatic(bool, Self* self,
                                                 LibXR::MicrosecondTimestamp timestamp,
                                                 const RawQuatSample& data)
{
  const QuatReading sample{.sensor_timestamp_us = static_cast<uint64_t>(timestamp),
                           .rotation_wxyz = ToQuatSample(data, self->raw_imu_frame_)};
  self->monitor_raw_quat_count_.fetch_add(1U, std::memory_order_relaxed);
  {
    LibXR::Mutex::LockGuard lock(self->sync_state_mutex_);
    if (self->pending_quats_->PushBackDropOldest(sample))
    {
      self->monitor_overflow_count_.fetch_add(1U, std::memory_order_relaxed);
      self->ResetRawImuLocked();
      self->RestartForMismatchLocked();
    }
    else
    {
      self->AssembleImuHistoryLocked();
      self->ProcessPendingLocked();
    }
  }
  self->DispatchOutbound();
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::OnSyncResultStatic(
    bool, Self* self, LibXR::MicrosecondTimestamp timestamp,
    const CameraSync::SyncEvent& event)
{
  {
    LibXR::Mutex::LockGuard lock(self->sync_state_mutex_);
    self->HandleSyncEventLocked(static_cast<uint64_t>(timestamp), event);
    self->ProcessPendingLocked();
  }
  self->DispatchOutbound();
}

template <CameraTypes::FrameLayout FrameLayoutV>
uint8_t CameraFrameSync<FrameLayoutV>::AllocateSyncSequence()
{
  const uint8_t sequence = next_sync_seq_++;
  if (next_sync_seq_ == 0U)
  {
    next_sync_seq_ = 1U;
  }
  return sequence;
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::BeginRestartLocked(ProfileId profile,
                                                       bool switch_profile)
{
  const CameraProfile* requested = FindProfile(profile);
  if (requested == nullptr)
  {
    FailControlLocked();
    return;
  }

  ResetMatchingLocked();
  requested_profile_ = profile;
  requested_trigger_period_us_ = requested->trigger_period_us;
  switch_profile_after_settle_ = switch_profile;
  settle_deadline_us_ = 0U;
  ClearCommandLocked();
  control_state_ = ControlState::WAIT_STOP_ACK;
  if (!QueueCommandLocked(CameraSync::Operation::STOP_TRIGGER, 0U))
  {
    FailControlLocked();
  }
  monitor_reset_count_.fetch_add(1U, std::memory_order_relaxed);
}

template <CameraTypes::FrameLayout FrameLayoutV>
bool CameraFrameSync<FrameLayoutV>::QueueCommandLocked(CameraSync::Operation operation,
                                                       uint32_t trigger_period_us)
{
  CameraSync::SyncCommand command{};
  command.operation = operation;
  command.active_level = sync_active_level_;
  command.seq = AllocateSyncSequence();
  command.reserved = 0U;
  command.trigger_period_us = trigger_period_us;

  OutboundItem output{};
  output.kind = OutboundKind::SYNC_COMMAND;
  output.command = command;
  if (!QueueOutboundLocked(std::move(output)))
  {
    return false;
  }
  ArmCommandLocked(command);
  return true;
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::ArmCommandLocked(
    const CameraSync::SyncCommand& command)
{
  pending_command_ = command;
  command_waiting_ack_ = true;
  command_pending_dispatch_ = true;
  command_retry_count_ = 0U;
  command_last_dispatch_imu_timestamp_us_ = 0U;
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::ClearCommandLocked()
{
  pending_command_ = {};
  command_waiting_ack_ = false;
  command_pending_dispatch_ = false;
  command_retry_count_ = 0U;
  command_last_dispatch_imu_timestamp_us_ = 0U;
}

template <CameraTypes::FrameLayout FrameLayoutV>
bool CameraFrameSync<FrameLayoutV>::IsPendingCommandLocked(
    const CameraSync::SyncCommand& command) const
{
  return command_waiting_ack_ && command_pending_dispatch_ &&
         command.operation == pending_command_.operation &&
         command.active_level == pending_command_.active_level &&
         command.seq == pending_command_.seq &&
         command.reserved == pending_command_.reserved &&
         command.trigger_period_us == pending_command_.trigger_period_us;
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::MaybeRetryCommandLocked(uint64_t gyro_timestamp_us)
{
  if (!command_waiting_ack_ || command_pending_dispatch_)
  {
    return;
  }
  if (command_last_dispatch_imu_timestamp_us_ == 0U)
  {
    command_last_dispatch_imu_timestamp_us_ = gyro_timestamp_us;
    return;
  }
  if (gyro_timestamp_us < command_last_dispatch_imu_timestamp_us_ ||
      gyro_timestamp_us - command_last_dispatch_imu_timestamp_us_ <
          command_retry_interval_us)
  {
    return;
  }
  if (command_retry_count_ >= command_retry_limit)
  {
    FailControlLocked();
    return;
  }

  OutboundItem output{};
  output.kind = OutboundKind::SYNC_COMMAND;
  output.command = pending_command_;
  if (!QueueOutboundLocked(std::move(output)))
  {
    FailControlLocked();
    return;
  }
  command_pending_dispatch_ = true;
  ++command_retry_count_;
}

template <CameraTypes::FrameLayout FrameLayoutV>
std::optional<typename CameraFrameSync<FrameLayoutV>::ProfileId>
CameraFrameSync<FrameLayoutV>::AdvanceSettlingLocked(uint64_t gyro_timestamp_us)
{
  if (control_state_ != ControlState::SETTLING || gyro_timestamp_us < settle_deadline_us_)
  {
    return std::nullopt;
  }

  if (switch_profile_after_settle_)
  {
    control_state_ = ControlState::SWITCHING_CAMERA;
    return requested_profile_;
  }

  QueueStartLocked();
  return std::nullopt;
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::ApplyProfileSwitch(ProfileId profile)
{
  AppliedProfile applied{};
  LibXR::ErrorCode result = camera_->SwitchProfile(profile, applied);
  if (result == LibXR::ErrorCode::OK &&
      (applied.id != profile ||
       !CameraTypes::ValidateFrameGeometry(frame_layout, calibration_, applied.geometry)))
  {
    result = LibXR::ErrorCode::STATE_ERR;
  }

  {
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    if (control_state_ != ControlState::SWITCHING_CAMERA || requested_profile_ != profile)
    {
      return;
    }
    if (result != LibXR::ErrorCode::OK)
    {
      FailControlLocked();
      return;
    }

    active_profile_ = applied.id;
    active_trigger_period_us_ = requested_trigger_period_us_;
    switch_profile_after_settle_ = false;
    QueueStartLocked();
  }
  DispatchOutbound();
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::QueueStartLocked()
{
  ResetMatchingLocked();
  control_state_ = ControlState::WAIT_START_ACK;
  if (!QueueCommandLocked(CameraSync::Operation::START_TRIGGER,
                          requested_trigger_period_us_))
  {
    FailControlLocked();
  }
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::FailControlLocked()
{
  control_state_ = ControlState::FAILED;
  ClearCommandLocked();
  ResetMatchingLocked();
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::RestartForMismatchLocked()
{
  if (sync_mode_ == SyncMode::TRIGGER && control_state_ == ControlState::RUNNING)
  {
    BeginRestartLocked(active_profile_, false);
  }
  else
  {
    ResetMatchingLocked();
  }
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::ResetMatchingLocked()
{
  images_.Clear();
  triggers_.Clear();
  trigger_stream_valid_ = false;
  last_received_trigger_sequence_ = 0U;
  last_received_trigger_timestamp_us_ = 0U;
  matched_pair_valid_ = false;
  last_matched_camera_timestamp_us_ = 0U;
  last_matched_trigger_sequence_ = 0U;
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::ResetRawImuLocked()
{
  pending_gyros_->Clear();
  pending_accls_->Clear();
  pending_quats_->Clear();
  imu_history_->Clear();
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::HandleSyncEventLocked(
    uint64_t event_timestamp_us, const CameraSync::SyncEvent& event)
{
  if (event.operation == CameraSync::Operation::STOP_TRIGGER &&
      control_state_ == ControlState::WAIT_STOP_ACK && command_waiting_ack_ &&
      pending_command_.operation == CameraSync::Operation::STOP_TRIGGER &&
      event.seq == pending_command_.seq)
  {
    const bool valid = event.active_level == sync_active_level_ && event.reserved == 0U &&
                       event.effective_period_us == 0U;
    if (!valid)
    {
      FailControlLocked();
      return;
    }
    ClearCommandLocked();
    ResetMatchingLocked();
    control_state_ = ControlState::SETTLING;
    settle_deadline_us_ = event_timestamp_us > UINT64_MAX - camera_settle_us_
                              ? UINT64_MAX
                              : event_timestamp_us + camera_settle_us_;
    return;
  }

  if (event.operation == CameraSync::Operation::START_TRIGGER &&
      control_state_ == ControlState::WAIT_START_ACK && command_waiting_ack_ &&
      pending_command_.operation == CameraSync::Operation::START_TRIGGER &&
      event.seq == pending_command_.seq)
  {
    const bool valid = event.active_level == sync_active_level_ && event.reserved == 0U &&
                       event.effective_period_us == requested_trigger_period_us_ &&
                       event.trigger_sequence == 0U;
    if (!valid)
    {
      FailControlLocked();
      return;
    }
    active_start_seq_ = event.seq;
    active_trigger_period_us_ = event.effective_period_us;
    ClearCommandLocked();
    ResetMatchingLocked();
    control_state_ = ControlState::RUNNING;
    return;
  }

  if (event.operation != CameraSync::Operation::FRAME_TRIGGER ||
      sync_mode_ != SyncMode::TRIGGER || control_state_ != ControlState::RUNNING)
  {
    return;
  }

  const uint32_t expected_sequence =
      trigger_stream_valid_ ? last_received_trigger_sequence_ + 1U : 1U;
  const bool sequence_wrapped = trigger_stream_valid_ && expected_sequence == 0U;
  const bool valid = !sequence_wrapped && event.seq == active_start_seq_ &&
                     event.active_level == sync_active_level_ && event.reserved == 0U &&
                     event.effective_period_us == active_trigger_period_us_ &&
                     event.trigger_sequence == expected_sequence &&
                     (!trigger_stream_valid_ ||
                      event_timestamp_us > last_received_trigger_timestamp_us_);
  if (!valid)
  {
    RestartForMismatchLocked();
    return;
  }

  TriggerSample trigger{.imu_timestamp_us = event_timestamp_us,
                        .sequence = event.trigger_sequence};
  if (!triggers_.Push(trigger))
  {
    monitor_overflow_count_.fetch_add(1U, std::memory_order_relaxed);
    RestartForMismatchLocked();
    return;
  }
  trigger_stream_valid_ = true;
  last_received_trigger_sequence_ = event.trigger_sequence;
  last_received_trigger_timestamp_us_ = event_timestamp_us;
  monitor_trigger_count_.fetch_add(1U, std::memory_order_relaxed);
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::AssembleImuHistoryLocked()
{
  while (TryAssembleOneImuLocked())
  {
  }
}

template <CameraTypes::FrameLayout FrameLayoutV>
bool CameraFrameSync<FrameLayoutV>::TryAssembleOneImuLocked()
{
  if (sync_mode_ == SyncMode::LATEST_IMU)
  {
    QuatReading quat{};
    if (!pending_quats_->Front(quat))
    {
      return false;
    }
    pending_quats_->PopFront();
    AcceptAssembledImuLocked({.sensor_timestamp_us = quat.sensor_timestamp_us,
                              .rotation_wxyz = quat.rotation_wxyz,
                              .angular_velocity_xyz = {0.0F, 0.0F, 0.0F},
                              .linear_acceleration_xyz = {0.0F, 0.0F, 0.0F}});
    return true;
  }

  GyroSample gyro{};
  if (!pending_gyros_->Front(gyro) || pending_accls_->Empty() || pending_quats_->Empty())
  {
    return false;
  }

  AcclSample accl{};
  while (pending_accls_->Front(accl) &&
         accl.sensor_timestamp_us < gyro.sensor_timestamp_us)
  {
    pending_accls_->PopFront();
  }
  QuatReading quat{};
  while (pending_quats_->Front(quat) &&
         quat.sensor_timestamp_us < gyro.sensor_timestamp_us)
  {
    pending_quats_->PopFront();
  }
  if (!pending_accls_->Front(accl) || !pending_quats_->Front(quat))
  {
    return false;
  }
  if (accl.sensor_timestamp_us != gyro.sensor_timestamp_us ||
      quat.sensor_timestamp_us != gyro.sensor_timestamp_us)
  {
    pending_gyros_->PopFront();
    return true;
  }

  pending_gyros_->PopFront();
  pending_accls_->PopFront();
  pending_quats_->PopFront();
  AcceptAssembledImuLocked({.sensor_timestamp_us = gyro.sensor_timestamp_us,
                            .rotation_wxyz = quat.rotation_wxyz,
                            .angular_velocity_xyz = gyro.angular_velocity_xyz,
                            .linear_acceleration_xyz = accl.linear_acceleration_xyz});
  return true;
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::AcceptAssembledImuLocked(const AssembledImu& imu)
{
  if (!imu_history_->Empty() &&
      imu.sensor_timestamp_us <= imu_history_->Back().sensor_timestamp_us)
  {
    imu_history_->Clear();
    RestartForMismatchLocked();
  }
  static_cast<void>(imu_history_->PushBackDropOldest(imu));
  monitor_assembled_imu_count_.fetch_add(1U, std::memory_order_relaxed);
}

template <CameraTypes::FrameLayout FrameLayoutV>
bool CameraFrameSync<FrameLayoutV>::QueueOutboundLocked(OutboundItem item)
{
  return outbound_.Push(std::move(item));
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::DispatchOutbound()
{
  {
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    if (dispatching_outbound_)
    {
      return;
    }
    dispatching_outbound_ = true;
  }

  while (true)
  {
    OutboundItem output{};
    bool discard = false;
    bool command = false;
    {
      LibXR::Mutex::LockGuard lock(sync_state_mutex_);
      if (!outbound_.Pop(output))
      {
        dispatching_outbound_ = false;
        return;
      }
      command = output.kind == OutboundKind::SYNC_COMMAND;
      if (command)
      {
        discard = !IsPendingCommandLocked(output.command);
      }
    }

    if (discard)
    {
      continue;
    }
    if (command)
    {
      topics_->sync_command.Publish(output.command);
      LibXR::Mutex::LockGuard lock(sync_state_mutex_);
      if (IsPendingCommandLocked(output.command))
      {
        command_pending_dispatch_ = false;
        command_last_dispatch_imu_timestamp_us_ = latest_raw_imu_timestamp_us_;
      }
      continue;
    }

    ASSERT(output.frame.Valid());
    SyncedFrameTopicPayload payload = &output.frame;
    topics_->synced_frame.Publish(payload);
    AutoAimReplayBenchmark::RecordSync(
        static_cast<uint64_t>(output.frame.imu.timestamp_us));
    monitor_synced_output_count_.fetch_add(1U, std::memory_order_relaxed);
  }
}

template <CameraTypes::FrameLayout FrameLayoutV>
void CameraFrameSync<FrameLayoutV>::ProcessSyncWorkWithoutImage()
{
  {
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    AssembleImuHistoryLocked();
    ProcessPendingLocked();
  }
  DispatchOutbound();
}
