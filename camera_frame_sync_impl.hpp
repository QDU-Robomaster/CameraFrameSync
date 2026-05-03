#pragma once

template <CameraTypes::CameraInfo CameraInfoV>
CameraFrameSync<CameraInfoV>::CameraFrameSync(
    LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
    typename CameraFrameSync<CameraInfoV>::Base& camera)
    : CameraFrameSync(hw, app, camera, RuntimeParam{})
{
}

template <CameraTypes::CameraInfo CameraInfoV>
CameraFrameSync<CameraInfoV>::CameraFrameSync(
    LibXR::HardwareContainer&, LibXR::ApplicationManager&,
    typename CameraFrameSync<CameraInfoV>::Base& camera,
    typename CameraFrameSync<CameraInfoV>::RuntimeParam runtime)
    : image_topic_name_(camera.ImageTopicNameView()),
      imu_topic_name_(camera.ImuTopicNameView()),
      host_topic_domain_name_(runtime.host_topic_domain_name),
      host_topic_domain_(host_topic_domain_name_.c_str()),
      sync_command_topic_name_(runtime.sync_command_topic_name),
      sync_result_topic_name_(runtime.sync_result_topic_name),
      sensor_name_(RequireSensorName(camera.NameView())),
      gyro_topic_name_(sensor_name_ + "_gyro"),
      accl_topic_name_(sensor_name_ + "_accl"),
      quat_topic_name_(sensor_name_ + "_quat"),
      image_topic_(image_topic_name_.c_str(), image_topic_config),
      synced_imu_topic_(LibXR::Topic::FindOrCreate<ImuStamped>(
          imu_topic_name_.c_str(), &host_topic_domain_)),
      sync_command_topic_(LibXR::Topic::FindOrCreate<CameraSync::SyncCommand>(
          sync_command_topic_name_.c_str(), &host_topic_domain_)),
      sync_result_topic_(LibXR::Topic::FindOrCreate<CameraSync::SyncEvent>(
          sync_result_topic_name_.c_str(), &host_topic_domain_)),
      gyro_topic_(LibXR::Topic::FindOrCreate<RawImuVector>(gyro_topic_name_.c_str(),
                                                           &host_topic_domain_)),
      accl_topic_(LibXR::Topic::FindOrCreate<RawImuVector>(accl_topic_name_.c_str(),
                                                           &host_topic_domain_)),
      quat_topic_(LibXR::Topic::FindOrCreate<RawQuatSample>(quat_topic_name_.c_str(),
                                                            &host_topic_domain_)),
      gyro_cb_(LibXR::Topic::Callback::Create(OnGyroStatic, this)),
      accl_cb_(LibXR::Topic::Callback::Create(OnAcclStatic, this)),
      quat_cb_(LibXR::Topic::Callback::Create(OnQuatStatic, this)),
      sync_result_cb_(LibXR::Topic::Callback::Create(OnSyncResultStatic, this)),
      sync_mode_(runtime.mode),
      offset_us_(runtime.offset_us),
      sync_probe_div_(runtime.sync_probe_div),
      sync_active_level_(runtime.sync_active_level == 0 ? 0U : 1U)
{
  ASSERT(sync_probe_div_ != 0);

  if (!image_topic_.Valid())
  {
    char message[96] = {};
    std::snprintf(message, sizeof(message),
                  "CameraFrameSync: image topic creation failed (err=%d)",
                  static_cast<int>(image_topic_.GetError()));
    throw std::runtime_error(message);
  }
  if (!AcquireInitialWritableImage())
  {
    throw std::runtime_error("CameraFrameSync: initial image slot acquisition failed");
  }
  if (!camera.RegisterImageSink(current_image_.GetData(),
                                ImageCommitCallback::Create(CommitImageAdapter, this)))
  {
    current_image_.Reset();
    throw std::runtime_error("CameraFrameSync: image sink registration failed");
  }

  gyro_topic_.RegisterCallback(gyro_cb_);
  accl_topic_.RegisterCallback(accl_cb_);
  quat_topic_.RegisterCallback(quat_cb_);
  sync_result_topic_.RegisterCallback(sync_result_cb_);

  XR_LOG_INFO(
      "CameraFrameSync: enabled sensor=%s domain=%s image=%s imu=%s raw=%s/%s/%s mode=%s",
      sensor_name_.c_str(), host_topic_domain_name_.c_str(),
      image_topic_name_.c_str(), imu_topic_name_.c_str(), gyro_topic_name_.c_str(),
      accl_topic_name_.c_str(), quat_topic_name_.c_str(), SyncModeName(sync_mode_));
}

template <CameraTypes::CameraInfo CameraInfoV>
const char* CameraFrameSync<CameraInfoV>::ImageTopicName() const
{
  return image_topic_name_.c_str();
}

template <CameraTypes::CameraInfo CameraInfoV>
const char* CameraFrameSync<CameraInfoV>::ImuTopicName() const
{
  return imu_topic_name_.c_str();
}

template <CameraTypes::CameraInfo CameraInfoV>
const char* CameraFrameSync<CameraInfoV>::HostTopicDomainName() const
{
  return host_topic_domain_name_.c_str();
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::SyncMode
CameraFrameSync<CameraInfoV>::GetSyncMode() const
{
  return sync_mode_;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::SetOffsetUs(int32_t offset_us)
{
  LibXR::Mutex::LockGuard lock(sync_state_mutex_);
  offset_us_ = offset_us;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::SetSyncMode(
    typename CameraFrameSync<CameraInfoV>::SyncMode mode)
{
  LibXR::Mutex::LockGuard lock(sync_state_mutex_);
  if (sync_mode_ == mode)
  {
    return;
  }

  XR_LOG_INFO("CameraFrameSync: mode %s -> %s, runtime state reset",
              SyncModeName(sync_mode_), SyncModeName(mode));
  sync_mode_ = mode;
  ResetRuntimeState();
}

template <CameraTypes::CameraInfo CameraInfoV>
std::string CameraFrameSync<CameraInfoV>::RequireSensorName(std::string_view name)
{
  if (name.empty())
  {
    throw std::runtime_error("CameraFrameSync: camera name is required");
  }
  return std::string(name);
}

template <CameraTypes::CameraInfo CameraInfoV>
const char* CameraFrameSync<CameraInfoV>::SyncModeName(
    typename CameraFrameSync<CameraInfoV>::SyncMode mode)
{
  switch (mode)
  {
    case SyncMode::RAW_PROBE:
      return "RAW_PROBE";
    case SyncMode::LATEST_IMU:
      return "LATEST_IMU";
  }
  return "UNKNOWN";
}

template <CameraTypes::CameraInfo CameraInfoV>
const char* CameraFrameSync<CameraInfoV>::StateName(
    typename CameraFrameSync<CameraInfoV>::SyncState state)
{
  switch (state)
  {
    case SyncState::OBSERVING:
      return "OBSERVING";
    case SyncState::PROBE_SENT:
      return "PROBE_SENT";
    case SyncState::SYNCED:
      return "SYNCED";
  }
  return "UNKNOWN";
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImuVector
CameraFrameSync<CameraInfoV>::ToImuVector(
    const typename CameraFrameSync<CameraInfoV>::RawImuVector& data)
{
  return {data.x(), data.y(), data.z()};
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::QuatSample
CameraFrameSync<CameraInfoV>::ToQuatSample(
    const typename CameraFrameSync<CameraInfoV>::RawQuatSample& data)
{
  return {data.w(), data.x(), data.y(), data.z()};
}

template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::AcquireInitialWritableImage()
{
  if (image_topic_.CreateData(current_image_) != LibXR::ErrorCode::OK)
  {
    return false;
  }
  return current_image_.GetData() != nullptr;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::CommitImageAdapter(
    bool, CameraFrameSync<CameraInfoV>* self,
    typename CameraFrameSync<CameraInfoV>::ImageFrame*& next_image)
{
  next_image = self->CommitImageAndLeaseNext();
}

template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageFrame*
CameraFrameSync<CameraInfoV>::CommitImageAndLeaseNext()
{
  ImageFrame* committed_image = current_image_.GetData();
  if (committed_image == nullptr)
  {
    return nullptr;
  }

  ImageData next_image;
  if (image_topic_.CreateData(next_image) != LibXR::ErrorCode::OK ||
      next_image.GetData() == nullptr)
  {
    ProcessSyncWorkWithoutImage();
    return committed_image;
  }

  const uint64_t image_timestamp_us =
      static_cast<uint64_t>(committed_image->timestamp_us);
  const auto publish_ans = image_topic_.Publish(current_image_);
  current_image_ = std::move(next_image);

  if (publish_ans == LibXR::ErrorCode::OK)
  {
    ProcessCommittedImage(image_timestamp_us);
  }
  else
  {
    XR_LOG_WARN("CameraFrameSync: image publish failed err=%d",
                static_cast<int>(publish_ans));
    ProcessSyncWorkWithoutImage();
  }
  return current_image_.GetData();
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::OnGyroStatic(
    bool, CameraFrameSync<CameraInfoV>* self, LibXR::MicrosecondTimestamp timestamp,
    const typename CameraFrameSync<CameraInfoV>::RawImuVector& data)
{
  const QueuedGyro sample{.sample = {.sensor_timestamp_us =
                                         static_cast<uint64_t>(timestamp),
                                     .angular_velocity_xyz = ToImuVector(data)}};
  if (self->gyro_ingress_.Push(sample) != LibXR::ErrorCode::OK)
  {
    self->overflowed_.store(true, std::memory_order_relaxed);
  }
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::OnAcclStatic(
    bool, CameraFrameSync<CameraInfoV>* self, LibXR::MicrosecondTimestamp timestamp,
    const typename CameraFrameSync<CameraInfoV>::RawImuVector& data)
{
  const QueuedAccl sample{.sample = {.sensor_timestamp_us =
                                         static_cast<uint64_t>(timestamp),
                                     .linear_acceleration_xyz = ToImuVector(data)}};
  if (self->accl_ingress_.Push(sample) != LibXR::ErrorCode::OK)
  {
    self->overflowed_.store(true, std::memory_order_relaxed);
  }
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::OnQuatStatic(
    bool, CameraFrameSync<CameraInfoV>* self, LibXR::MicrosecondTimestamp timestamp,
    const typename CameraFrameSync<CameraInfoV>::RawQuatSample& data)
{
  const QueuedQuat sample{.sample = {.sensor_timestamp_us =
                                         static_cast<uint64_t>(timestamp),
                                     .rotation_wxyz = ToQuatSample(data)}};
  if (self->quat_ingress_.Push(sample) != LibXR::ErrorCode::OK)
  {
    self->overflowed_.store(true, std::memory_order_relaxed);
  }
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::OnSyncResultStatic(
    bool, CameraFrameSync<CameraInfoV>* self, LibXR::MicrosecondTimestamp timestamp,
    const CameraSync::SyncEvent& event)
{
  const uint32_t active_seq = self->active_probe_seq_.load();
  if (active_seq == 0 || event.seq != active_seq)
  {
    return;
  }
  if (self->probe_ack_seq_.load() == event.seq)
  {
    return;
  }

  self->probe_ack_timestamp_us_.store(static_cast<uint64_t>(timestamp));
  self->probe_ack_seq_.store(event.seq);
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ProcessCommittedImage(uint64_t image_timestamp_us)
{
  LibXR::Mutex::LockGuard lock(sync_state_mutex_);
  CollectIncomingTopics();
  if (image_events_.PushBackDropOldest({.sensor_timestamp_us = image_timestamp_us}))
  {
    overflowed_.store(true, std::memory_order_relaxed);
  }
  HandleOverflowRecovery();
  ProcessImageEvents();
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ProcessSyncWorkWithoutImage()
{
  LibXR::Mutex::LockGuard lock(sync_state_mutex_);
  CollectIncomingTopics();
  HandleOverflowRecovery();
  ProcessImageEvents();
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::CollectIncomingTopics()
{
  QueuedGyro gyro{};
  while (gyro_ingress_.Pop(gyro) == LibXR::ErrorCode::OK)
  {
    if (pending_gyros_.PushBackDropOldest(gyro))
    {
      overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  QueuedAccl accl{};
  while (accl_ingress_.Pop(accl) == LibXR::ErrorCode::OK)
  {
    if (pending_accls_.PushBackDropOldest(accl))
    {
      overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  QueuedQuat quat{};
  while (quat_ingress_.Pop(quat) == LibXR::ErrorCode::OK)
  {
    if (pending_quats_.PushBackDropOldest(quat))
    {
      overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  AssembleImuHistory();
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::AssembleImuHistory()
{
  while (TryAssembleOneImu())
  {
  }
}

template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::TryAssembleOneImu()
{
  QueuedGyro queued_gyro{};
  if (!pending_gyros_.Front(queued_gyro))
  {
    return false;
  }
  if (pending_accls_.Empty() || pending_quats_.Empty())
  {
    return false;
  }

  const uint64_t gyro_ts = queued_gyro.sample.sensor_timestamp_us;
  QueuedAccl queued_accl{};
  while (pending_accls_.Front(queued_accl) &&
         queued_accl.sample.sensor_timestamp_us < gyro_ts)
  {
    pending_accls_.PopFront();
  }

  QueuedQuat queued_quat{};
  while (pending_quats_.Front(queued_quat) &&
         queued_quat.sample.sensor_timestamp_us < gyro_ts)
  {
    pending_quats_.PopFront();
  }
  if (!pending_accls_.Front(queued_accl) || !pending_quats_.Front(queued_quat))
  {
    return false;
  }

  const uint64_t accl_ts = queued_accl.sample.sensor_timestamp_us;
  const uint64_t quat_ts = queued_quat.sample.sensor_timestamp_us;
  if (accl_ts > gyro_ts || quat_ts > gyro_ts)
  {
    pending_gyros_.PopFront();
    ResetLock("raw-imu-missing-channel", "exact timestamp join failed");
    return true;
  }

  const auto& gyro = queued_gyro.sample;
  const auto& accl = queued_accl.sample;
  const auto& quat = queued_quat.sample;
  const AssembledImu imu{
      .sensor_timestamp_us = gyro.sensor_timestamp_us,
      .rotation_wxyz = quat.rotation_wxyz,
      .angular_velocity_xyz = gyro.angular_velocity_xyz,
      .linear_acceleration_xyz = accl.linear_acceleration_xyz,
  };

  pending_gyros_.PopFront();
  pending_accls_.PopFront();
  pending_quats_.PopFront();

  if (!imu_history_.Empty() &&
      imu.sensor_timestamp_us <= imu_history_.Back().sensor_timestamp_us)
  {
    ResetLock("raw-imu-out-of-order", "assembled imu timestamp");
    return true;
  }

  imu_history_.PushBackDropOldest(imu);
  ObserveImuCadence(imu.sensor_timestamp_us);
  return true;
}

template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ObserveImuCadence(uint64_t sensor_timestamp_us)
{
  const auto update = CameraFrameSyncCore::ObserveCadence(
      imu_cadence_, sensor_timestamp_us, cadence_stable_gaps,
      imu_cadence_tolerance_us);
  if (update == CameraFrameSyncCore::CadenceUpdate::BROKEN)
  {
    ResetLock("imu-cadence-broken", "gyro/accl/quat");
    return;
  }
  if (imu_cadence_.stable)
  {
    relation_.imu_period_us = imu_cadence_.period_us;
  }
}
