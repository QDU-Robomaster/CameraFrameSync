#pragma once

/**
 * @brief 使用默认 RAW_PROBE 运行参数构造同步模块。
 */
template <CameraTypes::CameraInfo CameraInfoV>
CameraFrameSync<CameraInfoV>::CameraFrameSync(
    LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
    typename CameraFrameSync<CameraInfoV>::Base& camera)
    : CameraFrameSync(hw, app, camera, RuntimeParam{})
{
}

/**
 * @brief 完成 topic、图像槽位、CameraBase sink 与原始 IMU 回调绑定。
 */
template <CameraTypes::CameraInfo CameraInfoV>
CameraFrameSync<CameraInfoV>::CameraFrameSync(
    LibXR::HardwareContainer&, LibXR::ApplicationManager& app,
    typename CameraFrameSync<CameraInfoV>::Base& camera,
    typename CameraFrameSync<CameraInfoV>::RuntimeParam runtime)
    : topics_(camera, runtime),
      callbacks_(this)
{
  sync_mode_ = runtime.mode;
  offset_us_ = runtime.offset_us;
  sync_probe_div_ = runtime.sync_probe_div;
  sync_active_level_ = runtime.sync_active_level == 0 ? 0U : 1U;
  target_trigger_hz_ = runtime.target_trigger_hz;
  raw_imu_frame_ = runtime.raw_imu_frame;

  ASSERT(sync_probe_div_ != 0);
  ASSERT(sync_probe_div_ <= UINT8_MAX);
  ASSERT(target_trigger_hz_ > 0.0F);

  // 共享图像 topic 是 Detector/Preview 的图像来源，创建失败直接中止。
  if (!topics_.image.Valid())
  {
    XR_LOG_ERROR("CameraFrameSync: image topic creation failed err=%d",
                 static_cast<int>(topics_.image.GetError()));
    ASSERT(false);
    return;
  }
  // CameraBase 写入当前共享槽位；拿不到初始槽位时不能注册 sink。
  if (!AcquireInitialWritableImage())
  {
    XR_LOG_ERROR("CameraFrameSync: initial image slot acquisition failed");
    ASSERT(false);
    return;
  }
  // 注册成功后，CameraBase 每次提交图像都会从 CommitImageAdapter 换下一块槽位。
  if (!camera.RegisterImageSink(current_image_.GetData(),
                                ImageCommitCallback::Create(CommitImageAdapter, this)))
  {
    current_image_.Reset();
    XR_LOG_ERROR("CameraFrameSync: image sink registration failed");
    ASSERT(false);
    return;
  }

  topics_.gyro.RegisterCallback(callbacks_.gyro);
  topics_.accl.RegisterCallback(callbacks_.accl);
  topics_.quat.RegisterCallback(callbacks_.quat);
  topics_.sync_result.RegisterCallback(callbacks_.sync_result);
  SendResetToDefaultCommand();

  XR_LOG_INFO(
      "CameraFrameSync: enabled raw_prefix=%s domain=%s image=%s imu=%s raw=%s/%s/%s mode=%s raw_imu_frame=%s target_trigger_hz=%.3f",
      topics_.raw_imu_prefix.c_str(), topics_.host_domain_name.c_str(),
      topics_.image_name.c_str(), topics_.imu_name.c_str(),
      topics_.gyro_name.c_str(), topics_.accl_name.c_str(),
      topics_.quat_name.c_str(), SyncModeName(sync_mode_),
      RawImuFrameName(raw_imu_frame_),
      static_cast<double>(target_trigger_hz_));
  app.Register(*this);
}

/**
 * @brief 输出上一监控周期内的相机、IMU 和同步帧统计。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::OnMonitor()
{
  const uint64_t raw_gyro =
      monitor_raw_gyro_count_.exchange(0, std::memory_order_relaxed);
  const uint64_t raw_accl =
      monitor_raw_accl_count_.exchange(0, std::memory_order_relaxed);
  const uint64_t raw_quat =
      monitor_raw_quat_count_.exchange(0, std::memory_order_relaxed);
  const uint64_t image_input =
      monitor_image_input_count_.exchange(0, std::memory_order_relaxed);
  const uint64_t image_publish_ok =
      monitor_image_publish_ok_count_.exchange(0, std::memory_order_relaxed);
  const uint64_t image_drop =
      monitor_image_drop_count_.exchange(0, std::memory_order_relaxed);
  const uint64_t assembled_imu =
      monitor_assembled_imu_count_.exchange(0, std::memory_order_relaxed);
  const uint64_t synced_output =
      monitor_synced_output_count_.exchange(0, std::memory_order_relaxed);
  const uint64_t reset_count =
      monitor_reset_count_.exchange(0, std::memory_order_relaxed);
  const uint64_t overflow_count =
      monitor_overflow_count_.exchange(0, std::memory_order_relaxed);

  SyncMode mode = SyncMode::RAW_PROBE;
  SyncState state = SyncState::OBSERVING;
  uint64_t image_period_us = 0;
  uint64_t imu_period_us = 0;
  uint64_t sync_period_us = 0;
  {
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    mode = sync_mode_;
    state = state_;
    image_period_us = periods_.image_us;
    imu_period_us = periods_.imu_us;
    sync_period_us = locked_sync_.period_us;
  }

  XR_LOG_INFO(
      "CameraFrameSync monitor: mode=%s state=%s raw_imu=%llu/%llu/%llu assembled=%llu image=%llu pub=%llu drop=%llu synced=%llu reset=%llu overflow=%llu image_period_us=%llu imu_period_us=%llu sync_period_us=%llu",
      SyncModeName(mode), StateName(state),
      static_cast<unsigned long long>(raw_gyro),
      static_cast<unsigned long long>(raw_accl),
      static_cast<unsigned long long>(raw_quat),
      static_cast<unsigned long long>(assembled_imu),
      static_cast<unsigned long long>(image_input),
      static_cast<unsigned long long>(image_publish_ok),
      static_cast<unsigned long long>(image_drop),
      static_cast<unsigned long long>(synced_output),
      static_cast<unsigned long long>(reset_count),
      static_cast<unsigned long long>(overflow_count),
      static_cast<unsigned long long>(image_period_us),
      static_cast<unsigned long long>(imu_period_us),
      static_cast<unsigned long long>(sync_period_us));
}

/**
 * @brief 返回共享图像 topic 名称。
 */
template <CameraTypes::CameraInfo CameraInfoV>
const char* CameraFrameSync<CameraInfoV>::ImageTopicName() const
{
  return topics_.image_name.c_str();
}

/**
 * @brief 返回同步后 IMU topic 名称。
 */
template <CameraTypes::CameraInfo CameraInfoV>
const char* CameraFrameSync<CameraInfoV>::ImuTopicName() const
{
  return topics_.imu_name.c_str();
}

/**
 * @brief 返回 Host topic domain 名称。
 */
template <CameraTypes::CameraInfo CameraInfoV>
const char* CameraFrameSync<CameraInfoV>::HostTopicDomainName() const
{
  return topics_.host_domain_name.c_str();
}

/**
 * @brief 读取当前同步模式。
 */
template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::SyncMode
CameraFrameSync<CameraInfoV>::GetSyncMode() const
{
  return sync_mode_;
}

/**
 * @brief 设置最终 IMU 选择 offset。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::SetOffsetUs(int32_t offset_us)
{
  LibXR::Mutex::LockGuard lock(sync_state_mutex_);
  offset_us_ = offset_us;
}

/**
 * @brief 切换同步模式并清空旧模式留下的运行时关系。
 */
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

/**
 * @brief 将同步模式转成日志字符串。
 */
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

/**
 * @brief 将 RAW_PROBE 状态转成日志字符串。
 */
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

/**
 * @brief 将原始 IMU 坐标系转成日志字符串。
 */
template <CameraTypes::CameraInfo CameraInfoV>
const char* CameraFrameSync<CameraInfoV>::RawImuFrameName(
    typename CameraFrameSync<CameraInfoV>::RawImuFrame frame)
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

/**
 * @brief 把 MCU 侧三轴向量规整成 Host 侧平铺数组。
 */
template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImuVector
CameraFrameSync<CameraInfoV>::ToImuVector(
    const typename CameraFrameSync<CameraInfoV>::RawImuVector& data,
    typename CameraFrameSync<CameraInfoV>::RawImuFrame raw_imu_frame)
{
  switch (raw_imu_frame)
  {
    case RawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP:
      return {data.x(), data.y(), data.z()};
    case RawImuFrame::X_FORWARD_Y_LEFT_Z_UP_TO_BODY:
      return {-data.y(), data.x(), data.z()};
  }
  return {data.x(), data.y(), data.z()};
}

/**
 * @brief 把 MCU 侧四元数规整成 Host 侧 wxyz 数组。
 */
template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::QuatSample
CameraFrameSync<CameraInfoV>::ToQuatSample(
    const typename CameraFrameSync<CameraInfoV>::RawQuatSample& data,
    typename CameraFrameSync<CameraInfoV>::RawImuFrame raw_imu_frame)
{
  switch (raw_imu_frame)
  {
    case RawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP:
      return {data.w(), data.x(), data.y(), data.z()};
    case RawImuFrame::X_FORWARD_Y_LEFT_Z_UP_TO_BODY:
      return {data.w(), -data.y(), data.x(), data.z()};
  }
  return {data.w(), data.x(), data.y(), data.z()};
}

/**
 * @brief 从共享图像 topic 获取第一块 CameraBase 可写槽位。
 */
template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::AcquireInitialWritableImage()
{
  if (topics_.image.CreateData(current_image_) != LibXR::ErrorCode::OK)
  {
    return false;
  }
  return current_image_.GetData() != nullptr;
}

/**
 * @brief CameraBase sink 回调适配层。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::CommitImageAdapter(
    bool, CameraFrameSync<CameraInfoV>* self,
    typename CameraFrameSync<CameraInfoV>::ImageFrame*& next_image)
{
  next_image = self->CommitImageAndLeaseNext();
}

/**
 * @brief 发布当前槽位图像，并把下一块槽位交给 CameraBase 继续写。
 */
template <CameraTypes::CameraInfo CameraInfoV>
typename CameraFrameSync<CameraInfoV>::ImageFrame*
CameraFrameSync<CameraInfoV>::CommitImageAndLeaseNext()
{
  ImageFrame* committed_image = current_image_.GetData();
  if (committed_image == nullptr)
  {
    return nullptr;
  }

  const uint64_t image_timestamp_us =
      static_cast<uint64_t>(committed_image->timestamp_us);
  monitor_image_input_count_.fetch_add(1, std::memory_order_relaxed);

  ImageData next_image;
  if (topics_.image.CreateData(next_image) != LibXR::ErrorCode::OK ||
      next_image.GetData() == nullptr)
  {
    // 没有新槽位时当前图像无法发布，但仍是一次真实相机帧到达。
    monitor_image_drop_count_.fetch_add(1, std::memory_order_relaxed);
    ProcessDroppedImage(image_timestamp_us);
    return committed_image;
  }

  const auto publish_ans = topics_.image.Publish(current_image_);
  current_image_ = std::move(next_image);

  if (publish_ans == LibXR::ErrorCode::OK)
  {
    // 只有共享图像发布成功，图像 timestamp 才进入同步状态机。
    monitor_image_publish_ok_count_.fetch_add(1, std::memory_order_relaxed);
    ProcessCommittedImage(image_timestamp_us);
  }
  else
  {
    XR_LOG_WARN("CameraFrameSync: image publish failed err=%d",
                static_cast<int>(publish_ans));
    // 发布失败时不能制造不存在的图像事件，但要把该图像计入同步时间点。
    monitor_image_drop_count_.fetch_add(1, std::memory_order_relaxed);
    ProcessDroppedImage(image_timestamp_us);
  }
  return current_image_.GetData();
}

/**
 * @brief gyro topic 回调：转换数据后写入无锁入口队列。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::OnGyroStatic(
    bool, CameraFrameSync<CameraInfoV>* self, LibXR::MicrosecondTimestamp timestamp,
    const typename CameraFrameSync<CameraInfoV>::RawImuVector& data)
{
  const GyroSample sample{.sensor_timestamp_us = static_cast<uint64_t>(timestamp),
                          .angular_velocity_xyz =
                              ToImuVector(data, self->raw_imu_frame_)};
  self->monitor_raw_gyro_count_.fetch_add(1, std::memory_order_relaxed);
  if (self->gyro_ingress_.Push(sample) != LibXR::ErrorCode::OK)
  {
    // 回调不能做重同步，只标记溢出，后续由图像提交路径统一恢复。
    self->monitor_overflow_count_.fetch_add(1, std::memory_order_relaxed);
    self->overflowed_.store(true, std::memory_order_relaxed);
  }
}

/**
 * @brief accl topic 回调：转换数据后写入无锁入口队列。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::OnAcclStatic(
    bool, CameraFrameSync<CameraInfoV>* self, LibXR::MicrosecondTimestamp timestamp,
    const typename CameraFrameSync<CameraInfoV>::RawImuVector& data)
{
  const AcclSample sample{.sensor_timestamp_us = static_cast<uint64_t>(timestamp),
                          .linear_acceleration_xyz =
                              ToImuVector(data, self->raw_imu_frame_)};
  self->monitor_raw_accl_count_.fetch_add(1, std::memory_order_relaxed);
  if (self->accl_ingress_.Push(sample) != LibXR::ErrorCode::OK)
  {
    // 回调不能做重同步，只标记溢出，后续由图像提交路径统一恢复。
    self->monitor_overflow_count_.fetch_add(1, std::memory_order_relaxed);
    self->overflowed_.store(true, std::memory_order_relaxed);
  }
}

/**
 * @brief quat topic 回调：转换数据后写入无锁入口队列。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::OnQuatStatic(
    bool, CameraFrameSync<CameraInfoV>* self, LibXR::MicrosecondTimestamp timestamp,
    const typename CameraFrameSync<CameraInfoV>::RawQuatSample& data)
{
  const QuatReading sample{.sensor_timestamp_us = static_cast<uint64_t>(timestamp),
                           .rotation_wxyz =
                               ToQuatSample(data, self->raw_imu_frame_)};
  self->monitor_raw_quat_count_.fetch_add(1, std::memory_order_relaxed);
  if (self->quat_ingress_.Push(sample) != LibXR::ErrorCode::OK)
  {
    // 回调不能做重同步，只标记溢出，后续由图像提交路径统一恢复。
    self->monitor_overflow_count_.fetch_add(1, std::memory_order_relaxed);
    self->overflowed_.store(true, std::memory_order_relaxed);
  }
}

/**
 * @brief CameraSync 回执回调：只记录当前 active probe 的匹配 timestamp。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::OnSyncResultStatic(
    bool, CameraFrameSync<CameraInfoV>* self, LibXR::MicrosecondTimestamp timestamp,
    const CameraSync::SyncEvent& event)
{
  if (event.run_trigger_div == 0)
  {
    return;
  }

  const uint32_t active_seq = self->active_probe_seq_.load();
  if (active_seq == 0 || event.seq != active_seq)
  {
    // 旧回执、串扰回执或没有 outstanding probe 时都不能影响状态机。
    return;
  }
  if (self->probe_ack_seq_.load() == event.seq)
  {
    // 同一个 probe 只接受第一次命中，避免重复回执改写同步锚点。
    return;
  }

  self->probe_ack_timestamp_us_.store(static_cast<uint64_t>(timestamp));
  self->probe_ack_run_div_.store(event.run_trigger_div);
  self->probe_ack_seq_.store(event.seq);
}

/**
 * @brief 图像发布成功后的主同步入口。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ProcessCommittedImage(uint64_t image_timestamp_us)
{
  LibXR::Mutex::LockGuard lock(sync_state_mutex_);
  CollectIncomingTopics();
  if (image_events_.PushBackDropOldest({.sensor_timestamp_us = image_timestamp_us}))
  {
    monitor_overflow_count_.fetch_add(1, std::memory_order_relaxed);
    overflowed_.store(true, std::memory_order_relaxed);
  }
  HandleOverflowRecovery();
  ProcessImageEvents();
}

/**
 * @brief 没有新图像事件时，仍推进 IMU 组装和挂起帧等待。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ProcessSyncWorkWithoutImage()
{
  LibXR::Mutex::LockGuard lock(sync_state_mutex_);
  CollectIncomingTopics();
  HandleOverflowRecovery();
  ProcessImageEvents();
}

/**
 * @brief 图像未发布时，维护内部时间点但不发布同步 IMU。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ProcessDroppedImage(uint64_t image_timestamp_us)
{
  LibXR::Mutex::LockGuard lock(sync_state_mutex_);
  CollectIncomingTopics();
  HandleOverflowRecovery();
  ObserveDroppedImage(image_timestamp_us);
  ProcessImageEvents();
}

/**
 * @brief 把 topic 回调入口队列搬运到状态机 pending 队列。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::CollectIncomingTopics()
{
  GyroSample gyro{};
  while (gyro_ingress_.Pop(gyro) == LibXR::ErrorCode::OK)
  {
    if (pending_gyros_.PushBackDropOldest(gyro))
    {
      // pending 队列丢旧样本后，同步关系已经不可信，交给统一恢复处理。
      monitor_overflow_count_.fetch_add(1, std::memory_order_relaxed);
      overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  AcclSample accl{};
  while (accl_ingress_.Pop(accl) == LibXR::ErrorCode::OK)
  {
    if (pending_accls_.PushBackDropOldest(accl))
    {
      // pending 队列丢旧样本后，同步关系已经不可信，交给统一恢复处理。
      monitor_overflow_count_.fetch_add(1, std::memory_order_relaxed);
      overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  QuatReading quat{};
  while (quat_ingress_.Pop(quat) == LibXR::ErrorCode::OK)
  {
    if (pending_quats_.PushBackDropOldest(quat))
    {
      // pending 队列丢旧样本后，同步关系已经不可信，交给统一恢复处理。
      monitor_overflow_count_.fetch_add(1, std::memory_order_relaxed);
      overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  AssembleImuHistory();
}

/**
 * @brief 连续组装所有当前可闭合的 IMU 样本。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::AssembleImuHistory()
{
  while (TryAssembleOneImu())
  {
  }
}

/**
 * @brief 以 gyro timestamp 为主键尝试组装一帧完整 IMU。
 */
template <CameraTypes::CameraInfo CameraInfoV>
bool CameraFrameSync<CameraInfoV>::TryAssembleOneImu()
{
  if (sync_mode_ == SyncMode::LATEST_IMU)
  {
    QuatReading queued_quat{};
    if (!pending_quats_.Front(queued_quat))
    {
      return false;
    }

    const AssembledImu imu{
        .sensor_timestamp_us = queued_quat.sensor_timestamp_us,
        .rotation_wxyz = queued_quat.rotation_wxyz,
        .angular_velocity_xyz = {0.0F, 0.0F, 0.0F},
        .linear_acceleration_xyz = {0.0F, 0.0F, 0.0F},
    };

    pending_quats_.PopFront();

    if (!imu_history_.Empty() &&
        imu.sensor_timestamp_us <= imu_history_.Back().sensor_timestamp_us)
    {
      const uint64_t previous_timestamp_us = imu_history_.Back().sensor_timestamp_us;
      if (imu.sensor_timestamp_us < previous_timestamp_us &&
          previous_timestamp_us - imu.sensor_timestamp_us >=
              raw_imu_epoch_reset_backward_us)
      {
        ResetRawImuEpoch(previous_timestamp_us, imu);
        return true;
      }

      ResetLock("raw-imu-out-of-order", "quat-only assembled imu timestamp");
      return true;
    }

    AcceptAssembledImu(imu);
    return true;
  }

  GyroSample queued_gyro{};
  if (!pending_gyros_.Front(queued_gyro))
  {
    // gyro 是主时间轴；没有 gyro 时不消费其他通道。
    return false;
  }
  if (pending_accls_.Empty() || pending_quats_.Empty())
  {
    // 等待其他通道，不提前丢 gyro。
    return false;
  }

  const uint64_t gyro_ts = queued_gyro.sensor_timestamp_us;
  AcclSample queued_accl{};
  // 当前 DevC 固件把 gyro/accl/quat 都写成同一个 gyro interrupt timestamp。
  // 因此 raw 三通道 join 保持 exact；不匹配代表某个通道缺了这一拍。
  while (pending_accls_.Front(queued_accl) &&
         queued_accl.sensor_timestamp_us < gyro_ts)
  {
    pending_accls_.PopFront();
  }

  QuatReading queued_quat{};
  while (pending_quats_.Front(queued_quat) &&
         queued_quat.sensor_timestamp_us < gyro_ts)
  {
    pending_quats_.PopFront();
  }
  if (!pending_accls_.Front(queued_accl) || !pending_quats_.Front(queued_quat))
  {
    // 其他通道暂时还没追上 gyro，保留当前 gyro 等下一轮。
    return false;
  }

  const uint64_t accl_ts = queued_accl.sensor_timestamp_us;
  const uint64_t quat_ts = queued_quat.sensor_timestamp_us;
  if (accl_ts != gyro_ts || quat_ts != gyro_ts)
  {
    // 当前 gyro 对应的辅通道样本已经错过或尚未出现在相同 timestamp；
    // 丢这一个 raw 点即可。
    // 是否影响相机同步由后续所需同步时间戳是否仍能在样本缓存中找到决定。
    pending_gyros_.PopFront();
    return true;
  }

  const AssembledImu imu{
      .sensor_timestamp_us = queued_gyro.sensor_timestamp_us,
      .rotation_wxyz = queued_quat.rotation_wxyz,
      .angular_velocity_xyz = queued_gyro.angular_velocity_xyz,
      .linear_acceleration_xyz = queued_accl.linear_acceleration_xyz,
  };

  pending_gyros_.PopFront();
  pending_accls_.PopFront();
  pending_quats_.PopFront();

  if (!imu_history_.Empty() &&
      imu.sensor_timestamp_us <= imu_history_.Back().sensor_timestamp_us)
  {
    const uint64_t previous_timestamp_us = imu_history_.Back().sensor_timestamp_us;
    if (imu.sensor_timestamp_us < previous_timestamp_us &&
        previous_timestamp_us - imu.sensor_timestamp_us >=
            raw_imu_epoch_reset_backward_us)
    {
      ResetRawImuEpoch(previous_timestamp_us, imu);
      return true;
    }

    // 小幅乱序或重复仍按坏样本处理；只有大幅回退才视为 USB/设备流重启。
    ResetLock("raw-imu-out-of-order", "assembled imu timestamp");
    return true;
  }

  AcceptAssembledImu(imu);
  return true;
}

/**
 * @brief 接受一帧单调递增的完整 raw IMU。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::AcceptAssembledImu(const AssembledImu& imu)
{
  imu_history_.PushBackDropOldest(imu);
  monitor_assembled_imu_count_.fetch_add(1, std::memory_order_relaxed);
  ObserveImuCadence(imu.sensor_timestamp_us);
}

/**
 * @brief raw IMU 时间戳大幅回退时，按设备/USB 流重启切换到新 epoch。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ResetRawImuEpoch(
    uint64_t previous_timestamp_us, const AssembledImu& first_imu)
{
  XR_LOG_WARN(
      "CameraFrameSync: raw IMU timestamp epoch reset previous=%llu current=%llu",
      static_cast<unsigned long long>(previous_timestamp_us),
      static_cast<unsigned long long>(first_imu.sensor_timestamp_us));
  ResetRuntimeState();
  AcceptAssembledImu(first_imu);
}

/**
 * @brief 观察完整 IMU 帧的发布周期。
 */
template <CameraTypes::CameraInfo CameraInfoV>
void CameraFrameSync<CameraInfoV>::ObserveImuCadence(uint64_t sensor_timestamp_us)
{
  if (sync_mode_ == SyncMode::LATEST_IMU)
  {
    if (!imu_cadence_.has_last_timestamp)
    {
      imu_cadence_.has_last_timestamp = true;
      imu_cadence_.last_timestamp_us = sensor_timestamp_us;
      return;
    }

    if (sensor_timestamp_us > imu_cadence_.last_timestamp_us)
    {
      periods_.imu_us = sensor_timestamp_us - imu_cadence_.last_timestamp_us;
      imu_cadence_.last_timestamp_us = sensor_timestamp_us;
      imu_cadence_.period_us = periods_.imu_us;
      imu_cadence_.stable = true;
    }
    return;
  }

  if (imu_cadence_.stable && imu_cadence_.has_last_timestamp &&
      imu_cadence_.period_us != 0 &&
      sensor_timestamp_us > imu_cadence_.last_timestamp_us)
  {
    const uint64_t gap_us = sensor_timestamp_us - imu_cadence_.last_timestamp_us;
    const uint32_t gap_stride = CameraFrameSyncCore::MatchPeriodGapStride(
        gap_us, imu_cadence_.period_us, imu_cadence_tolerance_us,
        max_raw_imu_gap_stride);
    if (gap_stride > 1)
    {
      imu_cadence_.last_timestamp_us = sensor_timestamp_us;
      periods_.imu_us = imu_cadence_.period_us;
      return;
    }
  }

  const auto update = CameraFrameSyncCore::ObserveCadence(
      imu_cadence_, sensor_timestamp_us, cadence_stable_gaps,
      imu_cadence_tolerance_us);
  if (update == CameraFrameSyncCore::CadenceUpdate::BROKEN)
  {
    // IMU 周期破坏会使 RAW_PROBE 的递推周期失效，必须回到观察状态。
    ResetLock("imu-cadence-broken", "gyro/accl/quat");
    return;
  }
  if (imu_cadence_.stable)
  {
    periods_.imu_us = imu_cadence_.period_us;
  }
}
