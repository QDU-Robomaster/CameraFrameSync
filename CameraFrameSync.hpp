#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机共享图像桥与原始 imu/image_event 同步器
constructor_args:
  camera: @camera
template_args:
  - Info:
      width: 1280
      height: 720
      step: 3840
      encoding: CameraTypes::Encoding::BGR8
      camera_matrix: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
      distortion_model: CameraTypes::DistortionModel::PLUMB_BOB
      distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
      rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      projection_matrix: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
required_hardware: []
depends:
  - qdu-future/CameraBase
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "camera_frame_sync_core.hpp"
#include "libxr.hpp"
#include "linux_shared_topic.hpp"

template <CameraTypes::CameraInfo CameraInfoV>
class CameraFrameSync
{
 public:
  using Self = CameraFrameSync<CameraInfoV>;
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using ImageFrame = typename Base::ImageFrame;
  using ImuStamped = typename Base::ImuStamped;
  using GyroStamped = typename Base::GyroStamped;
  using AcclStamped = typename Base::AcclStamped;
  using QuatStamped = typename Base::QuatStamped;
  using ImageEvent = typename Base::ImageEvent;
  using SyncConfig = typename Base::SyncConfig;
  using ImageTopic = LibXR::LinuxSharedTopic<ImageFrame>;
  using ImageData = typename ImageTopic::Data;

  static inline constexpr CameraInfo camera_info = CameraInfoV;
  static constexpr LibXR::LinuxSharedTopicConfig image_topic_config{
      .slot_num = 8,
      .subscriber_num = 8,
      .queue_num = 2,
  };

  struct SyncedFrame
  {
    ImageData image{};
    ImuStamped imu{};

    const ImageFrame* GetImageFrame() const { return image.GetData(); }
  };

  class Subscriber
  {
   public:
    explicit Subscriber(const Self& sync)
        : Subscriber(sync.ImageTopicName(), sync.ImuTopicName())
    {
    }

    Subscriber(const char* image_topic_name, const char* imu_topic_name)
        : image_sub_(image_topic_name),
          imu_queue_(kQueueLength),
          imu_sub_(LibXR::Topic(LibXR::Topic::WaitTopic(imu_topic_name)), imu_queue_)
    {
    }

    bool Valid() const { return image_sub_.Valid(); }

    LibXR::ErrorCode Wait(SyncedFrame& out, uint32_t timeout_ms)
    {
      const uint64_t deadline_ms = MakeDeadline(timeout_ms);

      while (true)
      {
        const uint32_t wait_ms = RemainingMs(deadline_ms, timeout_ms);
        if (timeout_ms != UINT32_MAX && wait_ms == 0)
        {
          return LibXR::ErrorCode::TIMEOUT;
        }

        ImageData image_data;
        const auto wait_ans = image_sub_.Wait(image_data, wait_ms);
        if (wait_ans != LibXR::ErrorCode::OK)
        {
          return wait_ans;
        }

        const ImageFrame* image = image_data.GetData();
        if (image == nullptr)
        {
          image_data.Reset();
          continue;
        }

        const uint64_t image_timestamp_us = image->timestamp_us;
        while (true)
        {
          DrainImuQueue();
          while (!pending_imus_.Empty() &&
                 pending_imus_.Front().timestamp_us < image_timestamp_us)
          {
            pending_imus_.PopFront();
          }

          if (!pending_imus_.Empty() &&
              pending_imus_.Front().timestamp_us == image_timestamp_us)
          {
            out.image = std::move(image_data);
            out.imu = pending_imus_.Front();
            pending_imus_.PopFront();
            return LibXR::ErrorCode::OK;
          }

          if (!pending_imus_.Empty() &&
              pending_imus_.Front().timestamp_us > image_timestamp_us)
          {
            image_data.Reset();
            break;
          }

          if (timeout_ms != UINT32_MAX && RemainingMs(deadline_ms, timeout_ms) == 0)
          {
            return LibXR::ErrorCode::TIMEOUT;
          }

          LibXR::Thread::Sleep(1);
        }
      }
    }

   private:
    static constexpr size_t kQueueLength = 64;

    static uint64_t MakeDeadline(uint32_t timeout_ms)
    {
      if (timeout_ms == UINT32_MAX)
      {
        return std::numeric_limits<uint64_t>::max();
      }
      return static_cast<uint64_t>(LibXR::Thread::GetTime()) + timeout_ms;
    }

    static uint32_t RemainingMs(uint64_t deadline_ms, uint32_t timeout_ms)
    {
      if (timeout_ms == UINT32_MAX)
      {
        return UINT32_MAX;
      }

      const uint64_t now_ms = static_cast<uint64_t>(LibXR::Thread::GetTime());
      if (now_ms >= deadline_ms)
      {
        return 0;
      }
      return static_cast<uint32_t>(deadline_ms - now_ms);
    }

    void DrainImuQueue()
    {
      ImuStamped imu{};
      while (imu_queue_.Pop(imu) == LibXR::ErrorCode::OK)
      {
        pending_imus_.PushBackDropOldest(imu);
      }
    }

   private:
    typename ImageTopic::Subscriber image_sub_;
    LibXR::LockFreeQueue<ImuStamped> imu_queue_;
    LibXR::Topic::QueuedSubscriber imu_sub_;
    CameraFrameSyncCore::FixedRingBuffer<ImuStamped, kQueueLength> pending_imus_{};
  };

  CameraFrameSync(LibXR::HardwareContainer&, LibXR::ApplicationManager&, Base& camera)
      : image_topic_name_(camera.ImageTopicName()),
        imu_topic_name_(camera.ImuTopicName()),
        topic_prefix_([&camera]()
                      {
                        const char* name = camera.Name();
                        return (name != nullptr && name[0] != '\0') ? std::string(name)
                                                                     : std::string("camera");
                      }()),
        gyro_topic_name_(topic_prefix_ + "_gyro"),
        accl_topic_name_(topic_prefix_ + "_accl"),
        quat_topic_name_(topic_prefix_ + "_quat"),
        image_event_topic_name_(topic_prefix_ + "_image_event"),
        sync_config_topic_name_(topic_prefix_ + "_sync_config"),
        image_topic_(image_topic_name_, image_topic_config),
        synced_imu_topic_(LibXR::Topic::FindOrCreate<ImuStamped>(imu_topic_name_)),
        sync_config_topic_(LibXR::Topic::FindOrCreate<SyncConfig>(sync_config_topic_name_.c_str())),
        gyro_topic_(LibXR::Topic::FindOrCreate<GyroStamped>(gyro_topic_name_.c_str())),
        accl_topic_(LibXR::Topic::FindOrCreate<AcclStamped>(accl_topic_name_.c_str())),
        quat_topic_(LibXR::Topic::FindOrCreate<QuatStamped>(quat_topic_name_.c_str())),
        image_event_topic_(LibXR::Topic::FindOrCreate<ImageEvent>(image_event_topic_name_.c_str())),
        gyro_cb_(LibXR::Topic::Callback::Create(OnGyroStatic, this)),
        accl_cb_(LibXR::Topic::Callback::Create(OnAcclStatic, this)),
        quat_cb_(LibXR::Topic::Callback::Create(OnQuatStatic, this)),
        image_event_cb_(LibXR::Topic::Callback::Create(OnImageEventStatic, this))
  {
    if (!image_topic_.Valid())
    {
      throw std::runtime_error("CameraFrameSync: image topic creation failed");
    }
    if (!AcquireInitialWritableImage())
    {
      throw std::runtime_error("CameraFrameSync: initial image slot acquisition failed");
    }
    if (!camera.RegisterImageSink(this, current_image_.GetData(), CommitImageAdapter))
    {
      current_image_.Reset();
      throw std::runtime_error("CameraFrameSync: image sink registration failed");
    }

    gyro_topic_.RegisterCallback(gyro_cb_);
    accl_topic_.RegisterCallback(accl_cb_);
    quat_topic_.RegisterCallback(quat_cb_);
    image_event_topic_.RegisterCallback(image_event_cb_);
  }

  ~CameraFrameSync() = default;

  const char* ImageTopicName() const { return image_topic_name_; }

  const char* ImuTopicName() const { return imu_topic_name_; }

  const char* GyroTopicName() const { return gyro_topic_name_.c_str(); }

  const char* AcclTopicName() const { return accl_topic_name_.c_str(); }

  const char* QuatTopicName() const { return quat_topic_name_.c_str(); }

  const char* ImageEventTopicName() const { return image_event_topic_name_.c_str(); }

  const char* SyncConfigTopicName() const { return sync_config_topic_name_.c_str(); }

  void PublishSyncConfig(const SyncConfig& config)
  {
    LibXR::Mutex::LockGuard lock(sync_config_mutex_);
    sync_config_ = SanitizeSyncConfig(config);
    SyncConfig publish_value = sync_config_;
    sync_config_topic_.Publish(publish_value);
  }

 private:
  using SyncLockState = CameraFrameSyncCore::SyncLockState;
  using SyncCadenceState = CameraFrameSyncCore::SyncCadenceState;
  template <typename T, size_t Capacity>
  using FixedRingBuffer = CameraFrameSyncCore::FixedRingBuffer<T, Capacity>;

  struct AssembledImu
  {
    uint64_t timestamp_us{};
    std::array<float, 4> rotation_wxyz{};
    std::array<float, 3> angular_velocity_xyz{};
    std::array<float, 3> linear_acceleration_xyz{};
  };

  static constexpr size_t kImuIngressLength = 1024;
  static constexpr size_t kImageEventIngressLength = 256;
  static constexpr size_t kPendingLimit = 2048;
  static constexpr size_t kHistoryLimit = 2048;

  static SyncConfig SanitizeSyncConfig(SyncConfig config)
  {
    if (config.image_divisor_steps == 0)
    {
      config.image_divisor_steps = 1;
    }
    if (config.image_phase_steps >= config.image_divisor_steps)
    {
      config.image_phase_steps =
          static_cast<uint16_t>(config.image_phase_steps % config.image_divisor_steps);
    }
    if (config.search_window_frames == 0)
    {
      config.search_window_frames = 6;
    }
    if (config.image_timeout_ms == 0)
    {
      config.image_timeout_ms = 200;
    }
    if (config.imu_timeout_ms == 0)
    {
      config.imu_timeout_ms = 100;
    }
    if (config.relock_confirm_frames == 0)
    {
      config.relock_confirm_frames = 3;
    }
    return config;
  }

  SyncConfig SnapshotSyncConfig()
  {
    LibXR::Mutex::LockGuard lock(sync_config_mutex_);
    return sync_config_;
  }

  bool AcquireInitialWritableImage()
  {
    if (image_topic_.CreateData(current_image_) != LibXR::ErrorCode::OK)
    {
      return false;
    }
    return current_image_.GetData() != nullptr;
  }

  static ImageFrame* CommitImageAdapter(void* ctx)
  {
    return static_cast<Self*>(ctx)->CommitImageAndLeaseNext();
  }

  ImageFrame* CommitImageAndLeaseNext()
  {
    ImageFrame* current_image = current_image_.GetData();
    if (current_image == nullptr)
    {
      return nullptr;
    }

    ImageData next_image;
    if (image_topic_.CreateData(next_image) != LibXR::ErrorCode::OK)
    {
      return current_image;
    }

    (void)image_topic_.Publish(current_image_);
    current_image_ = std::move(next_image);
    return current_image_.GetData();
  }

  static void OnGyroStatic(bool, Self* self, LibXR::RawData& data)
  {
    self->last_gyro_rx_ms_.store(LibXR::Thread::GetTime(), std::memory_order_relaxed);
    const auto& gyro = *reinterpret_cast<const GyroStamped*>(data.addr_);
    if (self->gyro_ingress_.Push(gyro) != LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  static void OnAcclStatic(bool, Self* self, LibXR::RawData& data)
  {
    self->last_accl_rx_ms_.store(LibXR::Thread::GetTime(), std::memory_order_relaxed);
    const auto& accl = *reinterpret_cast<const AcclStamped*>(data.addr_);
    if (self->accl_ingress_.Push(accl) != LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  static void OnQuatStatic(bool, Self* self, LibXR::RawData& data)
  {
    self->last_quat_rx_ms_.store(LibXR::Thread::GetTime(), std::memory_order_relaxed);
    const auto& quat = *reinterpret_cast<const QuatStamped*>(data.addr_);
    if (self->quat_ingress_.Push(quat) != LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  static void OnImageEventStatic(bool, Self* self, LibXR::RawData& data)
  {
    const uint32_t now_ms = LibXR::Thread::GetTime();
    const uint32_t previous_image_event_rx_ms =
        self->last_image_event_rx_ms_.exchange(now_ms, std::memory_order_relaxed);
    const auto& image_event = *reinterpret_cast<const ImageEvent*>(data.addr_);
    if (self->image_event_ingress_.Push(image_event) != LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
    // 图像事件就是同步触发点：原始 imu 回调只入队，对齐逻辑只在这里串行执行。
    self->ProcessPendingSyncWork(previous_image_event_rx_ms, now_ms);
  }

  void ProcessPendingSyncWork(uint32_t previous_image_event_rx_ms,
                              uint32_t current_image_event_rx_ms)
  {
    // 所有同步状态都只在图像事件触发路径里串行修改，不再引入额外线程。
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    const SyncConfig config = SnapshotSyncConfig();
    DrainIngressQueues();
    HandleRecoveryTriggers(config, previous_image_event_rx_ms, current_image_event_rx_ms);
    DrainPendingImageEvents(config);
  }

  void DrainIngressQueues()
  {
    GyroStamped gyro{};
    while (gyro_ingress_.Pop(gyro) == LibXR::ErrorCode::OK)
    {
      pending_gyros_.PushBackDropOldest(gyro);
    }

    AcclStamped accl{};
    while (accl_ingress_.Pop(accl) == LibXR::ErrorCode::OK)
    {
      pending_accls_.PushBackDropOldest(accl);
    }

    QuatStamped quat{};
    while (quat_ingress_.Pop(quat) == LibXR::ErrorCode::OK)
    {
      pending_quats_.PushBackDropOldest(quat);
    }

    ImageEvent image_event{};
    while (image_event_ingress_.Pop(image_event) == LibXR::ErrorCode::OK)
    {
      if (pending_image_events_.PushBackDropOldest(image_event))
      {
        overflowed_.store(true, std::memory_order_relaxed);
      }
    }

    AssembleImuHistory();
  }

  void AssembleImuHistory()
  {
    while (CameraFrameSyncCore::TryBuildFrontImu(
        pending_gyros_, pending_accls_, pending_quats_, imu_history_,
        cadence_state_.last_imu_period_us,
        kHistoryLimit,
        [](const GyroStamped& gyro, const AcclStamped& accl, const QuatStamped& quat)
        {
          return AssembledImu{
              .timestamp_us = gyro.sensor_timestamp_us,
              .rotation_wxyz = quat.rotation_wxyz,
              .angular_velocity_xyz = gyro.angular_velocity_xyz,
              .linear_acceleration_xyz = accl.linear_acceleration_xyz,
          };
        }))
    {
    }
  }

  void DrainPendingImageEvents(const SyncConfig& config)
  {
    while (!pending_image_events_.Empty())
    {
      const ImageEvent image_event = pending_image_events_.Front();
      const bool has_imu_history = !imu_history_.Empty();
      const uint64_t newest_imu_timestamp_us =
          has_imu_history ? imu_history_.Back().timestamp_us : 0ULL;
      if (CameraFrameSyncCore::NeedMoreImuForImage(
              image_event, has_imu_history, newest_imu_timestamp_us, config.offset_us))
      {
        break;
      }

      pending_image_events_.PopFront();

      const AssembledImu* imu = CameraFrameSyncCore::FindMatchedImu(
          image_event, imu_history_, config.offset_us, cadence_state_.last_imu_period_us);
      if (imu == nullptr)
      {
        CameraFrameSyncCore::EnterRecovering(lock_state_, cadence_state_);
        continue;
      }
      if (!CameraFrameSyncCore::IsCadenceStable(image_event, *imu, cadence_state_))
      {
        CameraFrameSyncCore::EnterRecovering(lock_state_, cadence_state_);
        continue;
      }

      PublishSyncedImu(image_event.sensor_timestamp_us, *imu);
      CameraFrameSyncCore::AcceptMatch(
          image_event, *imu, cadence_state_, lock_state_, config.relock_confirm_frames);
    }
  }

  void PublishSyncedImu(uint64_t image_timestamp_us, const AssembledImu& imu)
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

  void HandleRecoveryTriggers(const SyncConfig& config,
                              uint32_t previous_image_event_rx_ms,
                              uint32_t current_image_event_rx_ms)
  {
    const uint32_t last_imu_rx_ms =
        std::max(last_gyro_rx_ms_.load(std::memory_order_relaxed),
                 std::max(last_accl_rx_ms_.load(std::memory_order_relaxed),
                          last_quat_rx_ms_.load(std::memory_order_relaxed)));

    const bool image_timeout =
        previous_image_event_rx_ms != 0 &&
        static_cast<uint32_t>(current_image_event_rx_ms - previous_image_event_rx_ms) >
            config.image_timeout_ms;
    const bool imu_timeout =
        last_imu_rx_ms != 0 &&
        static_cast<uint32_t>(current_image_event_rx_ms - last_imu_rx_ms) >
            config.imu_timeout_ms;

    if (overflowed_.exchange(false, std::memory_order_relaxed) || image_timeout ||
        imu_timeout)
    {
      CameraFrameSyncCore::EnterRecovering(lock_state_, cadence_state_);
    }
  }

 private:
  const char* image_topic_name_;
  const char* imu_topic_name_;
  std::string topic_prefix_{};
  std::string gyro_topic_name_{};
  std::string accl_topic_name_{};
  std::string quat_topic_name_{};
  std::string image_event_topic_name_{};
  std::string sync_config_topic_name_{};

  ImageTopic image_topic_;
  ImageData current_image_{};
  LibXR::Topic synced_imu_topic_;
  LibXR::Topic sync_config_topic_;
  LibXR::Topic gyro_topic_;
  LibXR::Topic accl_topic_;
  LibXR::Topic quat_topic_;
  LibXR::Topic image_event_topic_;
  LibXR::Topic::Callback gyro_cb_{};
  LibXR::Topic::Callback accl_cb_{};
  LibXR::Topic::Callback quat_cb_{};
  LibXR::Topic::Callback image_event_cb_{};

  // 话题回调只负责把原始样本塞进各自的 ingress。
  LibXR::LockFreeQueue<GyroStamped> gyro_ingress_{kImuIngressLength};
  LibXR::LockFreeQueue<AcclStamped> accl_ingress_{kImuIngressLength};
  LibXR::LockFreeQueue<QuatStamped> quat_ingress_{kImuIngressLength};
  LibXR::LockFreeQueue<ImageEvent> image_event_ingress_{kImageEventIngressLength};
  std::atomic<bool> overflowed_{false};

  // 下面这些状态只在 image_event 触发的串行路径里访问。
  FixedRingBuffer<GyroStamped, kPendingLimit> pending_gyros_{};
  FixedRingBuffer<AcclStamped, kPendingLimit> pending_accls_{};
  FixedRingBuffer<QuatStamped, kPendingLimit> pending_quats_{};
  FixedRingBuffer<ImageEvent, kPendingLimit> pending_image_events_{};
  FixedRingBuffer<AssembledImu, kHistoryLimit> imu_history_{};
  LibXR::Mutex sync_state_mutex_{};

  LibXR::Mutex sync_config_mutex_{};
  SyncConfig sync_config_ = SanitizeSyncConfig(SyncConfig{});
  SyncLockState lock_state_{};
  SyncCadenceState cadence_state_{};

  std::atomic<uint32_t> last_image_event_rx_ms_{0};
  std::atomic<uint32_t> last_gyro_rx_ms_{0};
  std::atomic<uint32_t> last_accl_rx_ms_{0};
  std::atomic<uint32_t> last_quat_rx_ms_{0};
};
