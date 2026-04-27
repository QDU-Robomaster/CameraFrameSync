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
  using SensorSyncCmd = typename Base::SensorSyncCmd;
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

  struct SyncPolicy
  {
    int32_t offset_us{0};
    uint32_t image_timeout_ms{200};
    uint32_t imu_timeout_ms{100};
    uint32_t relock_confirm_frames{3};
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
        sensor_sync_cmd_topic_name_("sensor_sync_cmd"),
        image_topic_(image_topic_name_, image_topic_config),
        synced_imu_topic_(LibXR::Topic::FindOrCreate<ImuStamped>(imu_topic_name_)),
        sensor_sync_cmd_topic_(
            LibXR::Topic::FindOrCreate<SensorSyncCmd>(sensor_sync_cmd_topic_name_.c_str())),
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

  const char* SensorSyncCmdTopicName() const
  {
    return sensor_sync_cmd_topic_name_.c_str();
  }

  void PublishSensorSyncCmd(const SensorSyncCmd& cmd)
  {
    SensorSyncCmd publish_value = cmd;
    sensor_sync_cmd_topic_.Publish(publish_value);
  }

  void SetSyncPolicy(const SyncPolicy& policy)
  {
    LibXR::Mutex::LockGuard lock(sync_policy_mutex_);
    sync_policy_ = SanitizeSyncPolicy(policy);
  }

 private:
  using SyncLockState = CameraFrameSyncCore::SyncLockState;
  using SyncState = CameraFrameSyncCore::SyncState;
  template <typename T, size_t Capacity>
  using FixedRingBuffer = CameraFrameSyncCore::FixedRingBuffer<T, Capacity>;

  struct TimedGyro
  {
    GyroStamped sample{};
    uint64_t rx_time_us{};
  };

  struct TimedAccl
  {
    AcclStamped sample{};
    uint64_t rx_time_us{};
  };

  struct TimedQuat
  {
    QuatStamped sample{};
    uint64_t rx_time_us{};
  };

  struct TimedImageEvent
  {
    ImageEvent sample{};
    uint64_t rx_time_us{};
  };

  struct AssembledImu
  {
    uint64_t sensor_timestamp_us{};
    uint64_t rx_time_us{};
    std::array<float, 4> rotation_wxyz{};
    std::array<float, 3> angular_velocity_xyz{};
    std::array<float, 3> linear_acceleration_xyz{};
  };

  struct ImageReference
  {
    bool valid{false};
    TimedImageEvent image{};
  };

  struct SyncRelation
  {
    uint64_t base_image_rx_period_us{0};
    uint64_t last_imu_sensor_period_us{0};
    uint64_t last_imu_rx_period_us{0};
    uint64_t sync_imu_sensor_period_us{0};
    uint64_t last_image_rx_time_us{0};
    uint64_t last_sync_imu_sensor_timestamp_us{0};
    int64_t last_host_skew_us{0};
  };

  enum class ImageDecision : uint8_t
  {
    WAIT = 0,
    ACCEPT = 1,
    REJECT = 2,
  };

  struct SyncMatch
  {
    const AssembledImu* sync_imu{nullptr};
    uint64_t next_sync_sensor_period_us{0};
    int64_t host_skew_us{0};

    bool Valid() const
    {
      return sync_imu != nullptr && next_sync_sensor_period_us != 0;
    }
  };

  static constexpr size_t kImuIngressLength = 1024;
  static constexpr size_t kImageEventIngressLength = 256;
  static constexpr size_t kPendingLimit = 2048;
  static constexpr size_t kHistoryLimit = 2048;
  static constexpr uint32_t kProbeTimeoutMs = 500;

  static SyncPolicy SanitizeSyncPolicy(SyncPolicy policy)
  {
    if (policy.image_timeout_ms == 0)
    {
      policy.image_timeout_ms = 200;
    }
    if (policy.imu_timeout_ms == 0)
    {
      policy.imu_timeout_ms = 100;
    }
    if (policy.relock_confirm_frames == 0)
    {
      policy.relock_confirm_frames = 3;
    }
    return policy;
  }

  SyncPolicy SnapshotSyncPolicy()
  {
    LibXR::Mutex::LockGuard lock(sync_policy_mutex_);
    return sync_policy_;
  }

  static uint64_t NowUs()
  {
    return static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds());
  }

  static uint64_t ImageGapToleranceUs(const SyncRelation& relation)
  {
    return CameraFrameSyncCore::ImageGapToleranceUs(relation.base_image_rx_period_us,
                                                    relation.last_imu_rx_period_us);
  }

  static uint64_t HostSkewToleranceUs(const SyncRelation& relation)
  {
    return CameraFrameSyncCore::HostSkewToleranceUs(relation.last_imu_rx_period_us);
  }

  static uint64_t ProbeSearchWindowUs(const SyncRelation& relation)
  {
    return std::max<uint64_t>(100000ULL, relation.base_image_rx_period_us * 2ULL);
  }

  static uint64_t ProbeTimeoutUs(uint32_t image_timeout_ms)
  {
    return static_cast<uint64_t>(std::max<uint32_t>(kProbeTimeoutMs, image_timeout_ms * 2U)) *
           1000ULL;
  }

  void ResetLockedRelation()
  {
    relation_.sync_imu_sensor_period_us = 0;
    relation_.last_image_rx_time_us = 0;
    relation_.last_sync_imu_sensor_timestamp_us = 0;
    relation_.last_host_skew_us = 0;
  }

  void EnterRecovering()
  {
    CameraFrameSyncCore::EnterRecovering(lock_state_);
    ResetLockedRelation();
  }

  uint32_t EstimatedStrideSamples() const
  {
    return CameraFrameSyncCore::EstimateStrideSamples(relation_.base_image_rx_period_us,
                                                      relation_.last_imu_rx_period_us);
  }

  uint64_t EstimatedSyncImuSensorPeriodUs() const
  {
    if (relation_.sync_imu_sensor_period_us != 0)
    {
      return relation_.sync_imu_sensor_period_us;
    }

    const uint32_t stride_samples = EstimatedStrideSamples();
    if (stride_samples == 0 || relation_.last_imu_sensor_period_us == 0)
    {
      return 0;
    }

    return relation_.last_imu_sensor_period_us * static_cast<uint64_t>(stride_samples);
  }

  bool IsNormalImageGap(uint64_t image_gap_rx_us) const
  {
    if (relation_.base_image_rx_period_us == 0 || image_gap_rx_us == 0)
    {
      return false;
    }

    return CameraFrameSyncCore::AbsDiffUs(image_gap_rx_us, relation_.base_image_rx_period_us) <=
           ImageGapToleranceUs(relation_);
  }

  bool IsProbeImageGap(uint64_t image_gap_rx_us) const
  {
    if (relation_.base_image_rx_period_us == 0 || image_gap_rx_us == 0)
    {
      return false;
    }

    return CameraFrameSyncCore::AbsDiffUs(image_gap_rx_us,
                                          relation_.base_image_rx_period_us * 2ULL) <=
           ImageGapToleranceUs(relation_);
  }

  bool IsLockingOrSynced() const
  {
    return lock_state_.state == SyncState::LOCKING || lock_state_.state == SyncState::SYNCED;
  }

  void ClearPendingProbe()
  {
    probe_pending_ = false;
    pending_probe_sent_rx_us_ = 0;
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
    if (self->gyro_ingress_.Push(TimedGyro{.sample = gyro, .rx_time_us = NowUs()}) !=
        LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  static void OnAcclStatic(bool, Self* self, LibXR::RawData& data)
  {
    self->last_accl_rx_ms_.store(LibXR::Thread::GetTime(), std::memory_order_relaxed);
    const auto& accl = *reinterpret_cast<const AcclStamped*>(data.addr_);
    if (self->accl_ingress_.Push(TimedAccl{.sample = accl, .rx_time_us = NowUs()}) !=
        LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  static void OnQuatStatic(bool, Self* self, LibXR::RawData& data)
  {
    self->last_quat_rx_ms_.store(LibXR::Thread::GetTime(), std::memory_order_relaxed);
    const auto& quat = *reinterpret_cast<const QuatStamped*>(data.addr_);
    if (self->quat_ingress_.Push(TimedQuat{.sample = quat, .rx_time_us = NowUs()}) !=
        LibXR::ErrorCode::OK)
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
    if (self->image_event_ingress_.Push(
            TimedImageEvent{.sample = image_event, .rx_time_us = NowUs()}) !=
        LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }

    // 图像事件就是同步触发点：原始回调只入队，所有对齐状态都在这里串行推进。
    self->ProcessPendingSyncWork(previous_image_event_rx_ms, now_ms);
  }

  void ProcessPendingSyncWork(uint32_t previous_image_event_rx_ms,
                              uint32_t current_image_event_rx_ms)
  {
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    const SyncPolicy policy = SnapshotSyncPolicy();
    DrainIngressQueues();
    HandleRecoveryTriggers(policy, previous_image_event_rx_ms, current_image_event_rx_ms);
    MaybeSendProbe(policy);
    DrainPendingImageEvents(policy);
  }

  void DrainIngressQueues()
  {
    TimedGyro gyro{};
    while (gyro_ingress_.Pop(gyro) == LibXR::ErrorCode::OK)
    {
      if (pending_gyros_.PushBackDropOldest(gyro))
      {
        overflowed_.store(true, std::memory_order_relaxed);
      }
    }

    TimedAccl accl{};
    while (accl_ingress_.Pop(accl) == LibXR::ErrorCode::OK)
    {
      if (pending_accls_.PushBackDropOldest(accl))
      {
        overflowed_.store(true, std::memory_order_relaxed);
      }
    }

    TimedQuat quat{};
    while (quat_ingress_.Pop(quat) == LibXR::ErrorCode::OK)
    {
      if (pending_quats_.PushBackDropOldest(quat))
      {
        overflowed_.store(true, std::memory_order_relaxed);
      }
    }

    TimedImageEvent image_event{};
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
        relation_.last_imu_sensor_period_us, relation_.last_imu_rx_period_us, kHistoryLimit,
        [](const GyroStamped& gyro, const AcclStamped& accl, const QuatStamped& quat,
           uint64_t rx_time_us)
        {
          return AssembledImu{
              .sensor_timestamp_us = gyro.sensor_timestamp_us,
              .rx_time_us = rx_time_us,
              .rotation_wxyz = quat.rotation_wxyz,
              .angular_velocity_xyz = gyro.angular_velocity_xyz,
              .linear_acceleration_xyz = accl.linear_acceleration_xyz,
          };
        }))
    {
    }
  }

  void RememberObservedImage(const TimedImageEvent& image)
  {
    last_observed_image_.valid = true;
    last_observed_image_.image = image;
  }

  void ObserveNormalImagePeriod(const TimedImageEvent& image, uint64_t image_gap_rx_us)
  {
    if (image_gap_rx_us == 0)
    {
      return;
    }

    if (relation_.base_image_rx_period_us == 0 || IsNormalImageGap(image_gap_rx_us))
    {
      relation_.base_image_rx_period_us = image_gap_rx_us;
    }

    last_normal_image_.valid = true;
    last_normal_image_.image = image;
  }

  void DrainPendingImageEvents(const SyncPolicy& policy)
  {
    while (!pending_image_events_.Empty())
    {
      const TimedImageEvent image = pending_image_events_.Front();
      if (!last_observed_image_.valid)
      {
        RememberObservedImage(image);
        pending_image_events_.PopFront();
        continue;
      }

      if (image.rx_time_us <= last_observed_image_.image.rx_time_us)
      {
        pending_image_events_.PopFront();
        EnterRecovering();
        MaybeSendProbe(policy);
        continue;
      }

      const uint64_t image_gap_rx_us = image.rx_time_us - last_observed_image_.image.rx_time_us;
      const bool probe_pending = probe_pending_;

      ImageDecision decision = ImageDecision::REJECT;
      if (probe_pending)
      {
        decision = IsProbeImageGap(image_gap_rx_us) ? TryLockFromProbe(image, image_gap_rx_us, policy)
                                                    : ImageDecision::REJECT;
      }
      else if (IsLockingOrSynced())
      {
        decision =
            IsNormalImageGap(image_gap_rx_us) ? TryProcessLockedImage(image, image_gap_rx_us, policy)
                                              : ImageDecision::REJECT;
      }
      else
      {
        ObserveNormalImagePeriod(image, image_gap_rx_us);
        RememberObservedImage(image);
        pending_image_events_.PopFront();
        MaybeSendProbe(policy);
        continue;
      }

      if (decision == ImageDecision::WAIT)
      {
        break;
      }

      pending_image_events_.PopFront();
      if (decision == ImageDecision::ACCEPT)
      {
        if (probe_pending)
        {
          ClearPendingProbe();
        }
        else
        {
          ObserveNormalImagePeriod(image, image_gap_rx_us);
        }
        RememberObservedImage(image);
        continue;
      }

      if (probe_pending)
      {
        ClearPendingProbe();
      }
      RememberObservedImage(image);
      EnterRecovering();
      MaybeSendProbe(policy);
    }
  }

  void MaybeSendProbe(const SyncPolicy& policy)
  {
    if (IsLockingOrSynced())
    {
      return;
    }

    if (relation_.base_image_rx_period_us == 0 || EstimatedSyncImuSensorPeriodUs() == 0 ||
        !last_normal_image_.valid)
    {
      return;
    }

    const uint64_t now_us = NowUs();
    if (probe_pending_)
    {
      if (now_us - pending_probe_sent_rx_us_ <= ProbeTimeoutUs(policy.image_timeout_ms))
      {
        return;
      }
      ClearPendingProbe();
    }

    SensorSyncCmd cmd{};
    sensor_sync_cmd_topic_.Publish(cmd);
    probe_pending_ = true;
    pending_probe_sent_rx_us_ = now_us;
  }

  static bool IsBetterTrackedMatch(uint64_t host_skew_error_us,
                                   uint64_t best_host_skew_error_us,
                                   uint64_t sensor_gap_error_us,
                                   uint64_t best_sensor_gap_error_us)
  {
    return host_skew_error_us < best_host_skew_error_us ||
           (host_skew_error_us == best_host_skew_error_us &&
            sensor_gap_error_us < best_sensor_gap_error_us);
  }

  static bool IsBetterProbeMatch(uint64_t host_gap_error_us,
                                 uint64_t best_host_gap_error_us,
                                 uint64_t sensor_gap_error_us,
                                 uint64_t best_sensor_gap_error_us,
                                 int64_t abs_probe_skew_us,
                                 int64_t best_abs_probe_skew_us)
  {
    return host_gap_error_us < best_host_gap_error_us ||
           (host_gap_error_us == best_host_gap_error_us &&
            sensor_gap_error_us < best_sensor_gap_error_us) ||
           (host_gap_error_us == best_host_gap_error_us &&
            sensor_gap_error_us == best_sensor_gap_error_us &&
            abs_probe_skew_us < best_abs_probe_skew_us);
  }

  ImageDecision PublishMatchedImage(const TimedImageEvent& image,
                                    const SyncMatch& match,
                                    const SyncPolicy& policy)
  {
    // 先确定“同步帧是哪一条 IMU”，再只在 IMU 自己时间域里应用 offset 取最终样本。
    const AssembledImu* final_imu = FindFinalImu(*match.sync_imu, policy.offset_us);
    if (final_imu == nullptr)
    {
      return NeedMoreImuForOffset(*match.sync_imu, policy.offset_us) ? ImageDecision::WAIT
                                                                     : ImageDecision::REJECT;
    }

    PublishSyncedImu(image.sample.sensor_timestamp_us, *final_imu);
    relation_.sync_imu_sensor_period_us = match.next_sync_sensor_period_us;
    relation_.last_image_rx_time_us = image.rx_time_us;
    relation_.last_sync_imu_sensor_timestamp_us = match.sync_imu->sensor_timestamp_us;
    relation_.last_host_skew_us = match.host_skew_us;
    CameraFrameSyncCore::ObserveGoodFrame(lock_state_, policy.relock_confirm_frames);
    return ImageDecision::ACCEPT;
  }

  SyncMatch BuildTrackedSyncMatch(const TimedImageEvent& image,
                                  const AssembledImu& sync_imu,
                                  uint64_t expected_sync_sensor_gap_us,
                                  uint64_t sensor_gap_tolerance_us,
                                  uint64_t host_skew_tolerance_us) const
  {
    if (sync_imu.sensor_timestamp_us <= relation_.last_sync_imu_sensor_timestamp_us)
    {
      return {};
    }

    const uint64_t sync_sensor_gap_us =
        sync_imu.sensor_timestamp_us - relation_.last_sync_imu_sensor_timestamp_us;
    const uint64_t sensor_gap_error_us =
        CameraFrameSyncCore::AbsDiffUs(sync_sensor_gap_us, expected_sync_sensor_gap_us);
    if (sensor_gap_error_us > sensor_gap_tolerance_us)
    {
      return {};
    }

    const int64_t host_skew_us =
        static_cast<int64_t>(image.rx_time_us) - static_cast<int64_t>(sync_imu.rx_time_us);
    const uint64_t host_skew_error_us =
        CameraFrameSyncCore::AbsDiffSigned(host_skew_us, relation_.last_host_skew_us);
    if (host_skew_error_us > host_skew_tolerance_us)
    {
      return {};
    }

    return SyncMatch{
        .sync_imu = &sync_imu,
        .next_sync_sensor_period_us = sync_sensor_gap_us,
        .host_skew_us = host_skew_us,
    };
  }

  SyncMatch SelectTrackedSyncMatch(const TimedImageEvent& image,
                                   uint64_t expected_sync_sensor_gap_us,
                                   uint64_t sensor_gap_tolerance_us,
                                   uint64_t host_skew_tolerance_us,
                                   uint64_t search_window_us) const
  {
    const uint64_t expected_sync_sensor_timestamp_us =
        relation_.last_sync_imu_sensor_timestamp_us + expected_sync_sensor_gap_us;

    // 稳态跟踪优先走“预测下一帧在哪”，命中后可以避免每帧都扫描整段历史。
    const AssembledImu* predicted_sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
        imu_history_, expected_sync_sensor_timestamp_us, sensor_gap_tolerance_us);
    if (predicted_sync_imu != nullptr)
    {
      const SyncMatch predicted_match = BuildTrackedSyncMatch(
          image, *predicted_sync_imu, expected_sync_sensor_gap_us,
          sensor_gap_tolerance_us, host_skew_tolerance_us);
      if (predicted_match.Valid())
      {
        return predicted_match;
      }
    }

    SyncMatch best_match{};
    uint64_t best_host_skew_error_us = std::numeric_limits<uint64_t>::max();
    uint64_t best_sensor_gap_error_us = std::numeric_limits<uint64_t>::max();

    for (size_t i = imu_history_.Size(); i > 0; --i)
    {
      const AssembledImu& sync_imu = imu_history_[i - 1];
      if (CameraFrameSyncCore::AbsDiffUs(sync_imu.rx_time_us, image.rx_time_us) >
          search_window_us)
      {
        if (sync_imu.rx_time_us + search_window_us < image.rx_time_us)
        {
          break;
        }
        continue;
      }

      const SyncMatch match = BuildTrackedSyncMatch(
          image, sync_imu, expected_sync_sensor_gap_us,
          sensor_gap_tolerance_us, host_skew_tolerance_us);
      if (!match.Valid())
      {
        continue;
      }

      const uint64_t host_skew_error_us =
          CameraFrameSyncCore::AbsDiffSigned(match.host_skew_us, relation_.last_host_skew_us);
      const uint64_t sensor_gap_error_us =
          CameraFrameSyncCore::AbsDiffUs(match.next_sync_sensor_period_us,
                                         expected_sync_sensor_gap_us);
      if (IsBetterTrackedMatch(host_skew_error_us, best_host_skew_error_us,
                               sensor_gap_error_us, best_sensor_gap_error_us))
      {
        best_match = match;
        best_host_skew_error_us = host_skew_error_us;
        best_sensor_gap_error_us = sensor_gap_error_us;
      }
    }

    return best_match;
  }

  SyncMatch SelectProbeSyncMatch(const TimedImageEvent& probe,
                                 uint64_t image_gap_rx_us,
                                 uint64_t expected_probe_sensor_gap_us,
                                 uint64_t sensor_gap_tolerance_us,
                                 uint64_t search_window_us,
                                 uint64_t host_gap_tolerance_us) const
  {
    // probe 期望看到的是 2T 图像 gap；这里在 IMU 历史里找一对满足同样双周期关系的同步帧。
    SyncMatch best_match{};
    uint64_t best_host_gap_error_us = std::numeric_limits<uint64_t>::max();
    uint64_t best_sensor_gap_error_us = std::numeric_limits<uint64_t>::max();
    int64_t best_abs_probe_skew_us = std::numeric_limits<int64_t>::max();

    for (size_t i = imu_history_.Size(); i > 0; --i)
    {
      const AssembledImu& sync_imu = imu_history_[i - 1];
      if (CameraFrameSyncCore::AbsDiffUs(sync_imu.rx_time_us, probe.rx_time_us) >
          search_window_us)
      {
        if (sync_imu.rx_time_us + search_window_us < probe.rx_time_us)
        {
          break;
        }
        continue;
      }
      if (sync_imu.sensor_timestamp_us < expected_probe_sensor_gap_us)
      {
        continue;
      }

      const uint64_t prev_target_sensor_timestamp_us =
          sync_imu.sensor_timestamp_us - expected_probe_sensor_gap_us;
      const AssembledImu* prev_sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
          imu_history_, prev_target_sensor_timestamp_us, sensor_gap_tolerance_us);
      if (prev_sync_imu == nullptr || sync_imu.rx_time_us <= prev_sync_imu->rx_time_us)
      {
        continue;
      }

      const uint64_t host_gap_error_us = CameraFrameSyncCore::AbsDiffUs(
          sync_imu.rx_time_us - prev_sync_imu->rx_time_us, image_gap_rx_us);
      if (host_gap_error_us > host_gap_tolerance_us)
      {
        continue;
      }

      const uint64_t sync_sensor_gap_us =
          sync_imu.sensor_timestamp_us - prev_sync_imu->sensor_timestamp_us;
      const uint64_t sensor_gap_error_us =
          CameraFrameSyncCore::AbsDiffUs(sync_sensor_gap_us, expected_probe_sensor_gap_us);
      const int64_t probe_skew_us =
          static_cast<int64_t>(probe.rx_time_us) - static_cast<int64_t>(sync_imu.rx_time_us);
      const int64_t abs_probe_skew_us =
          probe_skew_us >= 0 ? probe_skew_us : -probe_skew_us;

      if (sync_sensor_gap_us < 2 ||
          !IsBetterProbeMatch(host_gap_error_us, best_host_gap_error_us,
                              sensor_gap_error_us, best_sensor_gap_error_us,
                              abs_probe_skew_us, best_abs_probe_skew_us))
      {
        continue;
      }

      best_match = SyncMatch{
          .sync_imu = &sync_imu,
          .next_sync_sensor_period_us = sync_sensor_gap_us / 2ULL,
          .host_skew_us = probe_skew_us,
      };
      best_host_gap_error_us = host_gap_error_us;
      best_sensor_gap_error_us = sensor_gap_error_us;
      best_abs_probe_skew_us = abs_probe_skew_us;
    }

    return best_match;
  }

  ImageDecision TryLockFromProbe(const TimedImageEvent& probe, uint64_t image_gap_rx_us,
                                 const SyncPolicy& policy)
  {
    if (!last_normal_image_.valid || !IsProbeImageGap(image_gap_rx_us))
    {
      return ImageDecision::REJECT;
    }

    const uint64_t sync_image_sensor_period_us = EstimatedSyncImuSensorPeriodUs();
    if (sync_image_sensor_period_us == 0 || imu_history_.Empty())
    {
      return ImageDecision::WAIT;
    }

    const uint64_t expected_probe_sensor_gap_us = sync_image_sensor_period_us * 2ULL;
    const uint64_t sensor_gap_tolerance_us =
        CameraFrameSyncCore::SyncSensorGapToleranceUs(expected_probe_sensor_gap_us,
                                                      relation_.last_imu_sensor_period_us);
    const uint64_t search_window_us = ProbeSearchWindowUs(relation_);
    const uint64_t host_gap_tolerance_us = HostSkewToleranceUs(relation_);
    if (imu_history_.Back().rx_time_us + host_gap_tolerance_us < probe.rx_time_us)
    {
      return ImageDecision::WAIT;
    }

    const SyncMatch match = SelectProbeSyncMatch(
        probe, image_gap_rx_us, expected_probe_sensor_gap_us,
        sensor_gap_tolerance_us, search_window_us, host_gap_tolerance_us);
    if (!match.Valid())
    {
      return ImageDecision::REJECT;
    }

    return PublishMatchedImage(probe, match, policy);
  }

  ImageDecision TryProcessLockedImage(const TimedImageEvent& image, uint64_t image_gap_rx_us,
                                      const SyncPolicy& policy)
  {
    if (relation_.last_image_rx_time_us == 0 || relation_.last_sync_imu_sensor_timestamp_us == 0)
    {
      return ImageDecision::REJECT;
    }

    if (image.rx_time_us <= relation_.last_image_rx_time_us || !IsNormalImageGap(image_gap_rx_us))
    {
      return ImageDecision::REJECT;
    }

    const uint64_t expected_sync_sensor_gap_us = EstimatedSyncImuSensorPeriodUs();
    if (expected_sync_sensor_gap_us == 0)
    {
      return ImageDecision::WAIT;
    }

    const uint64_t sensor_gap_tolerance_us =
        CameraFrameSyncCore::SyncSensorGapToleranceUs(expected_sync_sensor_gap_us,
                                                      relation_.last_imu_sensor_period_us);
    const uint64_t host_skew_tolerance_us = HostSkewToleranceUs(relation_);
    const uint64_t search_window_us = ProbeSearchWindowUs(relation_);

    int64_t expected_sync_rx_time_us =
        static_cast<int64_t>(image.rx_time_us) - relation_.last_host_skew_us;
    if (expected_sync_rx_time_us > 0 && !imu_history_.Empty() &&
        imu_history_.Back().rx_time_us + host_skew_tolerance_us <
            static_cast<uint64_t>(expected_sync_rx_time_us))
    {
      return ImageDecision::WAIT;
    }

    // 稳态下不再重做全局锁定，而是沿着上一帧的 host skew 和 sensor gap 继续跟踪。
    const SyncMatch match = SelectTrackedSyncMatch(
        image, expected_sync_sensor_gap_us, sensor_gap_tolerance_us,
        host_skew_tolerance_us, search_window_us);
    if (!match.Valid())
    {
      return ImageDecision::REJECT;
    }

    return PublishMatchedImage(image, match, policy);
  }

  const AssembledImu* FindFinalImu(const AssembledImu& sync_imu, int32_t offset_us) const
  {
    const uint64_t target_timestamp_us =
        CameraFrameSyncCore::ApplyOffsetUs(sync_imu.sensor_timestamp_us, offset_us);
    return CameraFrameSyncCore::FindBySensorTimestamp(
        imu_history_, target_timestamp_us,
        CameraFrameSyncCore::OffsetSearchToleranceUs(relation_.last_imu_sensor_period_us,
                                                     EstimatedStrideSamples()));
  }

  bool NeedMoreImuForOffset(const AssembledImu& sync_imu, int32_t offset_us) const
  {
    if (imu_history_.Empty())
    {
      return true;
    }

    const uint64_t target_timestamp_us =
        CameraFrameSyncCore::ApplyOffsetUs(sync_imu.sensor_timestamp_us, offset_us);
    return target_timestamp_us > imu_history_.Back().sensor_timestamp_us;
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

  void HandleRecoveryTriggers(const SyncPolicy& policy,
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
            policy.image_timeout_ms;
    const bool imu_timeout =
        last_imu_rx_ms != 0 &&
        static_cast<uint32_t>(current_image_event_rx_ms - last_imu_rx_ms) >
            policy.imu_timeout_ms;
    const bool probe_timeout =
        probe_pending_ && pending_probe_sent_rx_us_ != 0 &&
        (NowUs() - pending_probe_sent_rx_us_) > ProbeTimeoutUs(policy.image_timeout_ms);

    if (overflowed_.exchange(false, std::memory_order_relaxed) || image_timeout || imu_timeout ||
        probe_timeout)
    {
      ClearPendingProbe();
      EnterRecovering();
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
  std::string sensor_sync_cmd_topic_name_{};

  ImageTopic image_topic_;
  ImageData current_image_{};
  LibXR::Topic synced_imu_topic_;
  LibXR::Topic sensor_sync_cmd_topic_;
  LibXR::Topic gyro_topic_;
  LibXR::Topic accl_topic_;
  LibXR::Topic quat_topic_;
  LibXR::Topic image_event_topic_;
  LibXR::Topic::Callback gyro_cb_{};
  LibXR::Topic::Callback accl_cb_{};
  LibXR::Topic::Callback quat_cb_{};
  LibXR::Topic::Callback image_event_cb_{};

  // 每个回调各自入自己的 SPMC ingress，避免把多个 producer 压到同一个队列里。
  LibXR::LockFreeQueue<TimedGyro> gyro_ingress_{kImuIngressLength};
  LibXR::LockFreeQueue<TimedAccl> accl_ingress_{kImuIngressLength};
  LibXR::LockFreeQueue<TimedQuat> quat_ingress_{kImuIngressLength};
  LibXR::LockFreeQueue<TimedImageEvent> image_event_ingress_{kImageEventIngressLength};
  std::atomic<bool> overflowed_{false};

  // 下面这些状态只在 image_event 触发的串行路径里访问。
  FixedRingBuffer<TimedGyro, kPendingLimit> pending_gyros_{};
  FixedRingBuffer<TimedAccl, kPendingLimit> pending_accls_{};
  FixedRingBuffer<TimedQuat, kPendingLimit> pending_quats_{};
  FixedRingBuffer<TimedImageEvent, kPendingLimit> pending_image_events_{};
  FixedRingBuffer<AssembledImu, kHistoryLimit> imu_history_{};
  LibXR::Mutex sync_state_mutex_{};

  LibXR::Mutex sync_policy_mutex_{};
  SyncPolicy sync_policy_ = SanitizeSyncPolicy(SyncPolicy{});
  SyncLockState lock_state_{};
  SyncRelation relation_{};
  ImageReference last_observed_image_{};
  ImageReference last_normal_image_{};
  bool probe_pending_{false};
  uint64_t pending_probe_sent_rx_us_{0};

  std::atomic<uint32_t> last_image_event_rx_ms_{0};
  std::atomic<uint32_t> last_gyro_rx_ms_{0};
  std::atomic<uint32_t> last_accl_rx_ms_{0};
  std::atomic<uint32_t> last_quat_rx_ms_{0};
};
