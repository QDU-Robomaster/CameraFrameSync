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
    uint32_t sensor_sequence{};
    uint64_t rx_time_us{};
    std::array<float, 4> rotation_wxyz{};
    std::array<float, 3> angular_velocity_xyz{};
    std::array<float, 3> linear_acceleration_xyz{};
  };

  struct NormalImageReference
  {
    bool valid{false};
    TimedImageEvent image{};
  };

  struct SyncRelation
  {
    uint32_t stride_samples{0};
    uint32_t last_image_sequence{std::numeric_limits<uint32_t>::max()};
    uint32_t last_sync_imu_sequence{std::numeric_limits<uint32_t>::max()};
    uint64_t base_image_rx_period_us{0};
    uint64_t last_imu_sensor_period_us{0};
    uint64_t last_imu_rx_period_us{0};
    int64_t last_host_skew_us{0};
  };

  enum class ImageDecision : uint8_t
  {
    WAIT = 0,
    ACCEPT = 1,
    REJECT = 2,
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

  static uint32_t SequenceToleranceSamples(const SyncRelation& relation)
  {
    return std::max<uint32_t>(1U, relation.stride_samples / 4U);
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
    relation_.stride_samples = relation_.stride_samples != 0 ? relation_.stride_samples
                                                             : EstimatedStrideSamples();
    relation_.last_image_sequence = std::numeric_limits<uint32_t>::max();
    relation_.last_sync_imu_sequence = std::numeric_limits<uint32_t>::max();
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
              .sensor_sequence = gyro.sensor_sequence,
              .rx_time_us = rx_time_us,
              .rotation_wxyz = quat.rotation_wxyz,
              .angular_velocity_xyz = gyro.angular_velocity_xyz,
              .linear_acceleration_xyz = accl.linear_acceleration_xyz,
          };
        }))
    {
    }
  }

  void ObserveNormalImagePeriod(const TimedImageEvent& image)
  {
    if (image.sample.sync_cmd_id != 0)
    {
      return;
    }

    if (last_normal_image_.valid &&
        image.sample.image_sequence == last_normal_image_.image.sample.image_sequence + 1 &&
        image.rx_time_us > last_normal_image_.image.rx_time_us)
    {
      relation_.base_image_rx_period_us = image.rx_time_us - last_normal_image_.image.rx_time_us;
      relation_.stride_samples = EstimatedStrideSamples();
    }

    last_normal_image_.valid = true;
    last_normal_image_.image = image;
  }

  void DrainPendingImageEvents(const SyncPolicy& policy)
  {
    while (!pending_image_events_.Empty())
    {
      const TimedImageEvent image = pending_image_events_.Front();
      ObserveNormalImagePeriod(image);

      ImageDecision decision = ImageDecision::REJECT;
      if (lock_state_.state == SyncState::LOCKING || lock_state_.state == SyncState::SYNCED)
      {
        decision = TryProcessLockedImage(image, policy);
      }
      else if (image.sample.sync_cmd_id != 0)
      {
        if (pending_probe_cmd_id_ == image.sample.sync_cmd_id)
        {
          pending_probe_cmd_id_ = 0;
          pending_probe_sent_rx_us_ = 0;
        }
        decision = TryLockFromProbe(image, policy);
      }
      else
      {
        pending_image_events_.PopFront();
        continue;
      }

      if (decision == ImageDecision::WAIT)
      {
        break;
      }

      pending_image_events_.PopFront();
      if (decision == ImageDecision::REJECT)
      {
        EnterRecovering();
        MaybeSendProbe(policy);
      }
    }
  }

  void MaybeSendProbe(const SyncPolicy& policy)
  {
    if (lock_state_.state == SyncState::LOCKING || lock_state_.state == SyncState::SYNCED)
    {
      return;
    }

    if (EstimatedStrideSamples() == 0)
    {
      return;
    }

    const uint64_t now_us = NowUs();
    if (pending_probe_cmd_id_ != 0)
    {
      if (now_us - pending_probe_sent_rx_us_ <= ProbeTimeoutUs(policy.image_timeout_ms))
      {
        return;
      }
      pending_probe_cmd_id_ = 0;
      pending_probe_sent_rx_us_ = 0;
    }

    SensorSyncCmd cmd{.cmd_id = next_probe_cmd_id_++};
    sensor_sync_cmd_topic_.Publish(cmd);
    pending_probe_cmd_id_ = cmd.cmd_id;
    pending_probe_sent_rx_us_ = now_us;
  }

  ImageDecision TryLockFromProbe(const TimedImageEvent& probe, const SyncPolicy& policy)
  {
    if (!last_normal_image_.valid ||
        last_normal_image_.image.sample.image_sequence + 1 != probe.sample.image_sequence)
    {
      return ImageDecision::REJECT;
    }

    const uint32_t stride_samples =
        relation_.stride_samples != 0 ? relation_.stride_samples : EstimatedStrideSamples();
    if (stride_samples == 0 || imu_history_.Empty())
    {
      return ImageDecision::WAIT;
    }

    const uint32_t doubled_gap_samples = stride_samples * 2U;
    const uint64_t newest_sequence = imu_history_.Back().sensor_sequence;
    if (newest_sequence < doubled_gap_samples)
    {
      return ImageDecision::WAIT;
    }

    const uint64_t search_window_us = ProbeSearchWindowUs(relation_);
    const int64_t skew_tolerance_us = static_cast<int64_t>(
        CameraFrameSyncCore::SequenceSearchToleranceUs(relation_.last_imu_rx_period_us));

    const AssembledImu* best_sync_imu = nullptr;
    int64_t best_delta_error_us = std::numeric_limits<int64_t>::max();
    int64_t best_probe_skew_us = std::numeric_limits<int64_t>::max();

    for (size_t i = imu_history_.Size(); i > 0; --i)
    {
      const AssembledImu& sync_imu = imu_history_[i - 1];
      if (sync_imu.sensor_sequence < doubled_gap_samples)
      {
        break;
      }
      if (CameraFrameSyncCore::AbsDiffUs(sync_imu.rx_time_us, probe.rx_time_us) >
          search_window_us)
      {
        continue;
      }

      const AssembledImu* prev_sync_imu = CameraFrameSyncCore::FindBySequence(
          imu_history_, sync_imu.sensor_sequence - doubled_gap_samples,
          SequenceToleranceSamples(relation_));
      if (prev_sync_imu == nullptr)
      {
        continue;
      }

      const int64_t probe_skew_us =
          static_cast<int64_t>(probe.rx_time_us) - static_cast<int64_t>(sync_imu.rx_time_us);
      const int64_t prev_skew_us =
          static_cast<int64_t>(last_normal_image_.image.rx_time_us) -
          static_cast<int64_t>(prev_sync_imu->rx_time_us);
      const int64_t delta_error_us =
          CameraFrameSyncCore::AbsDiffSigned(probe_skew_us, prev_skew_us);
      const int64_t abs_probe_skew_us =
          probe_skew_us >= 0 ? probe_skew_us : -probe_skew_us;

      if (delta_error_us < best_delta_error_us ||
          (delta_error_us == best_delta_error_us && abs_probe_skew_us < best_probe_skew_us))
      {
        best_sync_imu = &sync_imu;
        best_delta_error_us = delta_error_us;
        best_probe_skew_us = abs_probe_skew_us;
      }
    }

    if (best_sync_imu == nullptr || best_delta_error_us > skew_tolerance_us)
    {
      return ImageDecision::REJECT;
    }

    const AssembledImu* final_imu = FindFinalImu(*best_sync_imu, policy.offset_us);
    if (final_imu == nullptr)
    {
      return NeedMoreImuForOffset(*best_sync_imu, policy.offset_us) ? ImageDecision::WAIT
                                                                    : ImageDecision::REJECT;
    }

    PublishSyncedImu(probe.sample.sensor_timestamp_us, *final_imu);
    relation_.stride_samples = stride_samples;
    relation_.last_image_sequence = probe.sample.image_sequence;
    relation_.last_sync_imu_sequence = best_sync_imu->sensor_sequence;
    relation_.last_host_skew_us =
        static_cast<int64_t>(probe.rx_time_us) - static_cast<int64_t>(best_sync_imu->rx_time_us);
    CameraFrameSyncCore::ObserveGoodFrame(lock_state_, policy.relock_confirm_frames);
    return ImageDecision::ACCEPT;
  }

  ImageDecision TryProcessLockedImage(const TimedImageEvent& image, const SyncPolicy& policy)
  {
    if (relation_.last_image_sequence == std::numeric_limits<uint32_t>::max() ||
        relation_.last_sync_imu_sequence == std::numeric_limits<uint32_t>::max() ||
        relation_.stride_samples == 0)
    {
      return ImageDecision::REJECT;
    }

    if (image.sample.image_sequence <= relation_.last_image_sequence)
    {
      return ImageDecision::REJECT;
    }

    const uint32_t image_gap = image.sample.image_sequence - relation_.last_image_sequence;
    uint64_t expected_sync_sequence =
        static_cast<uint64_t>(relation_.last_sync_imu_sequence) +
        static_cast<uint64_t>(image_gap) * static_cast<uint64_t>(relation_.stride_samples);
    if (image.sample.sync_cmd_id != 0)
    {
      expected_sync_sequence += relation_.stride_samples;
    }
    if (expected_sync_sequence > std::numeric_limits<uint32_t>::max())
    {
      return ImageDecision::REJECT;
    }

    const uint32_t expected_sequence = static_cast<uint32_t>(expected_sync_sequence);
    if (imu_history_.Empty() ||
        imu_history_.Back().sensor_sequence + SequenceToleranceSamples(relation_) <
            expected_sequence)
    {
      return ImageDecision::WAIT;
    }

    const AssembledImu* sync_imu = CameraFrameSyncCore::FindBySequence(
        imu_history_, expected_sequence, SequenceToleranceSamples(relation_));
    if (sync_imu == nullptr)
    {
      return ImageDecision::REJECT;
    }

    const int64_t host_skew_us =
        static_cast<int64_t>(image.rx_time_us) - static_cast<int64_t>(sync_imu->rx_time_us);
    const int64_t host_skew_error_us =
        CameraFrameSyncCore::AbsDiffSigned(host_skew_us, relation_.last_host_skew_us);
    if (host_skew_error_us >
        static_cast<int64_t>(
            CameraFrameSyncCore::SequenceSearchToleranceUs(relation_.last_imu_rx_period_us)))
    {
      return ImageDecision::REJECT;
    }

    const AssembledImu* final_imu = FindFinalImu(*sync_imu, policy.offset_us);
    if (final_imu == nullptr)
    {
      return NeedMoreImuForOffset(*sync_imu, policy.offset_us) ? ImageDecision::WAIT
                                                               : ImageDecision::REJECT;
    }

    PublishSyncedImu(image.sample.sensor_timestamp_us, *final_imu);
    relation_.last_image_sequence = image.sample.image_sequence;
    relation_.last_sync_imu_sequence = sync_imu->sensor_sequence;
    relation_.last_host_skew_us = host_skew_us;
    CameraFrameSyncCore::ObserveGoodFrame(lock_state_, policy.relock_confirm_frames);
    return ImageDecision::ACCEPT;
  }

  const AssembledImu* FindFinalImu(const AssembledImu& sync_imu, int32_t offset_us) const
  {
    const uint64_t target_timestamp_us =
        CameraFrameSyncCore::ApplyOffsetUs(sync_imu.sensor_timestamp_us, offset_us);
    return CameraFrameSyncCore::FindBySensorTimestamp(
        imu_history_, target_timestamp_us,
        CameraFrameSyncCore::OffsetSearchToleranceUs(relation_.last_imu_sensor_period_us,
                                                     relation_.stride_samples));
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
        pending_probe_cmd_id_ != 0 && pending_probe_sent_rx_us_ != 0 &&
        (NowUs() - pending_probe_sent_rx_us_) > ProbeTimeoutUs(policy.image_timeout_ms);

    if (overflowed_.exchange(false, std::memory_order_relaxed) || image_timeout || imu_timeout ||
        probe_timeout)
    {
      pending_probe_cmd_id_ = 0;
      pending_probe_sent_rx_us_ = 0;
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
  NormalImageReference last_normal_image_{};
  uint32_t next_probe_cmd_id_{1};
  uint32_t pending_probe_cmd_id_{0};
  uint64_t pending_probe_sent_rx_us_{0};

  std::atomic<uint32_t> last_image_event_rx_ms_{0};
  std::atomic<uint32_t> last_gyro_rx_ms_{0};
  std::atomic<uint32_t> last_accl_rx_ms_{0};
  std::atomic<uint32_t> last_quat_rx_ms_{0};
};
