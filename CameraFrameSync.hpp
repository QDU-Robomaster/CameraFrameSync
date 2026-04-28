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
#include <cstdio>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
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
  using ImageCommitCallback = typename Base::ImageCommitCallback;

  static inline constexpr CameraInfo camera_info = CameraInfoV;
  static constexpr const char* sensor_sync_cmd_topic_name = "sensor_sync_cmd";
  static constexpr LibXR::LinuxSharedTopicConfig image_topic_config{
      .slot_num = 8,
      .subscriber_num = 8,
      .queue_num = 2,
  };

  enum class SyncMode : uint8_t
  {
    RAW_PROBE = 0,  // 通过 sensor_sync_cmd 和节拍变化做显式重同步
    LATEST_IMU = 1,  // 认为图像与 IMU 已大致同步，图像直接取当前最新 IMU
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

    Subscriber(std::string_view image_topic_name, std::string_view imu_topic_name)
        : image_sub_(image_topic_name.data()),
          imu_sub_(LibXR::Topic(LibXR::Topic::WaitTopic(imu_topic_name.data())),
                   latest_imu_)
    {
    }

    bool Valid() const { return image_sub_.Valid(); }

    LibXR::ErrorCode Wait(SyncedFrame& out, uint32_t timeout_ms)
    {
      const uint64_t deadline_ms = MakeDeadline(timeout_ms);

      while (true)
      {
        // deadline 恰好耗尽时仍传 0 给底层 Wait，允许消费已经就绪的数据。
        const uint32_t wait_ms = RemainingMs(deadline_ms, timeout_ms);
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
        const auto imu_wait_ans = WaitForMatchingImu(image_timestamp_us, deadline_ms,
                                                     timeout_ms, out.imu);
        if (imu_wait_ans == LibXR::ErrorCode::OK)
        {
          out.image = std::move(image_data);
          return LibXR::ErrorCode::OK;
        }
        if (imu_wait_ans == LibXR::ErrorCode::EMPTY)
        {
          image_data.Reset();
          continue;
        }
        return imu_wait_ans;
      }
    }

   private:
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

    LibXR::ErrorCode WaitForMatchingImu(uint64_t image_timestamp_us,
                                        uint64_t deadline_ms,
                                        uint32_t timeout_ms,
                                        ImuStamped& matched_imu)
    {
      while (true)
      {
        if (latest_imu_valid_)
        {
          if (latest_imu_.timestamp_us == image_timestamp_us)
          {
            matched_imu = latest_imu_;
            latest_imu_valid_ = false;
            return LibXR::ErrorCode::OK;
          }
          if (latest_imu_.timestamp_us > image_timestamp_us)
          {
            return LibXR::ErrorCode::EMPTY;
          }
        }

        // 同样保留 Wait(0) 的非阻塞消费语义，避免 image/imu 到达顺序边界丢帧。
        const uint32_t wait_ms = RemainingMs(deadline_ms, timeout_ms);
        const auto wait_ans = imu_sub_.Wait(wait_ms);
        if (wait_ans != LibXR::ErrorCode::OK)
        {
          return wait_ans;
        }
        latest_imu_valid_ = true;
      }
    }

   private:
    typename ImageTopic::Subscriber image_sub_;
    ImuStamped latest_imu_{};
    LibXR::Topic::SyncSubscriber<ImuStamped> imu_sub_;
    bool latest_imu_valid_{false};
  };

  CameraFrameSync(LibXR::HardwareContainer&, LibXR::ApplicationManager&, Base& camera)
      : image_topic_name_(camera.ImageTopicNameView()),
        imu_topic_name_(camera.ImuTopicNameView()),
        sensor_name_([&camera]()
                     {
                       const std::string_view name = camera.NameView();
                       if (name.empty())
                       {
                         throw std::runtime_error("CameraFrameSync: camera name is required");
                       }
                       return std::string(name);
                     }()),
        gyro_topic_name_(sensor_name_ + "_gyro"),
        accl_topic_name_(sensor_name_ + "_accl"),
        quat_topic_name_(sensor_name_ + "_quat"),
        image_event_topic_name_(sensor_name_ + "_image_event"),
        image_topic_(image_topic_name_.data(), image_topic_config),
        synced_imu_topic_(LibXR::Topic::FindOrCreate<ImuStamped>(imu_topic_name_.data())),
        sensor_sync_cmd_topic_(
            LibXR::Topic::FindOrCreate<SensorSyncCmd>(sensor_sync_cmd_topic_name)),
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
    image_event_topic_.RegisterCallback(image_event_cb_);
  }

  ~CameraFrameSync() = default;

  const char* ImageTopicName() const { return image_topic_name_.data(); }

  const char* ImuTopicName() const { return imu_topic_name_.data(); }

  void SetOffsetUs(int32_t offset_us)
  {
    offset_us_.store(offset_us, std::memory_order_relaxed);
  }

  SyncMode GetSyncMode() const
  {
    return sync_mode_.load(std::memory_order_relaxed);
  }

  void SetSyncMode(SyncMode mode)
  {
    if (GetSyncMode() == mode)
    {
      return;
    }

    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    if (GetSyncMode() == mode)
    {
      return;
    }

    sync_mode_.store(mode, std::memory_order_relaxed);
    ResetRuntimeState();
  }

 private:
  using SyncLockState = CameraFrameSyncCore::SyncLockState;
  using SyncState = CameraFrameSyncCore::SyncState;
  template <typename T, size_t Capacity>
  using FixedRingBuffer = CameraFrameSyncCore::FixedRingBuffer<T, Capacity>;

  struct QueuedGyro
  {
    GyroStamped sample{};
  };

  struct QueuedAccl
  {
    AcclStamped sample{};
  };

  struct QueuedQuat
  {
    QuatStamped sample{};
  };

  struct QueuedImageEvent
  {
    ImageEvent sample{};
  };

  struct AssembledImu
  {
    uint64_t sensor_timestamp_us{};
    std::array<float, 4> rotation_wxyz{};
    std::array<float, 3> angular_velocity_xyz{};
    std::array<float, 3> linear_acceleration_xyz{};
  };

  struct ImageReference
  {
    bool valid{false};
    ImageEvent sample{};
  };

  struct PendingImageState
  {
    bool valid{false};
    ImageEvent sample{};
    bool cadence_observed{false};
    bool sync_candidate_valid{false};
    uint64_t sync_candidate_sensor_timestamp_us{0};
    uint64_t sync_candidate_period_us{0};
  };

  struct CadenceObservation
  {
    CameraFrameSyncCore::CadenceState gyro{};
    CameraFrameSyncCore::CadenceState accl{};
    CameraFrameSyncCore::CadenceState quat{};
    CameraFrameSyncCore::CadenceState image{};
  };

  // 这一组量描述“上一张已接受图像”和“上一条同步 IMU”之间的稳定节拍关系。
  // 锁定后只沿着这组关系递推，不再每帧重新做全局锁定。
  struct SyncRelation
  {
    uint64_t base_image_sensor_period_us{0};
    uint64_t last_imu_sensor_period_us{0};
    uint64_t sync_imu_sensor_period_us{0};
    uint64_t last_image_sensor_timestamp_us{0};
    uint64_t last_sync_imu_sensor_timestamp_us{0};
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

    bool Valid() const
    {
      return sync_imu != nullptr && next_sync_sensor_period_us != 0;
    }
  };

  // 这里按“100Hz 图像基线 + 1kHz IMU + 100ms 级搜索窗口”留余量，不再保留 1k/2k 级大队列。
  static constexpr size_t imu_ingress_length = 128;
  static constexpr size_t image_event_ingress_length = 32;
  static constexpr size_t pending_limit = 256;
  static constexpr size_t history_limit = 256;
  static constexpr uint32_t cadence_stable_gaps = 2;
  static constexpr uint64_t raw_cadence_min_tolerance_us = 300ULL;
  static constexpr uint64_t image_cadence_min_tolerance_us = 1500ULL;

  static uint64_t ImageGapToleranceUs(const SyncRelation& relation)
  {
    return CameraFrameSyncCore::ImageGapToleranceUs(relation.base_image_sensor_period_us);
  }

  void ResetLockedRelation()
  {
    relation_.sync_imu_sensor_period_us = 0;
    relation_.last_image_sensor_timestamp_us = 0;
    relation_.last_sync_imu_sensor_timestamp_us = 0;
  }

  void ResetImageObservation()
  {
    relation_.base_image_sensor_period_us = 0;
    last_observed_image_ = {};
    last_normal_image_ = {};
  }

  void ResetCadenceObservation()
  {
    cadence_ = {};
    ResetImageObservation();
  }

  void SetObservingState()
  {
    lock_state_.state = SyncState::OBSERVING;
  }

  void ResetSyncTracking()
  {
    ClearPendingProbe();
    ResetLockedRelation();
    ResetImageObservation();
    SetObservingState();
  }

  void ResetRuntimeState()
  {
    gyro_ingress_.Reset();
    accl_ingress_.Reset();
    quat_ingress_.Reset();
    image_event_ingress_.Reset();

    pending_gyros_ = {};
    pending_accls_ = {};
    pending_quats_ = {};
    pending_image_events_ = {};
    imu_history_ = {};
    relation_ = {};

    overflowed_.store(false, std::memory_order_relaxed);
    ClearPendingImage();
    ClearPendingProbe();
    ResetLockedRelation();
    ResetCadenceObservation();
    SetObservingState();
  }

  bool CadenceReady() const
  {
    return cadence_.gyro.stable && cadence_.image.stable;
  }

  void ObserveCadenceUpdate(CameraFrameSyncCore::CadenceUpdate update)
  {
    switch (update)
    {
      case CameraFrameSyncCore::CadenceUpdate::NO_GAP:
      case CameraFrameSyncCore::CadenceUpdate::WARMING:
      case CameraFrameSyncCore::CadenceUpdate::STABLE:
        break;
      case CameraFrameSyncCore::CadenceUpdate::BROKEN:
        ResetSyncTracking();
        break;
    }
  }

  void ObserveSampleCadence(CameraFrameSyncCore::CadenceState& cadence, uint64_t timestamp_us)
  {
    ObserveCadenceUpdate(CameraFrameSyncCore::ObserveCadence(
        cadence, timestamp_us, cadence_stable_gaps, raw_cadence_min_tolerance_us));
  }

  CameraFrameSyncCore::CadenceUpdate ObserveImageCadence(const ImageEvent& image)
  {
    const uint64_t image_timestamp_us = static_cast<uint64_t>(image.sensor_timestamp_us);
    // probe 的 2T 图像 gap 是主动扰动，不算图像流失稳，但要推进图像侧基线。
    if (probe_pending_ && cadence_.image.stable && relation_.base_image_sensor_period_us != 0 &&
        cadence_.image.has_last_timestamp &&
        image_timestamp_us > cadence_.image.last_timestamp_us)
    {
      const uint64_t image_gap_sensor_us = image_timestamp_us - cadence_.image.last_timestamp_us;
      if (CameraFrameSyncCore::AbsDiffUs(image_gap_sensor_us,
                                         relation_.base_image_sensor_period_us * 2ULL) <=
          ImageGapToleranceUs(relation_))
      {
        cadence_.image.last_timestamp_us = image_timestamp_us;
        ObserveCadenceUpdate(CameraFrameSyncCore::CadenceUpdate::STABLE);
        return CameraFrameSyncCore::CadenceUpdate::STABLE;
      }
    }

    const auto update = CameraFrameSyncCore::ObserveCadence(
        cadence_.image, image_timestamp_us, cadence_stable_gaps,
        image_cadence_min_tolerance_us);
    ObserveCadenceUpdate(update);
    return update;
  }

  void RecordStableImageAnchor(const ImageEvent& image)
  {
    if (!cadence_.image.stable)
    {
      return;
    }

    relation_.base_image_sensor_period_us = cadence_.image.period_us;
    last_normal_image_.valid = true;
    last_normal_image_.sample = image;
  }

  uint32_t EstimatedStrideSamples() const
  {
    return CameraFrameSyncCore::EstimateStrideSamples(relation_.base_image_sensor_period_us,
                                                      relation_.last_imu_sensor_period_us);
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

  bool IsNormalImageGap(uint64_t image_gap_sensor_us) const
  {
    if (relation_.base_image_sensor_period_us == 0 || image_gap_sensor_us == 0)
    {
      return false;
    }

    return CameraFrameSyncCore::AbsDiffUs(image_gap_sensor_us,
                                          relation_.base_image_sensor_period_us) <=
           ImageGapToleranceUs(relation_);
  }

  bool IsProbeImageGap(uint64_t image_gap_sensor_us) const
  {
    if (relation_.base_image_sensor_period_us == 0 || image_gap_sensor_us == 0)
    {
      return false;
    }

    return CameraFrameSyncCore::AbsDiffUs(image_gap_sensor_us,
                                          relation_.base_image_sensor_period_us * 2ULL) <=
           ImageGapToleranceUs(relation_);
  }

  bool IsSynced() const { return lock_state_.state == SyncState::SYNCED; }

  void ClearPendingProbe()
  {
    probe_pending_ = false;
  }

  void ClearPendingImage() { pending_image_ = {}; }

  bool AcquireInitialWritableImage()
  {
    if (image_topic_.CreateData(current_image_) != LibXR::ErrorCode::OK)
    {
      return false;
    }
    return current_image_.GetData() != nullptr;
  }

  static void CommitImageAdapter(bool, Self* self, ImageFrame*& next_image)
  {
    next_image = self->CommitImageAndLeaseNext();
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
    const auto& gyro = *reinterpret_cast<const GyroStamped*>(data.addr_);
    if (self->gyro_ingress_.Push(QueuedGyro{.sample = gyro}) != LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  static void OnAcclStatic(bool, Self* self, LibXR::RawData& data)
  {
    const auto& accl = *reinterpret_cast<const AcclStamped*>(data.addr_);
    if (self->accl_ingress_.Push(QueuedAccl{.sample = accl}) != LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  static void OnQuatStatic(bool, Self* self, LibXR::RawData& data)
  {
    const auto& quat = *reinterpret_cast<const QuatStamped*>(data.addr_);
    if (self->quat_ingress_.Push(QueuedQuat{.sample = quat}) != LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  static void OnImageEventStatic(bool, Self* self, LibXR::RawData& data)
  {
    const auto& image_event = *reinterpret_cast<const ImageEvent*>(data.addr_);
    if (self->image_event_ingress_.Push(QueuedImageEvent{.sample = image_event}) !=
        LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }

    // 图像事件就是同步触发点：原始回调只入队，所有对齐状态都在这里串行推进。
    self->ProcessPendingSyncWork();
  }

  void ProcessPendingSyncWork()
  {
    LibXR::Mutex::LockGuard lock(sync_state_mutex_);
    DrainIngressQueues();
    HandleOverflowRecovery();
    MaybeSendProbe();
    DrainPendingImageEvents();
  }

  void DrainIngressQueues()
  {
    QueuedGyro gyro{};
    while (gyro_ingress_.Pop(gyro) == LibXR::ErrorCode::OK)
    {
      ObserveSampleCadence(cadence_.gyro,
                           static_cast<uint64_t>(gyro.sample.sensor_timestamp_us));
      if (pending_gyros_.PushBackDropOldest(gyro))
      {
        overflowed_.store(true, std::memory_order_relaxed);
      }
    }

    QueuedAccl accl{};
    while (accl_ingress_.Pop(accl) == LibXR::ErrorCode::OK)
    {
      ObserveSampleCadence(cadence_.accl,
                           static_cast<uint64_t>(accl.sample.sensor_timestamp_us));
      if (pending_accls_.PushBackDropOldest(accl))
      {
        overflowed_.store(true, std::memory_order_relaxed);
      }
    }

    QueuedQuat quat{};
    while (quat_ingress_.Pop(quat) == LibXR::ErrorCode::OK)
    {
      ObserveSampleCadence(cadence_.quat,
                           static_cast<uint64_t>(quat.sample.sensor_timestamp_us));
      if (pending_quats_.PushBackDropOldest(quat))
      {
        overflowed_.store(true, std::memory_order_relaxed);
      }
    }

    QueuedImageEvent image_event{};
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
        relation_.last_imu_sensor_period_us, history_limit,
        [](const GyroStamped& gyro, const AcclStamped& accl, const QuatStamped& quat)
        {
          return AssembledImu{
              .sensor_timestamp_us = gyro.sensor_timestamp_us,
              .rotation_wxyz = quat.rotation_wxyz,
              .angular_velocity_xyz = gyro.angular_velocity_xyz,
              .linear_acceleration_xyz = accl.linear_acceleration_xyz,
          };
        }))
    {
    }
  }

  void RememberObservedImage(const ImageEvent& image)
  {
    last_observed_image_.valid = true;
    last_observed_image_.sample = image;
  }

  void DrainPendingImageEvents()
  {
    while (pending_image_.valid || !pending_image_events_.Empty())
    {
      if (!pending_image_.valid)
      {
        pending_image_.valid = true;
        pending_image_.sample = pending_image_events_.Front().sample;
        pending_image_events_.PopFront();
      }

      PendingImageState& image = pending_image_;
      if (GetSyncMode() == SyncMode::LATEST_IMU)
      {
        // 兼容旧模式：不做 probe 锁定，直接把当前看到的最新 IMU 绑定给这张图像。
        const uint64_t image_timestamp_us =
            static_cast<uint64_t>(image.sample.sensor_timestamp_us);
        if (last_observed_image_.valid &&
            image_timestamp_us <=
                static_cast<uint64_t>(last_observed_image_.sample.sensor_timestamp_us))
        {
          ClearPendingImage();
          ResetLockedRelation();
          ResetImageObservation();
          SetObservingState();
          continue;
        }

        const ImageDecision decision = image.sync_candidate_valid
                                           ? ResumePendingMatch(image)
                                           : TryProcessLatestImuImage(image);
        if (decision == ImageDecision::WAIT)
        {
          break;
        }

        RememberObservedImage(image.sample);
        ClearPendingImage();
        continue;
      }

      auto cadence_update = CameraFrameSyncCore::CadenceUpdate::NO_GAP;
      if (!image.cadence_observed)
      {
        cadence_update = ObserveImageCadence(image.sample);
        image.cadence_observed = true;
      }
      if (!last_observed_image_.valid)
      {
        RememberObservedImage(image.sample);
        ClearPendingImage();
        continue;
      }

      const uint64_t image_timestamp_us = static_cast<uint64_t>(image.sample.sensor_timestamp_us);
      const uint64_t last_image_timestamp_us =
          static_cast<uint64_t>(last_observed_image_.sample.sensor_timestamp_us);
      if (image_timestamp_us <= last_image_timestamp_us)
      {
        ClearPendingImage();
        ClearPendingProbe();
        ResetLockedRelation();
        ResetCadenceObservation();
        SetObservingState();
        continue;
      }

      const uint64_t image_gap_sensor_us = image_timestamp_us - last_image_timestamp_us;
      if (cadence_update == CameraFrameSyncCore::CadenceUpdate::BROKEN)
      {
        RememberObservedImage(image.sample);
        ClearPendingImage();
        continue;
      }

      const bool probe_pending = probe_pending_;

      ImageDecision decision = ImageDecision::REJECT;
      if (image.sync_candidate_valid)
      {
        decision = ResumePendingMatch(image);
      }
      else if (probe_pending)
      {
        decision = IsProbeImageGap(image_gap_sensor_us)
                       ? TryLockFromProbe(image, image_gap_sensor_us)
                       : ImageDecision::REJECT;
      }
      else if (IsSynced())
      {
        decision = IsNormalImageGap(image_gap_sensor_us)
                       ? TryProcessSyncedImage(image, image_gap_sensor_us)
                       : ImageDecision::REJECT;
      }
      else
      {
        RecordStableImageAnchor(image.sample);
        RememberObservedImage(image.sample);
        ClearPendingImage();
        MaybeSendProbe();
        continue;
      }

      if (decision == ImageDecision::WAIT)
      {
        break;
      }

      if (decision == ImageDecision::ACCEPT)
      {
        if (probe_pending)
        {
          ClearPendingProbe();
        }
        else
        {
          RecordStableImageAnchor(image.sample);
        }
        RememberObservedImage(image.sample);
        ClearPendingImage();
        continue;
      }

      if (probe_pending)
      {
        ClearPendingProbe();
      }
      ClearPendingImage();
      ResetSyncTracking();
      MaybeSendProbe();
    }
  }

  void MaybeSendProbe()
  {
    if (GetSyncMode() != SyncMode::RAW_PROBE)
    {
      return;
    }

    if (IsSynced())
    {
      return;
    }

    if (!CadenceReady() || relation_.base_image_sensor_period_us == 0 ||
        EstimatedSyncImuSensorPeriodUs() == 0 || !last_normal_image_.valid)
    {
      return;
    }
    if (probe_pending_)
    {
      return;
    }

    SensorSyncCmd cmd{};
    // 下位机只需要做一次 2T 探针，观察到节拍变化后就会自动恢复正常频率。
    sensor_sync_cmd_topic_.Publish(cmd);
    probe_pending_ = true;
  }

  ImageDecision TryProcessLatestImuImage(PendingImageState& image)
  {
    if (imu_history_.Empty())
    {
      return ImageDecision::WAIT;
    }

    const uint64_t sync_imu_period_us =
        relation_.last_imu_sensor_period_us != 0 ? relation_.last_imu_sensor_period_us : 1ULL;
    return PublishOrRememberMatch(
        image, SyncMatch{
                   .sync_imu = &imu_history_.Back(),
                   .next_sync_sensor_period_us = sync_imu_period_us,
               });
  }

  bool ImuHistoryReached(uint64_t target_timestamp_us) const
  {
    return !imu_history_.Empty() && imu_history_.Back().sensor_timestamp_us >= target_timestamp_us;
  }

  bool ImuHistorySpanReady(uint64_t required_span_us) const
  {
    if (imu_history_.Size() < 2)
    {
      return false;
    }

    return imu_history_.Back().sensor_timestamp_us - imu_history_.Front().sensor_timestamp_us >=
           required_span_us;
  }

  ImageDecision PublishMatchedImage(const ImageEvent& image,
                                    const SyncMatch& match)
  {
    // 先确定“同步帧是哪一条 IMU”，再只在 IMU 自己时间域里应用 offset 取最终样本。
    const int32_t offset_us = offset_us_.load(std::memory_order_relaxed);
    if (NeedMoreImuForOffset(*match.sync_imu, offset_us))
    {
      return ImageDecision::WAIT;
    }

    const AssembledImu* final_imu = FindFinalImu(*match.sync_imu, offset_us);
    if (final_imu == nullptr)
    {
      return ImageDecision::REJECT;
    }

    PublishSyncedImu(image.sensor_timestamp_us, *final_imu);
    relation_.sync_imu_sensor_period_us = match.next_sync_sensor_period_us;
    relation_.last_image_sensor_timestamp_us = static_cast<uint64_t>(image.sensor_timestamp_us);
    relation_.last_sync_imu_sensor_timestamp_us = match.sync_imu->sensor_timestamp_us;
    CameraFrameSyncCore::ObserveGoodFrame(lock_state_);
    return ImageDecision::ACCEPT;
  }

  ImageDecision PublishOrRememberMatch(PendingImageState& image, const SyncMatch& match)
  {
    const ImageDecision decision = PublishMatchedImage(image.sample, match);
    if (decision == ImageDecision::WAIT)
    {
      image.sync_candidate_valid = true;
      image.sync_candidate_sensor_timestamp_us = match.sync_imu->sensor_timestamp_us;
      image.sync_candidate_period_us = match.next_sync_sensor_period_us;
      return decision;
    }

    image.sync_candidate_valid = false;
    image.sync_candidate_sensor_timestamp_us = 0;
    image.sync_candidate_period_us = 0;
    return decision;
  }

  ImageDecision ResumePendingMatch(PendingImageState& image)
  {
    if (!image.sync_candidate_valid)
    {
      return ImageDecision::REJECT;
    }

    const AssembledImu* sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
        imu_history_, image.sync_candidate_sensor_timestamp_us, 0);
    if (sync_imu == nullptr ||
        sync_imu->sensor_timestamp_us != image.sync_candidate_sensor_timestamp_us)
    {
      return ImageDecision::REJECT;
    }

    return PublishOrRememberMatch(
        image, SyncMatch{
                   .sync_imu = sync_imu,
                   .next_sync_sensor_period_us = image.sync_candidate_period_us,
               });
  }

  SyncMatch BuildTrackedSyncMatch(const AssembledImu& sync_imu,
                                  uint64_t expected_sync_sensor_gap_us,
                                  uint64_t sensor_gap_tolerance_us) const
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

    return SyncMatch{
        .sync_imu = &sync_imu,
        .next_sync_sensor_period_us = sync_sensor_gap_us,
    };
  }

  SyncMatch SelectTrackedSyncMatch(uint64_t expected_sync_sensor_gap_us,
                                   uint64_t sensor_gap_tolerance_us) const
  {
    const uint64_t expected_sync_sensor_timestamp_us =
        relation_.last_sync_imu_sensor_timestamp_us + expected_sync_sensor_gap_us;
    const AssembledImu* predicted_sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
        imu_history_, expected_sync_sensor_timestamp_us, sensor_gap_tolerance_us);
    return predicted_sync_imu != nullptr
               ? BuildTrackedSyncMatch(
                     *predicted_sync_imu, expected_sync_sensor_gap_us, sensor_gap_tolerance_us)
               : SyncMatch{};
  }

  SyncMatch SelectProbeSyncMatch(uint64_t expected_probe_sensor_gap_us,
                                 uint64_t sensor_gap_tolerance_us) const
  {
    SyncMatch best_match{};
    uint64_t best_sensor_gap_error_us = std::numeric_limits<uint64_t>::max();
    uint64_t best_sync_timestamp_us = 0;

    for (size_t i = imu_history_.Size(); i > 0; --i)
    {
      const AssembledImu& sync_imu = imu_history_[i - 1];
      if (sync_imu.sensor_timestamp_us < expected_probe_sensor_gap_us)
      {
        continue;
      }

      const uint64_t prev_target_sensor_timestamp_us =
          sync_imu.sensor_timestamp_us - expected_probe_sensor_gap_us;
      const AssembledImu* prev_sync_imu = CameraFrameSyncCore::FindBySensorTimestamp(
          imu_history_, prev_target_sensor_timestamp_us, sensor_gap_tolerance_us);
      if (prev_sync_imu == nullptr ||
          sync_imu.sensor_timestamp_us <= prev_sync_imu->sensor_timestamp_us)
      {
        continue;
      }

      const uint64_t sync_sensor_gap_us =
          sync_imu.sensor_timestamp_us - prev_sync_imu->sensor_timestamp_us;
      const uint64_t sensor_gap_error_us =
          CameraFrameSyncCore::AbsDiffUs(sync_sensor_gap_us, expected_probe_sensor_gap_us);
      const bool better_match =
          !best_match.Valid() ||
          sensor_gap_error_us < best_sensor_gap_error_us ||
          (sensor_gap_error_us == best_sensor_gap_error_us &&
           sync_imu.sensor_timestamp_us > best_sync_timestamp_us);
      if (sync_sensor_gap_us < 2 || !better_match)
      {
        continue;
      }

      best_match = SyncMatch{
          .sync_imu = &sync_imu,
          .next_sync_sensor_period_us = sync_sensor_gap_us / 2ULL,
      };
      best_sensor_gap_error_us = sensor_gap_error_us;
      best_sync_timestamp_us = sync_imu.sensor_timestamp_us;
    }

    return best_match;
  }

  ImageDecision TryLockFromProbe(PendingImageState& probe, uint64_t image_gap_sensor_us)
  {
    if (!last_normal_image_.valid || !IsProbeImageGap(image_gap_sensor_us))
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
    if (!ImuHistorySpanReady(expected_probe_sensor_gap_us))
    {
      return ImageDecision::WAIT;
    }

    const SyncMatch match =
        SelectProbeSyncMatch(expected_probe_sensor_gap_us, sensor_gap_tolerance_us);
    if (!match.Valid())
    {
      return ImageDecision::REJECT;
    }

    return PublishOrRememberMatch(probe, match);
  }

  ImageDecision TryProcessSyncedImage(PendingImageState& image, uint64_t image_gap_sensor_us)
  {
    const uint64_t image_timestamp_us = static_cast<uint64_t>(image.sample.sensor_timestamp_us);
    if (relation_.last_image_sensor_timestamp_us == 0 ||
        relation_.last_sync_imu_sensor_timestamp_us == 0)
    {
      return ImageDecision::REJECT;
    }

    if (image_timestamp_us <= relation_.last_image_sensor_timestamp_us ||
        !IsNormalImageGap(image_gap_sensor_us))
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
    const uint64_t expected_sync_sensor_timestamp_us =
        relation_.last_sync_imu_sensor_timestamp_us + expected_sync_sensor_gap_us;
    if (!ImuHistoryReached(expected_sync_sensor_timestamp_us))
    {
      return ImageDecision::WAIT;
    }

    const SyncMatch match =
        SelectTrackedSyncMatch(expected_sync_sensor_gap_us, sensor_gap_tolerance_us);
    if (!match.Valid())
    {
      return ImageDecision::REJECT;
    }

    return PublishOrRememberMatch(image, match);
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

  void HandleOverflowRecovery()
  {
    if (overflowed_.exchange(false, std::memory_order_relaxed))
    {
      ClearPendingImage();
      ClearPendingProbe();
      ResetLockedRelation();
      ResetCadenceObservation();
      SetObservingState();
    }
  }

 private:
  std::string_view image_topic_name_;
  std::string_view imu_topic_name_;
  std::string sensor_name_{};
  std::string gyro_topic_name_{};
  std::string accl_topic_name_{};
  std::string quat_topic_name_{};
  std::string image_event_topic_name_{};

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
  LibXR::LockFreeQueue<QueuedGyro> gyro_ingress_{imu_ingress_length};
  LibXR::LockFreeQueue<QueuedAccl> accl_ingress_{imu_ingress_length};
  LibXR::LockFreeQueue<QueuedQuat> quat_ingress_{imu_ingress_length};
  LibXR::LockFreeQueue<QueuedImageEvent> image_event_ingress_{image_event_ingress_length};
  std::atomic<bool> overflowed_{false};

  // 下面这些状态只在 image_event 触发的串行路径里访问。
  FixedRingBuffer<QueuedGyro, pending_limit> pending_gyros_{};
  FixedRingBuffer<QueuedAccl, pending_limit> pending_accls_{};
  FixedRingBuffer<QueuedQuat, pending_limit> pending_quats_{};
  FixedRingBuffer<QueuedImageEvent, pending_limit> pending_image_events_{};
  FixedRingBuffer<AssembledImu, history_limit> imu_history_{};
  LibXR::Mutex sync_state_mutex_{};

  std::atomic<int32_t> offset_us_{0};
  std::atomic<SyncMode> sync_mode_{SyncMode::RAW_PROBE};
  SyncLockState lock_state_{};
  CadenceObservation cadence_{};
  SyncRelation relation_{};
  ImageReference last_observed_image_{};
  ImageReference last_normal_image_{};
  PendingImageState pending_image_{};
  bool probe_pending_{false};
};
