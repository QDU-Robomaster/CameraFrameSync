#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机共享图像桥与原始 imu/image_event 同步器 / Shared image bridge and raw imu/image_event synchronizer
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
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "linux_shared_topic.hpp"
#include "logger.hpp"

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
          while (!pending_imus_.empty() &&
                 pending_imus_.front().timestamp_us < image_timestamp_us)
          {
            pending_imus_.pop_front();
          }

          if (!pending_imus_.empty() &&
              pending_imus_.front().timestamp_us == image_timestamp_us)
          {
            out.image = std::move(image_data);
            out.imu = pending_imus_.front();
            pending_imus_.pop_front();
            return LibXR::ErrorCode::OK;
          }

          if (!pending_imus_.empty() &&
              pending_imus_.front().timestamp_us > image_timestamp_us)
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
        pending_imus_.push_back(imu);
        while (pending_imus_.size() > kQueueLength)
        {
          pending_imus_.pop_front();
        }
      }
    }

   private:
    typename ImageTopic::Subscriber image_sub_;
    LibXR::LockFreeQueue<ImuStamped> imu_queue_;
    LibXR::Topic::QueuedSubscriber imu_sub_;
    std::deque<ImuStamped> pending_imus_{};
  };

  CameraFrameSync(LibXR::HardwareContainer&, LibXR::ApplicationManager&, Base& camera)
      : camera_(camera),
        image_topic_name_(camera.ImageTopicName()),
        imu_topic_name_(camera.ImuTopicName()),
        gyro_topic_name_(std::string((camera.Name() != nullptr && camera.Name()[0] != '\0')
                                         ? camera.Name()
                                         : "camera") +
                         "_gyro"),
        accl_topic_name_(std::string((camera.Name() != nullptr && camera.Name()[0] != '\0')
                                         ? camera.Name()
                                         : "camera") +
                         "_accl"),
        quat_topic_name_(std::string((camera.Name() != nullptr && camera.Name()[0] != '\0')
                                         ? camera.Name()
                                         : "camera") +
                         "_quat"),
        image_event_topic_name_(
            std::string((camera.Name() != nullptr && camera.Name()[0] != '\0')
                            ? camera.Name()
                            : "camera") +
            "_image_event"),
        sync_config_topic_name_(
            std::string((camera.Name() != nullptr && camera.Name()[0] != '\0')
                            ? camera.Name()
                            : "camera") +
            "_sync_config"),
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
    if (!camera_.RegisterImageSink(this, current_image_.GetData(), CommitImageAdapter))
    {
      current_image_.Reset();
      throw std::runtime_error("CameraFrameSync: image sink registration failed");
    }

    gyro_topic_.RegisterCallback(gyro_cb_);
    accl_topic_.RegisterCallback(accl_cb_);
    quat_topic_.RegisterCallback(quat_cb_);
    image_event_topic_.RegisterCallback(image_event_cb_);

    running_.store(true, std::memory_order_release);
    sync_thread_.Create(this, SyncThreadMain, "CameraFrameSync",
                        static_cast<size_t>(1024 * 128),
                        LibXR::Thread::Priority::REALTIME);
  }

  ~CameraFrameSync() { running_.store(false, std::memory_order_release); }

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
  enum class SyncState : uint8_t
  {
    UNSYNCED = 0,
    LOCKING = 1,
    SYNCED = 2,
    RECOVERING = 3,
  };

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
  static constexpr uint32_t kPollMs = 2;

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

  static uint64_t AbsDiff(uint64_t lhs, uint64_t rhs)
  {
    return lhs >= rhs ? (lhs - rhs) : (rhs - lhs);
  }

  static uint64_t ApplyOffset(uint64_t base_us, int32_t offset_us)
  {
    if (offset_us >= 0)
    {
      return base_us + static_cast<uint64_t>(offset_us);
    }

    const uint64_t abs_offset = static_cast<uint64_t>(-static_cast<int64_t>(offset_us));
    return base_us > abs_offset ? (base_us - abs_offset) : 0ULL;
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
    if (self->gyro_ingress_.Push(*reinterpret_cast<const GyroStamped*>(data.addr_)) !=
        LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  static void OnAcclStatic(bool, Self* self, LibXR::RawData& data)
  {
    self->last_accl_rx_ms_.store(LibXR::Thread::GetTime(), std::memory_order_relaxed);
    if (self->accl_ingress_.Push(*reinterpret_cast<const AcclStamped*>(data.addr_)) !=
        LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  static void OnQuatStatic(bool, Self* self, LibXR::RawData& data)
  {
    self->last_quat_rx_ms_.store(LibXR::Thread::GetTime(), std::memory_order_relaxed);
    if (self->quat_ingress_.Push(*reinterpret_cast<const QuatStamped*>(data.addr_)) !=
        LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
    }
  }

  static void OnImageEventStatic(bool, Self* self, LibXR::RawData& data)
  {
    self->last_image_event_rx_ms_.store(LibXR::Thread::GetTime(),
                                        std::memory_order_relaxed);
    if (self->image_event_ingress_.Push(*reinterpret_cast<const ImageEvent*>(data.addr_)) !=
        LibXR::ErrorCode::OK)
    {
      self->overflowed_.store(true, std::memory_order_relaxed);
      return;
    }
    self->image_event_sem_.Post();
  }

  static void SyncThreadMain(Self* self)
  {
    while (self->running_.load(std::memory_order_acquire))
    {
      (void)self->image_event_sem_.Wait(kPollMs);
      self->DrainIngress();
      self->ProcessImageEvents();
      self->HandleTimeouts();
    }
  }

  void DrainIngress()
  {
    GyroStamped gyro{};
    while (gyro_ingress_.Pop(gyro) == LibXR::ErrorCode::OK)
    {
      pending_gyros_.push_back(gyro);
      while (pending_gyros_.size() > kPendingLimit)
      {
        pending_gyros_.pop_front();
      }
    }

    AcclStamped accl{};
    while (accl_ingress_.Pop(accl) == LibXR::ErrorCode::OK)
    {
      pending_accls_.push_back(accl);
      while (pending_accls_.size() > kPendingLimit)
      {
        pending_accls_.pop_front();
      }
    }

    QuatStamped quat{};
    while (quat_ingress_.Pop(quat) == LibXR::ErrorCode::OK)
    {
      pending_quats_.push_back(quat);
      while (pending_quats_.size() > kPendingLimit)
      {
        pending_quats_.pop_front();
      }
    }

    ImageEvent image_event{};
    while (image_event_ingress_.Pop(image_event) == LibXR::ErrorCode::OK)
    {
      pending_image_events_.push_back(image_event);
    }

    AssembleImuHistory();
  }

  void AssembleImuHistory()
  {
    while (!pending_gyros_.empty())
    {
      const GyroStamped& gyro = pending_gyros_.front();
      auto accl_it = std::find_if(
          pending_accls_.begin(), pending_accls_.end(),
          [&](const AcclStamped& sample)
          { return sample.sensor_timestamp_us >= gyro.sensor_timestamp_us; });
      auto quat_it = std::find_if(
          pending_quats_.begin(), pending_quats_.end(),
          [&](const QuatStamped& sample)
          { return sample.sensor_timestamp_us >= gyro.sensor_timestamp_us; });
      if (accl_it == pending_accls_.end() || quat_it == pending_quats_.end())
      {
        break;
      }

      if (!imu_history_.empty() && gyro.sensor_timestamp_us > imu_history_.back().timestamp_us)
      {
        last_imu_period_us_ = gyro.sensor_timestamp_us - imu_history_.back().timestamp_us;
      }

      imu_history_.push_back(AssembledImu{
          .timestamp_us = gyro.sensor_timestamp_us,
          .rotation_wxyz = quat_it->rotation_wxyz,
          .angular_velocity_xyz = gyro.angular_velocity_xyz,
          .linear_acceleration_xyz = accl_it->linear_acceleration_xyz,
      });
      while (imu_history_.size() > kHistoryLimit)
      {
        imu_history_.pop_front();
      }

      pending_gyros_.pop_front();
      pending_accls_.erase(pending_accls_.begin(), std::next(accl_it));
      pending_quats_.erase(pending_quats_.begin(), std::next(quat_it));
    }
  }

  void ProcessImageEvents()
  {
    while (!pending_image_events_.empty())
    {
      const ImageEvent image_event = pending_image_events_.front();
      pending_image_events_.pop_front();

      const AssembledImu* imu = FindMatchedImu(image_event);
      if (imu == nullptr || !CadenceAcceptable(image_event, *imu))
      {
        EnterRecovering();
        continue;
      }

      PublishSyncedImu(image_event, *imu);
      OnLockSuccess(image_event, *imu);
    }
  }

  const AssembledImu* FindMatchedImu(const ImageEvent& image_event) const
  {
    if (imu_history_.empty())
    {
      return nullptr;
    }

    const SyncConfig config = const_cast<Self*>(this)->SnapshotSyncConfig();
    const uint64_t target_us =
        ApplyOffset(image_event.sensor_timestamp_us, config.offset_us);
    const uint64_t tolerance_us =
        std::max<uint64_t>(1000ULL, last_imu_period_us_ != 0 ? last_imu_period_us_ * 2 : 10000ULL);

    const AssembledImu* best = nullptr;
    uint64_t best_error = std::numeric_limits<uint64_t>::max();
    for (auto it = imu_history_.rbegin(); it != imu_history_.rend(); ++it)
    {
      const uint64_t error = AbsDiff(it->timestamp_us, target_us);
      if (error < best_error)
      {
        best = &(*it);
        best_error = error;
      }
      if (it->timestamp_us + tolerance_us < target_us)
      {
        break;
      }
    }

    return (best != nullptr && best_error <= tolerance_us) ? best : nullptr;
  }

  bool CadenceAcceptable(const ImageEvent& image_event, const AssembledImu& imu) const
  {
    if (last_image_timestamp_us_ == 0)
    {
      return true;
    }

    if (image_event.sensor_timestamp_us <= last_image_timestamp_us_ ||
        imu.timestamp_us <= last_synced_imu_timestamp_us_)
    {
      return false;
    }

    const uint64_t image_dt = image_event.sensor_timestamp_us - last_image_timestamp_us_;
    const uint64_t imu_dt = imu.timestamp_us - last_synced_imu_timestamp_us_;
    const uint64_t image_tol =
        std::max<uint64_t>(2000ULL, last_imu_period_us_ != 0 ? last_imu_period_us_ * 2 : 2000ULL);

    if (last_image_period_us_ != 0 && AbsDiff(image_dt, last_image_period_us_) > image_tol)
    {
      return false;
    }
    if (last_image_sequence_ != std::numeric_limits<uint32_t>::max() &&
        image_event.image_sequence != last_image_sequence_ + 1)
    {
      return false;
    }
    if (last_imu_period_us_ != 0 && imu_dt > last_imu_period_us_ * 4)
    {
      return false;
    }

    return true;
  }

  void PublishSyncedImu(const ImageEvent& image_event, const AssembledImu& imu)
  {
    ImuStamped synced{
        .timestamp_us = image_event.sensor_timestamp_us,
        .rotation_wxyz = imu.rotation_wxyz,
        .translation_xyz = {0.0f, 0.0f, 0.0f},
        .angular_velocity_xyz = imu.angular_velocity_xyz,
        .linear_acceleration_xyz = imu.linear_acceleration_xyz,
    };
    synced_imu_topic_.Publish(synced);
  }

  void OnLockSuccess(const ImageEvent& image_event, const AssembledImu& imu)
  {
    if (last_image_timestamp_us_ != 0)
    {
      last_image_period_us_ = image_event.sensor_timestamp_us - last_image_timestamp_us_;
    }

    last_image_timestamp_us_ = image_event.sensor_timestamp_us;
    last_synced_imu_timestamp_us_ = imu.timestamp_us;
    last_image_sequence_ = image_event.image_sequence;

    const SyncConfig config = SnapshotSyncConfig();
    switch (state_)
    {
      case SyncState::UNSYNCED:
      case SyncState::RECOVERING:
        state_ = SyncState::LOCKING;
        lock_confirm_count_ = 1;
        break;
      case SyncState::LOCKING:
        lock_confirm_count_++;
        if (lock_confirm_count_ >= config.relock_confirm_frames)
        {
          state_ = SyncState::SYNCED;
        }
        break;
      case SyncState::SYNCED:
        break;
    }
  }

  void EnterRecovering()
  {
    if (state_ != SyncState::UNSYNCED)
    {
      state_ = SyncState::RECOVERING;
      lock_confirm_count_ = 0;
    }
  }

  void HandleTimeouts()
  {
    const SyncConfig config = SnapshotSyncConfig();
    const uint32_t now_ms = LibXR::Thread::GetTime();
    const uint32_t last_imu_rx_ms =
        std::max(last_gyro_rx_ms_.load(std::memory_order_relaxed),
                 std::max(last_accl_rx_ms_.load(std::memory_order_relaxed),
                          last_quat_rx_ms_.load(std::memory_order_relaxed)));

    const bool image_timeout =
        last_image_event_rx_ms_.load(std::memory_order_relaxed) != 0 &&
        static_cast<uint32_t>(now_ms -
                              last_image_event_rx_ms_.load(std::memory_order_relaxed)) >
            config.image_timeout_ms;
    const bool imu_timeout =
        last_imu_rx_ms != 0 &&
        static_cast<uint32_t>(now_ms - last_imu_rx_ms) > config.imu_timeout_ms;

    if (overflowed_.exchange(false, std::memory_order_relaxed) || image_timeout ||
        imu_timeout)
    {
      EnterRecovering();
    }
  }

 private:
  Base& camera_;
  const char* image_topic_name_;
  const char* imu_topic_name_;
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

  LibXR::LockFreeQueue<GyroStamped> gyro_ingress_{kImuIngressLength};
  LibXR::LockFreeQueue<AcclStamped> accl_ingress_{kImuIngressLength};
  LibXR::LockFreeQueue<QuatStamped> quat_ingress_{kImuIngressLength};
  LibXR::LockFreeQueue<ImageEvent> image_event_ingress_{kImageEventIngressLength};
  LibXR::Semaphore image_event_sem_{0};
  std::atomic<bool> running_{false};
  std::atomic<bool> overflowed_{false};
  LibXR::Thread sync_thread_{};

  std::deque<GyroStamped> pending_gyros_{};
  std::deque<AcclStamped> pending_accls_{};
  std::deque<QuatStamped> pending_quats_{};
  std::deque<ImageEvent> pending_image_events_{};
  std::deque<AssembledImu> imu_history_{};

  LibXR::Mutex sync_config_mutex_{};
  SyncConfig sync_config_ = SanitizeSyncConfig(SyncConfig{});
  SyncState state_{SyncState::UNSYNCED};
  uint32_t lock_confirm_count_{0};
  uint32_t last_image_sequence_{std::numeric_limits<uint32_t>::max()};
  uint64_t last_image_timestamp_us_{0};
  uint64_t last_synced_imu_timestamp_us_{0};
  uint64_t last_imu_period_us_{0};
  uint64_t last_image_period_us_{0};

  std::atomic<uint32_t> last_gyro_rx_ms_{0};
  std::atomic<uint32_t> last_accl_rx_ms_{0};
  std::atomic<uint32_t> last_quat_rx_ms_{0};
  std::atomic<uint32_t> last_image_event_rx_ms_{0};
};
