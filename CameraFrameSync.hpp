#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: In-process camera frame ownership and MCU trigger timestamp synchronization
constructor_args:
  camera: '@nullptr'
template_args:
  - Layout:
      width: 720
      height: 540
      step: 2160
      encoding: CameraTypes::Encoding::BGR8
required_hardware: []
depends:
  - qdu-future/CameraBase
  - qdu-future/CameraSync
=== END MANIFEST === */
// clang-format on

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "AutoAimReplayBenchmark.hpp"
#include "CameraBase.hpp"
#include "CameraFrameSyncCore.hpp"
#include "CameraFrameSyncFrameQueue.hpp"
#include "CameraSync.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "transform.hpp"

enum class CameraFrameSyncMode : uint8_t
{
  TRIGGER = 0,
  RAW_PROBE = TRIGGER,  // Source-compatible name for existing product configs.
  LATEST_IMU = 1,
};

enum class CameraFrameSyncRawImuFrame : uint8_t
{
  BODY_X_RIGHT_Y_FORWARD_Z_UP = 0,
  X_FORWARD_Y_LEFT_Z_UP_TO_BODY = 1,
};

/**
 * CameraFrameSync retains CameraBase SharedFrame handles, pairs each image with
 * a real MCU FRAME_TRIGGER event, and publishes a transient SyncedFrame pointer.
 * All callbacks share one state mutex. Topic publication happens outside that
 * mutex so synchronous subscribers may re-enter other module APIs safely.
 */
template <CameraTypes::FrameLayout FrameLayoutV>
class CameraFrameSync : public LibXR::Application
{
 public:
  using Self = CameraFrameSync<FrameLayoutV>;
  using Base = CameraBase<FrameLayoutV>;
  using FrameGeometry = CameraTypes::FrameGeometry;
  using CameraCalibration = typename Base::CameraCalibration;
  using ImageFrame = typename Base::ImageFrame;
  using ImuStamped = typename Base::ImuStamped;
  using ImuVector = std::array<float, 3>;
  using QuatSample = std::array<float, 4>;
  using RawImuVector = Eigen::Matrix<float, 3, 1>;
  using RawQuatSample = LibXR::Quaternion<float>;
  using SharedFrame = typename Base::SharedFrame;
  using CameraImageTopicPayload = typename Base::ImageTopicPayload;
  using ProfileId = typename Base::ProfileId;
  using CameraProfile = typename Base::CameraProfile;
  using AppliedProfile = typename Base::AppliedProfile;
  using SyncMode = CameraFrameSyncMode;
  using RawImuFrame = CameraFrameSyncRawImuFrame;

  static inline constexpr auto frame_layout = FrameLayoutV;

  struct SyncedFrame
  {
    uint64_t sequence{};
    SharedFrame image{};
    ImuStamped imu{};

    [[nodiscard]] const ImageFrame* GetImageFrame() const noexcept { return image.Get(); }

    [[nodiscard]] bool Valid() const noexcept { return image.Valid(); }
  };

  /** Borrowed only for the duration of synchronous Topic callbacks. */
  using SyncedFrameTopicPayload = const SyncedFrame*;

  struct RuntimeParam
  {
    SyncMode mode = SyncMode::TRIGGER;
    int32_t offset_us = 0;
    std::string_view host_topic_domain_name = "shared_memory";
    std::string_view sync_command_topic_name = "camera_sync_command";
    std::string_view sync_result_topic_name = "camera_sync_result";
    uint32_t sync_active_level = 1;
    uint64_t camera_settle_us = 10000U;
    RawImuFrame raw_imu_frame = RawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP;
    std::string_view raw_quat_topic_name = {};
    std::string_view synced_frame_topic_name = {};
  };

  CameraFrameSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                  Base* camera);
  CameraFrameSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                  Base& camera);
  CameraFrameSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                  Base* camera, RuntimeParam runtime);
  CameraFrameSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                  Base& camera, RuntimeParam runtime);

  [[nodiscard]] const char* SyncedFrameTopicName() const;
  [[nodiscard]] const char* RawTopicDomainName() const;
  [[nodiscard]] const CameraCalibration& Calibration() const noexcept
  {
    return calibration_;
  }
  [[nodiscard]] std::span<const CameraProfile> Profiles() const noexcept;
  [[nodiscard]] ProfileId ActiveProfile() const;
  [[nodiscard]] SyncMode GetSyncMode() const;

  /**
   * Requests a different fixed profile. The call only admits the asynchronous
   * STOP/settle/SwitchProfile/START transaction. TRIGGER mode accepts requests
   * only while RUNNING; after failure, even the active profile is rejected.
   */
  LibXR::ErrorCode RequestProfile(ProfileId profile);

  /**
   * Selects a nearby IMU sample without changing the authoritative trigger
   * timestamp carried by SyncedFrame::imu.timestamp_us.
   */
  void SetOffsetUs(int32_t offset_us);

  void FlushPendingFrames();
  void OnMonitor() override;

 private:
  template <typename T, std::size_t Capacity>
  using SampleHistory = CameraFrameSyncCore::SampleHistory<T, Capacity>;

  template <typename T>
  class DropOldestQueue
  {
   public:
    explicit DropOldestQueue(std::size_t capacity) : queue_(capacity) {}

    [[nodiscard]] bool Empty() const { return queue_.Size() == 0U; }
    [[nodiscard]] bool Front(T& out) { return queue_.Peek(out) == LibXR::ErrorCode::OK; }
    void PopFront() { queue_.Pop(); }
    void Clear() { queue_.Reset(); }

    /** @return true when the oldest element had to be discarded. */
    [[nodiscard]] bool PushBackDropOldest(const T& value)
    {
      if (queue_.Push(value) == LibXR::ErrorCode::OK)
      {
        return false;
      }
      T dropped{};
      queue_.Pop(dropped);
      const auto result = queue_.Push(value);
      ASSERT(result == LibXR::ErrorCode::OK);
      return true;
    }

   private:
    LibXR::Queue<T> queue_;
  };

  struct Topics
  {
    Topics(const Base& camera, const RuntimeParam& runtime)
        : camera_image_name(camera.ImageTopicNameView()),
          synced_frame_name(runtime.synced_frame_topic_name.empty()
                                ? std::string(camera.ImageTopicNameView()) + "_synced"
                                : std::string(runtime.synced_frame_topic_name)),
          host_domain_name(runtime.host_topic_domain_name),
          sync_command_name(runtime.sync_command_topic_name),
          sync_result_name(runtime.sync_result_topic_name),
          raw_imu_prefix(camera.NameView()),
          gyro_name(raw_imu_prefix + "_gyro"),
          accl_name(raw_imu_prefix + "_accl"),
          quat_name(runtime.raw_quat_topic_name.empty()
                        ? raw_imu_prefix + "_quat"
                        : std::string(runtime.raw_quat_topic_name)),
          host_domain(host_domain_name.c_str()),
          camera_image(LibXR::Topic::FindOrCreate<CameraImageTopicPayload>(
              camera_image_name.c_str())),
          synced_frame(LibXR::Topic::FindOrCreate<SyncedFrameTopicPayload>(
              synced_frame_name.c_str())),
          sync_command(LibXR::Topic::FindOrCreate<CameraSync::SyncCommand>(
              sync_command_name.c_str(), &host_domain)),
          sync_result(LibXR::Topic::FindOrCreate<CameraSync::SyncEvent>(
              sync_result_name.c_str(), &host_domain)),
          gyro(LibXR::Topic::FindOrCreate<RawImuVector>(gyro_name.c_str(), &host_domain)),
          accl(LibXR::Topic::FindOrCreate<RawImuVector>(accl_name.c_str(), &host_domain)),
          quat(LibXR::Topic::FindOrCreate<RawQuatSample>(quat_name.c_str(), &host_domain))
    {
      ASSERT(!raw_imu_prefix.empty());
    }

    std::string camera_image_name;
    std::string synced_frame_name;
    std::string host_domain_name;
    std::string sync_command_name;
    std::string sync_result_name;
    std::string raw_imu_prefix;
    std::string gyro_name;
    std::string accl_name;
    std::string quat_name;
    LibXR::Topic::Domain host_domain;
    LibXR::Topic camera_image;
    LibXR::Topic synced_frame;
    LibXR::Topic sync_command;
    LibXR::Topic sync_result;
    LibXR::Topic gyro;
    LibXR::Topic accl;
    LibXR::Topic quat;
  };

  struct TopicCallbacks
  {
    explicit TopicCallbacks(Self* self)
        : image(LibXR::Topic::Callback::Create(OnImageStatic, self)),
          gyro(LibXR::Topic::Callback::Create(OnGyroStatic, self)),
          accl(LibXR::Topic::Callback::Create(OnAcclStatic, self)),
          quat(LibXR::Topic::Callback::Create(OnQuatStatic, self)),
          sync_result(LibXR::Topic::Callback::Create(OnSyncResultStatic, self))
    {
    }

    LibXR::Topic::Callback image;
    LibXR::Topic::Callback gyro;
    LibXR::Topic::Callback accl;
    LibXR::Topic::Callback quat;
    LibXR::Topic::Callback sync_result;
  };

  struct GyroSample
  {
    uint64_t sensor_timestamp_us{};
    ImuVector angular_velocity_xyz{};
  };

  struct AcclSample
  {
    uint64_t sensor_timestamp_us{};
    ImuVector linear_acceleration_xyz{};
  };

  struct QuatReading
  {
    uint64_t sensor_timestamp_us{};
    QuatSample rotation_wxyz{};
  };

  struct AssembledImu
  {
    uint64_t sensor_timestamp_us{};
    QuatSample rotation_wxyz{};
    ImuVector angular_velocity_xyz{};
    ImuVector linear_acceleration_xyz{};
  };

  struct ImageSample
  {
    uint64_t camera_timestamp_us{};
    uint64_t sequence{};
    SharedFrame frame{};
  };

  struct TriggerSample
  {
    uint64_t imu_timestamp_us{};
    uint32_t sequence{};
  };

  enum class OutboundKind : uint8_t
  {
    SYNC_COMMAND = 0,
    SYNCED_FRAME = 1,
  };

  struct OutboundItem
  {
    OutboundKind kind{OutboundKind::SYNC_COMMAND};
    CameraSync::SyncCommand command{};
    SyncedFrame frame{};
  };

  enum class ControlState : uint8_t
  {
    BYPASS = 0,
    WAIT_STOP_ACK,
    SETTLING,
    SWITCHING_CAMERA,
    WAIT_START_ACK,
    RUNNING,
    FAILED,
  };

  enum class ImuLookup : uint8_t
  {
    WAIT = 0,
    FOUND,
    MISSING,
  };

  static constexpr std::size_t pending_limit = 1024U;
  static constexpr std::size_t history_limit = 1024U;
  static constexpr std::size_t image_queue_capacity = Base::image_slot_count;
  static constexpr std::size_t trigger_queue_capacity = 256U;
  static constexpr std::size_t outbound_queue_capacity = Base::image_slot_count + 4U;
  static constexpr uint32_t max_camera_gap_stride = 128U;
  static constexpr uint64_t command_retry_interval_us = 100000U;
  static constexpr uint8_t command_retry_limit = 3U;
  static constexpr uint64_t offset_lookup_tolerance_us = 500U;

  static const char* SyncModeName(SyncMode mode);
  static const char* ControlStateName(ControlState state);
  static const char* RawImuFrameName(RawImuFrame frame);
  static ImuVector ToImuVector(const RawImuVector& data, RawImuFrame frame);
  static QuatSample ToQuatSample(const RawQuatSample& data, RawImuFrame frame);

  static void OnImageStatic(bool, Self* self, CameraImageTopicPayload borrowed);
  static void OnGyroStatic(bool, Self* self, LibXR::MicrosecondTimestamp timestamp,
                           const RawImuVector& data);
  static void OnAcclStatic(bool, Self* self, LibXR::MicrosecondTimestamp timestamp,
                           const RawImuVector& data);
  static void OnQuatStatic(bool, Self* self, LibXR::MicrosecondTimestamp timestamp,
                           const RawQuatSample& data);
  static void OnSyncResultStatic(bool, Self* self, LibXR::MicrosecondTimestamp timestamp,
                                 const CameraSync::SyncEvent& event);

  void HandleImage(SharedFrame image);
  void HandleSyncEventLocked(uint64_t event_timestamp_us,
                             const CameraSync::SyncEvent& event);

  [[nodiscard]] const CameraProfile* FindProfile(ProfileId profile) const noexcept;
  uint8_t AllocateSyncSequence();
  void BeginRestartLocked(ProfileId profile, bool switch_profile);
  [[nodiscard]] bool QueueCommandLocked(CameraSync::Operation operation,
                                        uint32_t trigger_period_us);
  void ArmCommandLocked(const CameraSync::SyncCommand& command);
  void ClearCommandLocked();
  [[nodiscard]] bool IsPendingCommandLocked(const CameraSync::SyncCommand& command) const;
  void MaybeRetryCommandLocked(uint64_t gyro_timestamp_us);
  [[nodiscard]] std::optional<ProfileId> AdvanceSettlingLocked(
      uint64_t gyro_timestamp_us);
  void ApplyProfileSwitch(ProfileId profile);
  void QueueStartLocked();
  void FailControlLocked();
  void RestartForMismatchLocked();
  void ResetMatchingLocked();
  void ResetRawImuLocked();

  void AssembleImuHistoryLocked();
  [[nodiscard]] bool TryAssembleOneImuLocked();
  void AcceptAssembledImuLocked(const AssembledImu& imu);
  void ProcessPendingLocked();
  void ProcessTriggerMatchesLocked();
  void ProcessLatestMatchesLocked();
  [[nodiscard]] ImuLookup FindImuLocked(uint64_t trigger_timestamp_us,
                                        const AssembledImu*& imu) const;
  [[nodiscard]] bool QueueSyncedFrameLocked(ImageSample& image, const AssembledImu& imu,
                                            uint64_t authoritative_timestamp_us);
  void CompleteMatchedImageLocked(uint64_t camera_timestamp_us,
                                  uint32_t trigger_sequence);

  [[nodiscard]] bool QueueOutboundLocked(OutboundItem item);
  void DispatchOutbound();
  void ProcessSyncWorkWithoutImage();

  Base* camera_{nullptr};
  CameraCalibration calibration_{};
  std::optional<Topics> topics_{};
  std::optional<TopicCallbacks> callbacks_{};
  std::optional<DropOldestQueue<GyroSample>> pending_gyros_{};
  std::optional<DropOldestQueue<AcclSample>> pending_accls_{};
  std::optional<DropOldestQueue<QuatReading>> pending_quats_{};
  std::optional<SampleHistory<AssembledImu, history_limit>> imu_history_{};

  mutable LibXR::Mutex sync_state_mutex_{};
  CameraFrameSyncDetail::FrameQueue<ImageSample, image_queue_capacity> images_{};
  CameraFrameSyncDetail::FrameQueue<TriggerSample, trigger_queue_capacity> triggers_{};
  CameraFrameSyncDetail::FrameQueue<OutboundItem, outbound_queue_capacity> outbound_{};

  SyncMode sync_mode_{SyncMode::TRIGGER};
  RawImuFrame raw_imu_frame_{RawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP};
  int32_t offset_us_{0};
  uint8_t sync_active_level_{1U};
  uint64_t camera_settle_us_{10000U};

  ControlState control_state_{ControlState::BYPASS};
  ProfileId active_profile_{ProfileId::WIDE};
  ProfileId requested_profile_{ProfileId::WIDE};
  uint32_t active_trigger_period_us_{0U};
  uint32_t requested_trigger_period_us_{0U};
  bool switch_profile_after_settle_{false};
  uint64_t settle_deadline_us_{0U};
  uint8_t active_start_seq_{0U};
  uint8_t next_sync_seq_{1U};

  CameraSync::SyncCommand pending_command_{};
  bool command_waiting_ack_{false};
  bool command_pending_dispatch_{false};
  uint8_t command_retry_count_{0U};
  uint64_t command_last_dispatch_imu_timestamp_us_{0U};
  uint64_t latest_raw_imu_timestamp_us_{0U};

  bool trigger_stream_valid_{false};
  uint32_t last_received_trigger_sequence_{0U};
  uint64_t last_received_trigger_timestamp_us_{0U};
  bool matched_pair_valid_{false};
  uint64_t last_matched_camera_timestamp_us_{0U};
  uint32_t last_matched_trigger_sequence_{0U};
  bool last_output_timestamp_valid_{false};
  uint64_t last_output_timestamp_us_{0U};
  uint64_t next_frame_sequence_{1U};
  bool dispatching_outbound_{false};
  bool geometry_reject_logged_{false};

  std::atomic<uint64_t> monitor_raw_gyro_count_{0U};
  std::atomic<uint64_t> monitor_raw_accl_count_{0U};
  std::atomic<uint64_t> monitor_raw_quat_count_{0U};
  std::atomic<uint64_t> monitor_assembled_imu_count_{0U};
  std::atomic<uint64_t> monitor_trigger_count_{0U};
  std::atomic<uint64_t> monitor_image_input_count_{0U};
  std::atomic<uint64_t> monitor_image_retained_count_{0U};
  std::atomic<uint64_t> monitor_image_drop_count_{0U};
  std::atomic<uint64_t> monitor_synced_output_count_{0U};
  std::atomic<uint64_t> monitor_reset_count_{0U};
  std::atomic<uint64_t> monitor_overflow_count_{0U};
};

#include "CameraFrameSyncImpl.hpp"
#include "CameraFrameSyncStateMachine.hpp"
