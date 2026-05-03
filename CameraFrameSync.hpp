#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机共享图像桥与原始 IMU 同步器
constructor_args:
  camera: '@camera'
  runtime:
    mode: CameraFrameSync<Info>::SyncMode::RAW_PROBE
    offset_us: 0
    host_topic_domain_name: "host"
    sync_command_topic_name: "camera_sync_command"
    sync_result_topic_name: "camera_sync_result"
    sync_probe_div: 3
    sync_active_level: 1
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
  - qdu-future/CameraSync
=== END MANIFEST === */
// clang-format on

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "CameraBase.hpp"
#include "CameraSync.hpp"
#include "app_framework.hpp"
#include "camera_frame_sync_core.hpp"
#include "camera_frame_sync_subscriber.hpp"
#include "libxr.hpp"
#include "linux_shared_topic.hpp"
#include "logger.hpp"
#include "transform.hpp"

/**
 * @brief 图像共享发布与 IMU 同步模块。
 *
 * 原始 gyro/accl/quat 回调只入队；同步回执只记录当前 probe 的命中结果。
 * 完整同步状态机仍然在图像提交时串行推进。
 * RAW_PROBE 模式使用 MCU 侧 CameraSync 的回执 timestamp 锁定 IMU 时间轴。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CameraFrameSync
{
 public:
  using Self = CameraFrameSync<CameraInfoV>;
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using ImageFrame = typename Base::ImageFrame;
  using ImuStamped = typename Base::ImuStamped;
  using ImuVector = std::array<float, 3>;
  using QuatSample = std::array<float, 4>;
  using RawImuVector = Eigen::Matrix<float, 3, 1>;
  using RawQuatSample = LibXR::Quaternion<float>;
  using ImageTopic = LibXR::LinuxSharedTopic<ImageFrame>;
  using ImageData = typename ImageTopic::Data;
  using ImageCommitCallback = typename Base::ImageCommitCallback;
  using SyncedFrame = CameraFrameSyncSyncedFrame<ImageTopic, ImuStamped>;
  using Subscriber = CameraFrameSyncSubscriber<ImageTopic, ImuStamped>;

  static inline constexpr CameraInfo camera_info = CameraInfoV;
  static constexpr LibXR::LinuxSharedTopicConfig image_topic_config{
      .slot_num = 8,
      .subscriber_num = 8,
      .queue_num = 2,
  };

  /**
   * @brief 图像与 IMU 的同步模式。
   */
  enum class SyncMode : uint8_t
  {
    RAW_PROBE = 0,   ///< 通过 CameraSync command/result 显式锁定。
    LATEST_IMU = 1,  ///< 数据源已同步时，图像直接绑定最新完整 IMU。
  };

  /**
   * @brief 运行时配置。
   *
   * Topic 名称只决定 Host 侧订阅/发布边界；图像和 IMU 的同步关系只由各自
   * sensor timestamp、CameraSync 回执和 offset_us 决定。
   */
  struct RuntimeParam
  {
    SyncMode mode = SyncMode::RAW_PROBE;  ///< 同步模式。
    int32_t offset_us = 0;                ///< IMU 时间域内的最终采样偏移。
    std::string_view host_topic_domain_name = "host";  ///< Host topic domain。
    std::string_view sync_command_topic_name =
        "camera_sync_command";  ///< CameraSync 命令 topic。
    std::string_view sync_result_topic_name =
        "camera_sync_result";           ///< CameraSync 回执 topic。
    uint32_t sync_probe_div = 3;         ///< MCU 临时分频系数。
    uint32_t sync_active_level = 1;      ///< 同步触发输出有效电平。
  };

  /**
   * @brief 使用默认 RAW_PROBE 配置创建同步桥。
   */
  CameraFrameSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                  Base& camera);

  /**
   * @brief 创建同步桥并绑定相机图像槽位。
   *
   * 构造函数会注册 CameraBase 图像 sink、订阅原始 IMU topic，并创建图像共享
   * topic。模块本身不创建线程；图像提交回调是同步状态机的推进点。
   */
  CameraFrameSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                  Base& camera, RuntimeParam runtime);

  /**
   * @brief 返回共享图像 topic 名称。
   */
  const char* ImageTopicName() const;

  /**
   * @brief 返回同步后 IMU topic 名称。
   */
  const char* ImuTopicName() const;

  /**
   * @brief 返回 Host 侧 topic domain 名称。
   */
  const char* HostTopicDomainName() const;

  /**
   * @brief 获取当前同步模式。
   */
  SyncMode GetSyncMode() const;

  /**
   * @brief 设置 IMU 时间域内的最终采样偏移。
   *
   * offset 不参与相机时间和 IMU 时间的绝对值比较，只在已经找到同步 IMU 后，
   * 沿 IMU 时间轴选择最终发布的姿态样本。
   */
  void SetOffsetUs(int32_t offset_us);

  /**
   * @brief 切换同步模式并重置运行时状态。
   */
  void SetSyncMode(SyncMode mode);

 private:
  using SyncState = CameraFrameSyncCore::SyncState;
  template <typename T, size_t Capacity>
  using SampleHistory = CameraFrameSyncCore::SampleHistory<T, Capacity>;

  template <typename T>
  class DropOldestQueue
  {
   public:
    explicit DropOldestQueue(size_t capacity) : queue_(capacity) {}

    bool Empty() const { return queue_.Size() == 0; }
    bool Front(T& out) { return queue_.Peek(out) == LibXR::ErrorCode::OK; }
    void PopFront() { queue_.Pop(); }
    void Clear() { queue_.Reset(); }

    bool PushBackDropOldest(const T& value)
    {
      if (queue_.Push(value) == LibXR::ErrorCode::OK)
      {
        return false;
      }

      T dropped{};
      queue_.Pop(dropped);
      const auto push_ans = queue_.Push(value);
      ASSERT(push_ans == LibXR::ErrorCode::OK);
      return true;
    }

   private:
    LibXR::LockFreeQueue<T> queue_;
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

  struct QueuedGyro
  {
    GyroSample sample{};
  };

  struct QueuedAccl
  {
    AcclSample sample{};
  };

  struct QueuedQuat
  {
    QuatReading sample{};
  };

  struct ImageSample
  {
    uint64_t sensor_timestamp_us{};
  };

  struct AssembledImu
  {
    uint64_t sensor_timestamp_us{};
    QuatSample rotation_wxyz{};
    ImuVector angular_velocity_xyz{};
    ImuVector linear_acceleration_xyz{};
  };

  struct PendingImage
  {
    bool valid{false};
    ImageSample sample{};
    bool cadence_observed{false};
    bool sync_candidate_valid{false};
    uint64_t sync_candidate_sensor_timestamp_us{0};
    uint64_t sync_candidate_period_us{0};
  };

  struct SyncMatch
  {
    const AssembledImu* sync_imu{nullptr};
    uint64_t sync_period_us{0};
  };

  struct SyncRelation
  {
    uint64_t image_period_us{0};
    uint64_t imu_period_us{0};
    uint64_t sync_period_us{0};
    uint64_t last_sync_imu_timestamp_us{0};
  };

  enum class ImageDecision : uint8_t
  {
    WAIT = 0,
    DONE = 1,
    RESET = 2,
  };

  static constexpr size_t imu_ingress_length = 128;
  static constexpr size_t pending_limit = 128;
  static constexpr size_t image_event_limit = 32;
  static constexpr size_t history_limit = 128;
  static constexpr uint32_t cadence_stable_gaps = 2;
  static constexpr uint64_t imu_cadence_tolerance_us = 300ULL;
  static constexpr uint64_t image_cadence_tolerance_us = 1500ULL;

  static std::string RequireSensorName(std::string_view name);
  static const char* SyncModeName(SyncMode mode);
  static const char* StateName(SyncState state);
  static ImuVector ToImuVector(const RawImuVector& data);
  static QuatSample ToQuatSample(const RawQuatSample& data);

  bool AcquireInitialWritableImage();
  static void CommitImageAdapter(bool, Self* self, ImageFrame*& next_image);
  ImageFrame* CommitImageAndLeaseNext();

  static void OnGyroStatic(bool, Self* self, LibXR::MicrosecondTimestamp timestamp,
                           const RawImuVector& data);
  static void OnAcclStatic(bool, Self* self, LibXR::MicrosecondTimestamp timestamp,
                           const RawImuVector& data);
  static void OnQuatStatic(bool, Self* self, LibXR::MicrosecondTimestamp timestamp,
                           const RawQuatSample& data);
  static void OnSyncResultStatic(bool, Self* self,
                                 LibXR::MicrosecondTimestamp timestamp,
                                 const CameraSync::SyncEvent& event);

  void ProcessCommittedImage(uint64_t image_timestamp_us);
  void ProcessSyncWorkWithoutImage();
  void CollectIncomingTopics();
  void AssembleImuHistory();
  bool TryAssembleOneImu();
  void ObserveImuCadence(uint64_t sensor_timestamp_us);

  void ProcessImageEvents();
  ImageDecision ProcessLatestImage(PendingImage& image);
  ImageDecision ProcessRawProbeImage(PendingImage& image);
  CameraFrameSyncCore::CadenceUpdate ObserveNormalImageCadence(uint64_t image_ts);
  void MaybeStartProbe();
  ImageDecision TryProbeImage(PendingImage& image);
  ImageDecision TryLatestImuMatch(PendingImage& image);
  ImageDecision TrySyncedImage(PendingImage& image);
  ImageDecision ResumePendingMatch(PendingImage& image);
  ImageDecision PublishOrRememberMatch(PendingImage& image,
                                       const SyncMatch& match);
  ImageDecision PublishMatchedImage(const ImageSample& image, const SyncMatch& match);
  void PublishSyncedImu(uint64_t image_timestamp_us, const AssembledImu& imu);

  uint64_t EstimatedSyncPeriodUs() const;
  bool IsNormalImageGap(uint64_t image_gap_us) const;
  bool IsProbeImageGap(uint64_t image_gap_us) const;
  bool ImuHistoryReached(uint64_t target_timestamp_us) const;
  void RememberImage(uint64_t image_timestamp_us);
  void ResetImageObservation();
  void ClearPendingProbe();
  void ClearPendingImage();
  void ResetLock(const char* reason, const char* detail);
  void ResetRuntimeState();
  void HandleOverflowRecovery();

 private:
  std::string image_topic_name_;
  std::string imu_topic_name_;
  std::string host_topic_domain_name_;
  LibXR::Topic::Domain host_topic_domain_;
  std::string sync_command_topic_name_;
  std::string sync_result_topic_name_;
  std::string sensor_name_;
  std::string gyro_topic_name_;
  std::string accl_topic_name_;
  std::string quat_topic_name_;

  ImageTopic image_topic_;
  ImageData current_image_{};
  LibXR::Topic synced_imu_topic_;
  LibXR::Topic sync_command_topic_;
  LibXR::Topic sync_result_topic_;
  LibXR::Topic gyro_topic_;
  LibXR::Topic accl_topic_;
  LibXR::Topic quat_topic_;
  LibXR::Topic::Callback gyro_cb_{};
  LibXR::Topic::Callback accl_cb_{};
  LibXR::Topic::Callback quat_cb_{};
  LibXR::Topic::Callback sync_result_cb_{};

  LibXR::LockFreeQueue<QueuedGyro> gyro_ingress_{imu_ingress_length};
  LibXR::LockFreeQueue<QueuedAccl> accl_ingress_{imu_ingress_length};
  LibXR::LockFreeQueue<QueuedQuat> quat_ingress_{imu_ingress_length};
  std::atomic<bool> overflowed_{false};

  DropOldestQueue<QueuedGyro> pending_gyros_{pending_limit};
  DropOldestQueue<QueuedAccl> pending_accls_{pending_limit};
  DropOldestQueue<QueuedQuat> pending_quats_{pending_limit};
  DropOldestQueue<ImageSample> image_events_{image_event_limit};
  SampleHistory<AssembledImu, history_limit> imu_history_{};

  LibXR::Mutex sync_state_mutex_{};
  SyncMode sync_mode_{SyncMode::RAW_PROBE};
  int32_t offset_us_{0};
  uint32_t sync_probe_div_{3};
  uint32_t sync_active_level_{1};
  uint32_t next_sync_seq_{1};
  // 回执回调不入队，只接受当前 probe 的一次反馈。
  std::atomic<uint32_t> active_probe_seq_{0};
  std::atomic<uint32_t> probe_ack_seq_{0};
  std::atomic<uint64_t> probe_ack_timestamp_us_{0};

  SyncState state_{SyncState::OBSERVING};
  CameraFrameSyncCore::CadenceState image_cadence_{};
  CameraFrameSyncCore::CadenceState imu_cadence_{};
  SyncRelation relation_{};
  bool last_image_valid_{false};
  uint64_t last_image_timestamp_us_{0};
  PendingImage pending_image_{};
  uint32_t pending_probe_seq_{0};
  bool pending_probe_ack_valid_{false};
  uint64_t pending_probe_imu_timestamp_us_{0};
};

#include "camera_frame_sync_impl.hpp"
#include "camera_frame_sync_state_machine.hpp"
