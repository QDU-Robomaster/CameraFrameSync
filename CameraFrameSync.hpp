#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机帧与 MCU 触发时间戳同步模块：持有相机帧并与 IMU 触发时间配对，发布 SyncedFrame / Camera frame and MCU trigger timestamp synchronization Module that holds camera frames, pairs them with IMU trigger times and publishes SyncedFrame
depends:
- id: QDU-Robomaster/CameraBase
  ref: same-or-dev
- id: QDU-Robomaster/CameraSync
  ref: same-or-dev
- id: xrobot-org/DurationStatistics
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "CameraBase.hpp"
#include "CameraFrameSyncCore.hpp"
#include "CameraFrameSyncFrameQueue.hpp"
#include "CameraSync.hpp"
#include "DurationStatistics.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "logger.hpp"
#include "transform.hpp"

/**
 * @brief 同步模式。
 *        Synchronization mode.
 */
enum class CameraFrameSyncMode : uint8_t
{
  TRIGGER = 0,          ///< 与 MCU 真实触发边沿配对
                        ///< Pair images with real MCU trigger edges
  RAW_PROBE = TRIGGER,  ///< TRIGGER 的别名
                        ///< Alias of TRIGGER
  LATEST_IMU = 1,       ///< 每张图像取最新姿态四元数，数据源已自行同步
                        ///< Take the latest attitude quaternion per image; the data
                        ///< source is already synchronized
};

/**
 * @brief 原始 IMU 的坐标约定。
 *        Coordinate convention of the raw IMU.
 */
enum class CameraFrameSyncRawImuFrame : uint8_t
{
  BODY_X_RIGHT_Y_FORWARD_Z_UP = 0,  ///< 本体系 x 右 / y 前 / z 上，数据原样使用
                                    ///< Body frame x right / y forward / z up, used as is
  X_FORWARD_Y_LEFT_Z_UP_TO_BODY = 1,  ///< 转换：x 前 / y 左 / z 上
                                      ///< Converted: x forward / y left / z up
};

/**
 * @brief 相机帧同步模块：持有相机帧，与 MCU 触发事件配对并发布 SyncedFrame。
 *        Camera frame synchronization Module that holds camera frames, pairs them with
 *        MCU trigger events and publishes SyncedFrame.
 *
 * 所有回调共用一把状态互斥锁，Topic 发布在锁外执行，同步订阅者可以重入模块接口。
 * All callbacks share one state mutex. Topic publication runs outside the mutex, so
 * synchronous subscribers may re-enter the Module interface.
 *
 * @tparam FrameLayoutV 帧布局，与相机实例相同。
 *                      Frame layout, identical to that of the camera instance.
 */
template <CameraTypes::FrameLayout FrameLayoutV>
class CameraFrameSync
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

  /**
   * @brief 同步后的图像帧。
   *        Synchronized image frame.
   */
  struct SyncedFrame
  {
    uint64_t sequence{};  ///< 单调递增的输出序号
                          ///< Monotonically increasing output sequence number
    SharedFrame image{};  ///< 图像句柄
                          ///< Image handle
    ImuStamped imu{};     ///< 配对的 IMU 数据，时间戳为权威时间
                          ///< Paired IMU data whose timestamp is the authoritative time

    /**
     * @brief 获取只读图像帧。
     *        Get the read-only image frame.
     *
     * @return 图像帧指针，句柄无效时为空指针。
     *         Image frame pointer; null when the handle is invalid.
     */
    [[nodiscard]] const ImageFrame* GetImageFrame() const noexcept { return image.Get(); }

    /**
     * @brief 判断图像句柄是否有效。
     *        Check whether the image handle is valid.
     *
     * @return 句柄有效为 true。
     *         True when the handle is valid.
     */
    [[nodiscard]] bool Valid() const noexcept { return image.Valid(); }
  };

  /**
   * @brief SyncedFrame Topic 载荷，指针在同步回调期间有效。
   *        SyncedFrame Topic payload; the pointer is valid during synchronous callbacks.
   */
  using SyncedFrameTopicPayload = const SyncedFrame*;

  /**
   * @brief 运行参数。
   *        Runtime parameters.
   */
  struct RuntimeParam
  {
    SyncMode mode = SyncMode::TRIGGER;  ///< 同步模式
                                        ///< Synchronization mode
    int32_t offset_us = 0;  ///< IMU 时间域中相对触发时间的取样偏移 (us)，作用于 TRIGGER
                            ///< Sampling offset relative to the trigger time in the IMU
                            ///< time domain (us); applies to TRIGGER
    std::string_view host_topic_domain_name = "shared_memory";
    ///< 命令、事件和原始 IMU Topic 的 domain 名称
    ///< Domain name of the command, event and raw IMU Topics
    std::string_view sync_command_topic_name = "camera_sync_command";
    ///< 发给 CameraSync 的命令 Topic 名称
    ///< Name of the command Topic sent to CameraSync
    std::string_view sync_result_topic_name = "camera_sync_result";
    ///< CameraSync 回执与触发事件 Topic 名称
    ///< Name of the CameraSync acknowledgement and trigger event Topic
    uint32_t sync_active_level = 1;  ///< 触发有效电平，非零按 1 处理
                                     ///< Trigger active level; non-zero is treated as 1
    uint64_t camera_settle_us = 10000U;  ///< STOP ACK 后等待相机稳定的时间 (us)
                                         ///< Time to wait for the camera to settle after
                                         ///< the STOP ACK (us)
    RawImuFrame raw_imu_frame = RawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP;
    ///< 原始 IMU 坐标约定
    ///< Raw IMU coordinate convention
    std::string_view raw_quat_topic_name = {};
    ///< 非空时替代 <相机名>_quat 的姿态 Topic 名称
    ///< Attitude Topic name replacing <camera_name>_quat when non-empty
    std::string_view synced_frame_topic_name = {};
    ///< 同步结果 Topic 名称，为空时使用 <图像 Topic>_synced
    ///< Synchronized result Topic name; <image Topic>_synced when empty

    /**
     * @brief 使用全部默认值构造。
     *        Construct with all defaults.
     */
    RuntimeParam() = default;

    /**
     * @brief 逐项构造运行参数。
     *        Construct runtime parameters field by field.
     *
     * @param mode 同步模式。
     *             Synchronization mode.
     * @param offset_us IMU 时间域取样偏移 (us)。
     *                  Sampling offset in the IMU time domain (us).
     * @param host_topic_domain_name 命令、事件和原始 IMU Topic 的 domain 名称。
     *                               Domain name of the command, event and raw IMU Topics.
     * @param sync_command_topic_name 命令 Topic 名称。
     *                                Command Topic name.
     * @param sync_result_topic_name 回执与触发事件 Topic 名称。
     *                               Acknowledgement and trigger event Topic name.
     * @param sync_active_level 触发有效电平。
     *                          Trigger active level.
     * @param camera_settle_us STOP ACK 后等待相机稳定的时间 (us)。
     *                         Time to wait for the camera to settle after the STOP ACK
     *                         (us).
     * @param raw_imu_frame 原始 IMU 坐标约定。
     *                      Raw IMU coordinate convention.
     * @param raw_quat_topic_name 姿态 Topic 名称，为空时使用 <相机名>_quat。
     *                            Attitude Topic name; <camera_name>_quat when empty.
     * @param synced_frame_topic_name 同步结果 Topic 名称，为空时用 <图像 Topic>_synced。
     *                                Synced result Topic name; <image Topic>_synced when
     *                                empty.
     */
    constexpr RuntimeParam(
        SyncMode mode, int32_t offset_us, std::string_view host_topic_domain_name,
        std::string_view sync_command_topic_name, std::string_view sync_result_topic_name,
        uint32_t sync_active_level, uint64_t camera_settle_us = 10000U,
        RawImuFrame raw_imu_frame = RawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP,
        std::string_view raw_quat_topic_name = {},
        std::string_view synced_frame_topic_name = {})
        : mode(mode),
          offset_us(offset_us),
          host_topic_domain_name(host_topic_domain_name),
          sync_command_topic_name(sync_command_topic_name),
          sync_result_topic_name(sync_result_topic_name),
          sync_active_level(sync_active_level),
          camera_settle_us(camera_settle_us),
          raw_imu_frame(raw_imu_frame),
          raw_quat_topic_name(raw_quat_topic_name),
          synced_frame_topic_name(synced_frame_topic_name)
    {
    }
  };

  /**
   * @brief 返回全部默认值的运行参数。
   *        Return runtime parameters with all defaults.
   *
   * @return 默认运行参数。
   *         Default runtime parameters.
   */
  static RuntimeParam DefaultRuntime() { return {}; }

  /**
   * @brief 构造 CameraFrameSync，订阅图像、原始 IMU 与同步事件 Topic。
   *        TRIGGER 模式下立即对当前 profile 发起一次 STOP / settle / START。
   *        Construct CameraFrameSync and subscribe to the image, raw IMU and
   *        synchronization event Topics. In TRIGGER mode it immediately starts one
   *        STOP / settle / START sequence on the current profile.
   *
   * 相机名、图像 Topic、domain 与两个同步 Topic 名为非空值，相机 profile 数量为 1 到 2
   * 且周期非零、几何合法、ID 唯一；否则 REQUIRE 失败。
   * The camera name, image Topic, domain and both synchronization Topic names are
   * non-empty, and the camera has one or two profiles with non-zero periods, valid
   * geometry and unique IDs; otherwise REQUIRE fails.
   *
   * @param camera 相机实例，其帧布局为 FrameLayoutV。
   *               Camera instance whose frame layout is FrameLayoutV.
   * @param runtime 运行参数。
   *                Runtime parameters.
   */
  CameraFrameSync(Base& camera, RuntimeParam runtime = DefaultRuntime());

  /**
   * @brief 获取同步结果 Topic 名称。
   *        Get the name of the synchronized result Topic.
   *
   * @return Topic 名称。
   *         Topic name.
   */
  [[nodiscard]] const char* SyncedFrameTopicName() const;

  /**
   * @brief 获取命令、事件和原始 IMU Topic 的 domain 名称。
   *        Get the domain name of the command, event and raw IMU Topics.
   *
   * @return domain 名称。
   *         Domain name.
   */
  [[nodiscard]] const char* RawTopicDomainName() const;

  /**
   * @brief 获取构造时复制的相机原生标定。
   *        Get the native camera calibration copied at construction.
   *
   * @return 相机标定。
   *         Camera calibration.
   */
  [[nodiscard]] const CameraCalibration& Calibration() const noexcept
  {
    return calibration_;
  }

  /**
   * @brief 获取相机声明的固定 profile。
   *        Get the fixed profiles declared by the camera.
   *
   * @return profile 列表。
   *         Profile list.
   */
  [[nodiscard]] std::span<const CameraProfile> Profiles() const noexcept;

  /**
   * @brief 获取当前生效的 profile。
   *        Get the profile currently in effect.
   *
   * @return profile ID。
   *         Profile ID.
   */
  [[nodiscard]] ProfileId ActiveProfile() const;

  /**
   * @brief 获取同步模式。
   *        Get the synchronization mode.
   *
   * @return 同步模式。
   *         Synchronization mode.
   */
  [[nodiscard]] SyncMode GetSyncMode() const;

  /**
   * @brief 请求切换到另一个固定 profile，仅接纳异步的 STOP / settle /
   *        SwitchProfile / START 事务。
   *        Request a different fixed profile; the call admits the asynchronous
   *        STOP / settle / SwitchProfile / START transaction.
   *
   * TRIGGER 模式仅在 RUNNING 状态接纳请求，进入 FAILED 之后对当前 profile 的请求同样
   * 返回 STATE_ERR。LATEST_IMU 模式仅当前 profile 可用。
   * TRIGGER mode admits requests only while RUNNING; after FAILED, a request for the
   * current profile also returns STATE_ERR. In LATEST_IMU mode only the current profile
   * is available.
   *
   * @param profile 目标 profile ID。
   *                Target profile ID.
   * @return OK 表示已接纳或已处于该 profile，NOT_SUPPORT 表示相机未声明该 profile
   *         （LATEST_IMU 模式下为非当前 profile），STATE_ERR 表示当前状态不接纳请求。
   *         OK when admitted or already on that profile; NOT_SUPPORT when the camera
   *         does not declare the profile (a non-current profile in LATEST_IMU mode);
   *         STATE_ERR when the current state does not admit requests.
   */
  LibXR::ErrorCode RequestProfile(ProfileId profile);

  /**
   * @brief 设置 IMU 取样偏移，选择触发时间附近的 IMU 样本，
   *        SyncedFrame::imu.timestamp_us 保留原始触发时间。
   *        Set the IMU sampling offset that selects a nearby IMU sample; the
   *        authoritative trigger timestamp carried by SyncedFrame::imu.timestamp_us is
   *        unchanged.
   *
   * @param offset_us IMU 时间域取样偏移 (us)。
   *                  Sampling offset in the IMU time domain (us).
   */
  void SetOffsetUs(int32_t offset_us);

  /**
   * @brief 立即处理待匹配数据，然后清空图像与 trigger 队列。
   *        Process pending data immediately, then clear the image and trigger queues.
   */
  void FlushPendingFrames();

  /**
   * @brief 监控回调，输出控制状态、profile、周期、计数与 pending_processing 耗时统计。
   *        Monitor callback that logs the control state, profile, period, counters and
   *        the pending_processing duration statistics.
   */
  void OnMonitor();

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
    bool consumes_retry{false};
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
  void ResetTimestampEpochLocked(uint64_t gyro_timestamp_us);
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
  bool camera_switch_in_progress_{false};
  bool epoch_recovery_{false};
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
  XRobot::DurationStatistics pending_processing_duration_{};
};

#include "CameraFrameSyncImpl.hpp"
#include "CameraFrameSyncStateMachine.hpp"
