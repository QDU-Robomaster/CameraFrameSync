#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机共享图像桥与原始 IMU 同步器
constructor_args:
  camera: '@camera'
template_args:
  - Info:
      width: 1280
      height: 720
      step: 3840
      encoding: CameraTypes::Encoding::BGR8
      camera_matrix: [800.0, 0.0, 640.0, 0.0, 800.0, 360.0, 0.0, 0.0, 1.0]
      distortion_model: CameraTypes::DistortionModel::PLUMB_BOB
      distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
      rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      projection_matrix: [800.0, 0.0, 640.0, 0.0, 0.0, 800.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
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
#include <string>
#include <string_view>
#include <utility>

#include "CameraBase.hpp"
#include "CameraSync.hpp"
#include "app_framework.hpp"
#include "camera_frame_sync_core.hpp"
#include "camera_frame_sync_recording.hpp"
#include "camera_frame_sync_subscriber.hpp"
#include "libxr.hpp"
#include "linux_shared_topic.hpp"
#include "logger.hpp"
#include "transform.hpp"

/**
 * @brief 图像与 IMU 的同步模式。
 */
enum class CameraFrameSyncMode : uint8_t
{
  RAW_PROBE = 0,   ///< 通过 CameraSync command/result 显式锁定。
  LATEST_IMU = 1,  ///< 数据源已同步时，图像直接绑定最新完整 IMU。
};

/**
 * @brief 图像共享发布与 IMU 同步模块。
 *
 * 原始 gyro/accl/quat 回调只入队；同步回执只记录当前 probe 的命中结果。
 * 完整同步状态机仍然在图像提交时串行推进。
 * RAW_PROBE 模式使用 MCU 侧 CameraSync 的回执 timestamp 锁定 IMU 时间轴。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CameraFrameSync : public LibXR::Application
{
 public:
  using Self = CameraFrameSync<CameraInfoV>;        ///< 当前模板实例类型。
  using Base = CameraBase<CameraInfoV>;             ///< 上游相机基类类型。
  using CameraInfo = typename Base::CameraInfo;     ///< 相机静态标定信息类型。
  using ImageFrame = typename Base::ImageFrame;     ///< 共享图像帧类型。
  using ImuStamped = typename Base::ImuStamped;     ///< 同步后 IMU 输出类型。
  using ImuVector = std::array<float, 3>;           ///< Host 侧三轴平铺 ABI。
  using QuatSample = std::array<float, 4>;          ///< Host 侧 wxyz 四元数 ABI。
  using RawImuVector = Eigen::Matrix<float, 3, 1>;  ///< MCU 侧三轴 topic ABI。
  using RawQuatSample = LibXR::Quaternion<float>;   ///< MCU 侧四元数 topic ABI。
  using ImageTopic = LibXR::LinuxSharedTopic<ImageFrame>;  ///< 图像共享 topic 类型。
  using ImageData = typename ImageTopic::Data;             ///< 图像槽位租约类型。
  using ImageCommitCallback =
      typename Base::ImageCommitCallback;  ///< CameraBase 图像提交回调类型。
  using SyncedFrame = CameraFrameSyncSyncedFrame<ImageTopic, ImuStamped>;
  using Subscriber = CameraFrameSyncSubscriber<ImageTopic, ImuStamped>;

  /**
   * @brief 当前模块实例使用的相机静态信息。
   */
  static inline constexpr CameraInfo camera_info = CameraInfoV;

  /**
   * @brief 共享图像 topic 的默认槽位配置。
   */
  static constexpr LibXR::LinuxSharedTopicConfig image_topic_config{
      .slot_num = 8,
      .subscriber_num = 8,
      .queue_num = 2,
  };

  using SyncMode = CameraFrameSyncMode;  ///< 图像与 IMU 的同步模式。

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
    uint32_t sync_probe_div = 3;         ///< CameraSync 探针分频倍率。
    uint32_t sync_active_level = 1;      ///< 同步触发输出有效电平。
    float target_trigger_hz = 50.0F;     ///< 同步完成后的目标相机触发频率。
    bool record_enable = false;          ///< 是否记录同步映射和 IMU 数据。
    std::string_view record_dir = {};    ///< 为空时自动创建 runs/camera_sync/...。
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

  ~CameraFrameSync() { recording_.Close(); }

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

  /**
   * @brief 输出上一监控周期内的相机、IMU 和同步帧统计。
   */
  void OnMonitor() override;

 private:
  using SyncState = CameraFrameSyncCore::SyncState;
  template <typename T, size_t Capacity>
  using SampleHistory = CameraFrameSyncCore::SampleHistory<T, Capacity>;

  /**
   * @brief 状态机侧使用的定长 FIFO，满时丢弃最旧样本。
   *
   * 回调入口仍然直接写 LibXR::LockFreeQueue；这个包装只用于图像提交路径内的
   * 待处理队列，保证消费不及时不会让后续同步永远卡在旧数据上。
   */
  template <typename T>
  class DropOldestQueue
  {
   public:
    /**
     * @brief 创建固定容量队列。
     */
    explicit DropOldestQueue(size_t capacity) : queue_(capacity) {}

    /**
     * @brief 队列当前是否为空。
     */
    bool Empty() const { return queue_.Size() == 0; }

    /**
     * @brief 读取最旧元素但不出队。
     */
    bool Front(T& out) { return queue_.Peek(out) == LibXR::ErrorCode::OK; }

    /**
     * @brief 丢弃最旧元素。
     */
    void PopFront() { queue_.Pop(); }

    /**
     * @brief 清空队列。
     */
    void Clear() { queue_.Reset(); }

    /**
     * @brief 追加元素；容量不足时先弹出最旧元素。
     *
     * @return true 表示本次追加丢弃了旧样本。
     */
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

  /**
   * @brief 模块持有的 topic 名称和句柄。
   *
   * 字符串必须由模块持有，保证 Topic/Domain 内部保存的 c_str() 生命周期覆盖模块
   * 全生命周期。
   */
  struct Topics
  {
    Topics(const Base& camera, const RuntimeParam& runtime)
        : image_name(camera.ImageTopicNameView()),
          imu_name(camera.ImuTopicNameView()),
          host_domain_name(runtime.host_topic_domain_name),
          sync_command_name(runtime.sync_command_topic_name),
          sync_result_name(runtime.sync_result_topic_name),
          raw_imu_prefix(camera.NameView()),
          gyro_name(raw_imu_prefix + "_gyro"),
          accl_name(raw_imu_prefix + "_accl"),
          quat_name(raw_imu_prefix + "_quat"),
          host_domain(host_domain_name.c_str()),
          image(image_name.c_str(), image_topic_config),
          synced_imu(LibXR::Topic::FindOrCreate<ImuStamped>(
              imu_name.c_str(), &host_domain)),
          sync_command(LibXR::Topic::FindOrCreate<CameraSync::SyncCommand>(
              sync_command_name.c_str(), &host_domain)),
          sync_result(LibXR::Topic::FindOrCreate<CameraSync::SyncEvent>(
              sync_result_name.c_str(), &host_domain)),
          gyro(LibXR::Topic::FindOrCreate<RawImuVector>(gyro_name.c_str(),
                                                        &host_domain)),
          accl(LibXR::Topic::FindOrCreate<RawImuVector>(accl_name.c_str(),
                                                        &host_domain)),
          quat(LibXR::Topic::FindOrCreate<RawQuatSample>(quat_name.c_str(),
                                                         &host_domain))
    {
      ASSERT(!raw_imu_prefix.empty());
    }

    std::string image_name;
    std::string imu_name;
    std::string host_domain_name;
    std::string sync_command_name;
    std::string sync_result_name;
    std::string raw_imu_prefix;
    std::string gyro_name;
    std::string accl_name;
    std::string quat_name;
    LibXR::Topic::Domain host_domain;
    ImageTopic image;
    LibXR::Topic synced_imu;
    LibXR::Topic sync_command;
    LibXR::Topic sync_result;
    LibXR::Topic gyro;
    LibXR::Topic accl;
    LibXR::Topic quat;
  };

  /**
   * @brief topic 回调对象集合，回调只负责把输入塞进模块内部队列/邮箱。
   */
  struct TopicCallbacks
  {
    explicit TopicCallbacks(Self* self)
        : gyro(LibXR::Topic::Callback::Create(OnGyroStatic, self)),
          accl(LibXR::Topic::Callback::Create(OnAcclStatic, self)),
          quat(LibXR::Topic::Callback::Create(OnQuatStatic, self)),
          sync_result(LibXR::Topic::Callback::Create(OnSyncResultStatic, self))
    {
    }

    LibXR::Topic::Callback gyro;
    LibXR::Topic::Callback accl;
    LibXR::Topic::Callback quat;
    LibXR::Topic::Callback sync_result;
  };

  /**
   * @brief gyro 原始样本，timestamp 为 Topic envelope 中的传感器时间。
   */
  struct GyroSample
  {
    uint64_t sensor_timestamp_us{};
    ImuVector angular_velocity_xyz{};
  };

  /**
   * @brief accl 原始样本，timestamp 必须与同一 IMU 帧的 gyro 对齐。
   */
  struct AcclSample
  {
    uint64_t sensor_timestamp_us{};
    ImuVector linear_acceleration_xyz{};
  };

  /**
   * @brief quat 原始样本，内部顺序统一规整为 wxyz。
   */
  struct QuatReading
  {
    uint64_t sensor_timestamp_us{};
    QuatSample rotation_wxyz{};
  };

  /**
   * @brief 已提交图像的传感器时间戳。
   *
   * 大图像数据本身已经交给 LinuxSharedTopic，这里只保留同步需要的时间基线。
   */
  struct ImageSample
  {
    uint64_t sensor_timestamp_us{};
  };

  /**
   * @brief 由同一 timestamp 的 gyro/accl/quat 组装出的完整 IMU 帧。
   */
  struct AssembledImu
  {
    uint64_t sensor_timestamp_us{};
    QuatSample rotation_wxyz{};
    ImuVector angular_velocity_xyz{};
    ImuVector linear_acceleration_xyz{};
  };

  /**
   * @brief 已锁定同步基准、但还在等待 offset 后最终 IMU 的挂起匹配。
   */
  struct PendingMatch
  {
    bool valid{false};
    uint64_t imu_timestamp_us{0};
    uint64_t period_us{0};
  };

  /**
   * @brief 状态机当前持有的一帧图像时间戳。
   *
   * 真实图像已经发布到共享槽位；这里不能换下一张图像重试，否则会破坏
   * 图像时间戳、同步基准 IMU 和最终 offset IMU 的对应关系。
   */
  struct PendingFrame
  {
    bool valid{false};
    ImageSample image{};
    bool cadence_consumed{false};
    PendingMatch match{};
  };

  /**
   * @brief 本帧图像锁定的同步基准 IMU。
   */
  struct SyncMatch
  {
    const AssembledImu* imu{nullptr};
    uint64_t period_us{0};
  };

  /**
   * @brief 已观察到的正常发布周期。
   *
   * 这些周期只用于识别探针 gap 和在 IMU 时间轴上递推，不用于比较相机与
   * IMU 时间戳的绝对值。
   */
  struct ObservedPeriods
  {
    uint64_t image_us{0};
    uint64_t imu_us{0};
  };

  /**
   * @brief RAW_PROBE 锁定后的 IMU 时间轴关系。
   */
  struct LockedSync
  {
    uint64_t period_us{0};
    uint64_t last_imu_timestamp_us{0};
  };

  /**
   * @brief 单帧图像在状态机里的处理结果。
   */
  enum class ImageDecision : uint8_t
  {
    WAIT = 0,   ///< 保留当前帧，等待未来 IMU 或 CameraSync 回执。
    DONE = 1,   ///< 当前帧已经完成处理，可以切到下一帧。
    RESET = 2,  ///< 当前帧打破同步关系，丢帧后重新观察。
  };

  static constexpr size_t imu_ingress_length = 1024;   ///< topic 回调入口队列长度。
  static constexpr size_t pending_limit = 1024;        ///< 状态机待处理队列长度。
  static constexpr size_t image_event_limit = 64;      ///< 图像时间戳待处理队列长度。
  static constexpr size_t history_limit = 1024;        ///< 可供 offset 查找的 IMU 历史长度。
  static constexpr uint32_t cadence_stable_gaps = 2;   ///< 判定周期稳定所需连续 gap 数。
  static constexpr uint32_t max_synced_image_gap_stride = 8;  ///< 同步态可接受的连续丢图数量。
  static constexpr uint32_t max_raw_imu_gap_stride = 8;  ///< 稳定后可接受的连续 raw IMU 缺样数量。
  static constexpr uint64_t imu_cadence_tolerance_us = 300ULL;     ///< IMU 周期容差。
  static constexpr uint64_t image_cadence_tolerance_us = 1500ULL;  ///< 图像周期容差。
  static constexpr uint64_t raw_imu_epoch_reset_backward_us =
      100000ULL;  ///< raw IMU 时间戳大幅回退时判定设备流重启。
  static constexpr uint64_t probe_ack_timeout_min_us =
      100000ULL;  ///< probe 发出后等待回执的最短超时时间。

  /**
   * @brief 返回同步模式的日志名称。
   */
  static const char* SyncModeName(SyncMode mode);

  /**
   * @brief 返回 RAW_PROBE 状态机状态的日志名称。
   */
  static const char* StateName(SyncState state);

  /**
   * @brief 将 MCU 侧 IMU 向量 payload 规整为 Host 侧平铺数组。
   */
  static ImuVector ToImuVector(const RawImuVector& data);

  /**
   * @brief 将 MCU 侧四元数 payload 规整为 Host 侧 wxyz 顺序。
   */
  static QuatSample ToQuatSample(const RawQuatSample& data);

  /**
   * @brief 启动时从共享图像 topic 租用第一块可写图像槽位。
   */
  bool AcquireInitialWritableImage();

  /**
   * @brief CameraBase 图像提交回调适配层。
   */
  static void CommitImageAdapter(bool, Self* self, ImageFrame*& next_image);

  /**
   * @brief 发布当前图像槽位，并把下一块可写槽位交还给 CameraBase。
   */
  ImageFrame* CommitImageAndLeaseNext();

  /**
   * @brief gyro topic 回调；只做规整和入队，不推进同步状态机。
   */
  static void OnGyroStatic(bool, Self* self, LibXR::MicrosecondTimestamp timestamp,
                           const RawImuVector& data);

  /**
   * @brief accl topic 回调；只做规整和入队，不推进同步状态机。
   */
  static void OnAcclStatic(bool, Self* self, LibXR::MicrosecondTimestamp timestamp,
                           const RawImuVector& data);

  /**
   * @brief quat topic 回调；只做规整和入队，不推进同步状态机。
   */
  static void OnQuatStatic(bool, Self* self, LibXR::MicrosecondTimestamp timestamp,
                           const RawQuatSample& data);

  /**
   * @brief CameraSync 回执回调；只记录当前 active probe 的一次命中。
   */
  static void OnSyncResultStatic(bool, Self* self,
                                 LibXR::MicrosecondTimestamp timestamp,
                                 const CameraSync::SyncEvent& event);

  /**
   * @brief 将图像周期观察切到同步完成后的目标运行周期。
   */
  void LockImageCadence(uint64_t image_timestamp_us, uint64_t image_period_us);

  /**
   * @brief 图像成功发布后的同步处理入口。
   */
  void ProcessCommittedImage(uint64_t image_timestamp_us);

  /**
   * @brief 没有新图像可发布时，也推进已到达的 IMU / 回执数据。
   */
  void ProcessSyncWorkWithoutImage();

  /**
   * @brief 图像共享 topic 发布失败后，只推进同步时间基线。
   */
  void ProcessDroppedImage(uint64_t image_timestamp_us);

  /**
   * @brief 将回调入口队列的数据搬到状态机私有 pending 队列。
   */
  void CollectIncomingTopics();

  /**
   * @brief 尽可能把 pending gyro/accl/quat 组装为完整 IMU 历史。
   */
  void AssembleImuHistory();

  /**
   * @brief 以 gyro timestamp 为主键尝试组装一帧完整 IMU。
   */
  bool TryAssembleOneImu();

  /**
   * @brief 将一帧完整 IMU 写入历史并更新周期观察。
   */
  void AcceptAssembledImu(const AssembledImu& imu);

  /**
   * @brief raw IMU 时间轴重启后清空旧历史，并接受新 epoch 第一帧。
   */
  void ResetRawImuEpoch(uint64_t previous_timestamp_us,
                        const AssembledImu& first_imu);

  /**
   * @brief 更新 IMU 发布周期观察，并在周期破坏时触发重同步。
   */
  void ObserveImuCadence(uint64_t sensor_timestamp_us);

  /**
   * @brief 按图像时间顺序推进同步状态机。
   */
  void ProcessImageEvents();

  /**
   * @brief LATEST_IMU 模式下处理一帧图像。
   */
  ImageDecision ProcessLatestImage(PendingFrame& frame);

  /**
   * @brief RAW_PROBE 模式下处理一帧图像。
   */
  ImageDecision ProcessRawProbeImage(PendingFrame& frame);

  /**
   * @brief 观察正常图像发布周期；probe gap 不应调用此路径。
   */
  CameraFrameSyncCore::CadenceUpdate ObserveNormalImageCadence(uint64_t image_ts);

  /**
   * @brief 图像未发布给下游时，按丢帧维护 RAW_PROBE 时间基线。
   */
  void ObserveDroppedImage(uint64_t image_ts);

  /**
   * @brief 周期稳定后向 MCU 下发一次 CameraSync 探针命令。
   */
  void MaybeStartProbe();

  /**
   * @brief 处理探针图像，等待对应 CameraSync 回执和 IMU 历史到达。
   */
  ImageDecision TryProbeImage(PendingFrame& frame);

  /**
   * @brief 估算当前 probe 最长等待时间。
   */
  uint64_t ProbeTimeoutUs() const;

  /**
   * @brief 判断当前 probe 是否已经超时。
   */
  bool ProbeTimedOut() const;

  /**
   * @brief LATEST_IMU 模式下用最新完整 IMU 建立本帧匹配。
   */
  ImageDecision TryLatestImuMatch(PendingFrame& frame);

  /**
   * @brief RAW_PROBE 已锁定后，沿 IMU 时间轴递推下一帧同步基准。
   */
  ImageDecision TrySyncedImage(PendingFrame& frame, uint32_t image_gap_stride);

  /**
   * @brief 继续处理之前已锁定同步基准、但等待 offset IMU 的图像。
   */
  ImageDecision ResumePendingMatch(PendingFrame& frame);

  /**
   * @brief 尝试发布匹配结果；若 offset 后 IMU 未到达则把匹配保存在 frame。
   */
  ImageDecision PublishOrRememberMatch(PendingFrame& frame,
                                       const SyncMatch& match);

  /**
   * @brief 根据同步基准 IMU 和 offset 选择最终 IMU 并发布。
   */
  ImageDecision PublishMatchedImage(const ImageSample& image, const SyncMatch& match);

  /**
   * @brief 发布 timestamp 与图像一致的同步 IMU topic。
   */
  void PublishSyncedImu(uint64_t image_timestamp_us, const AssembledImu& imu);

  /**
   * @brief 估计相邻图像在 IMU 时间轴上跨过的采样周期。
   */
  uint64_t EstimatedSyncPeriodUs() const;

  /**
   * @brief 根据 IMU 周期和目标频率计算 MCU 正常运行分频。
   */
  uint8_t TargetRunTriggerDiv() const;

  /**
   * @brief 判断图像间隔是否仍符合正常发布周期。
   */
  bool IsNormalImageGap(uint64_t image_gap_us) const;

  /**
   * @brief 判断图像间隔是否为正常周期的整数倍。
   */
  uint32_t MatchImageGapStride(uint64_t image_gap_us) const;

  /**
   * @brief 判断图像间隔是否符合 CameraSync 探针制造的异常周期。
   */
  bool IsProbeImageGap(uint64_t image_gap_us) const;

  /**
   * @brief 判断 IMU 历史是否已经覆盖到目标时间戳。
   */
  bool ImuHistoryReached(uint64_t target_timestamp_us) const;

  /**
   * @brief 记录最近一帧已被状态机消费的图像时间戳。
   */
  void RememberImage(uint64_t image_timestamp_us);

  /**
   * @brief 清空图像周期观察，不清空 IMU 历史。
   */
  void ResetImageObservation();

  /**
   * @brief 清空当前 CameraSync 探针邮箱和待处理回执。
   */
  void ClearPendingProbe();

  /**
   * @brief 丢弃当前挂起图像。
   */
  void ClearPendingFrame();

  /**
   * @brief 解除 RAW_PROBE 锁定关系并回到周期观察状态。
   */
  void ResetLock(const char* reason, const char* detail);

  /**
   * @brief 清空全部运行时队列、历史和状态。
   */
  void ResetRuntimeState();

  /**
   * @brief 入口队列溢出后执行保守恢复。
   */
  void HandleOverflowRecovery();

 private:
  /**
   * @brief topic 句柄与回调对象必须先于运行时状态存在。
   */
  Topics topics_;
  TopicCallbacks callbacks_;

  /**
   * @brief 当前交给 CameraBase 写入的共享图像槽位。
   */
  ImageData current_image_{};

  /**
   * @brief topic 回调写入的无锁入口队列。
   */
  LibXR::LockFreeQueue<GyroSample> gyro_ingress_{imu_ingress_length};
  LibXR::LockFreeQueue<AcclSample> accl_ingress_{imu_ingress_length};
  LibXR::LockFreeQueue<QuatReading> quat_ingress_{imu_ingress_length};

  /**
   * @brief 任一路队列溢出都会置位，随后在图像提交路径统一重置。
   */
  std::atomic<bool> overflowed_{false};

  /**
   * @brief 监控周期计数器。
   */
  std::atomic<uint64_t> monitor_raw_gyro_count_{0};
  std::atomic<uint64_t> monitor_raw_accl_count_{0};
  std::atomic<uint64_t> monitor_raw_quat_count_{0};
  std::atomic<uint64_t> monitor_image_input_count_{0};
  std::atomic<uint64_t> monitor_image_publish_ok_count_{0};
  std::atomic<uint64_t> monitor_image_drop_count_{0};
  std::atomic<uint64_t> monitor_assembled_imu_count_{0};
  std::atomic<uint64_t> monitor_synced_output_count_{0};
  std::atomic<uint64_t> monitor_reset_count_{0};
  std::atomic<uint64_t> monitor_overflow_count_{0};

  /**
   * @brief 状态机私有 pending 队列和短 IMU 历史。
   */
  DropOldestQueue<GyroSample> pending_gyros_{pending_limit};
  DropOldestQueue<AcclSample> pending_accls_{pending_limit};
  DropOldestQueue<QuatReading> pending_quats_{pending_limit};
  DropOldestQueue<ImageSample> image_events_{image_event_limit};
  SampleHistory<AssembledImu, history_limit> imu_history_{};

  /**
   * @brief 保护图像提交路径、参数切换和 pending 队列消费。
   */
  LibXR::Mutex sync_state_mutex_{};

  /**
   * @brief 运行时配置副本。
   */
  SyncMode sync_mode_{SyncMode::RAW_PROBE};
  int32_t offset_us_{0};
  uint32_t sync_probe_div_{3};
  uint32_t sync_active_level_{1};
  float target_trigger_hz_{50.0F};
  uint8_t next_sync_seq_{1};

  /**
   * @brief CameraSync 回执邮箱；回调只接受 active probe 的一次反馈。
   */
  std::atomic<uint32_t> active_probe_seq_{0};
  std::atomic<uint32_t> probe_ack_seq_{0};
  std::atomic<uint32_t> probe_ack_run_div_{0};
  std::atomic<uint64_t> probe_ack_timestamp_us_{0};

  /**
   * @brief RAW_PROBE 状态机当前锁定关系与周期观察结果。
   */
  SyncState state_{SyncState::OBSERVING};
  CameraFrameSyncCore::CadenceState image_cadence_{};
  CameraFrameSyncCore::CadenceState imu_cadence_{};
  ObservedPeriods periods_{};
  LockedSync locked_sync_{};

  /**
   * @brief 最近一帧被同步状态机接受的图像时间戳。
   */
  bool last_image_valid_{false};
  uint64_t last_image_timestamp_us_{0};

  /**
   * @brief 当前因等待 IMU / 回执而挂起的图像。
   */
  PendingFrame pending_frame_{};

  /**
   * @brief 当前探针命令的本地状态。
   */
  uint32_t pending_probe_seq_{0};
  bool pending_probe_ack_valid_{false};
  uint64_t pending_probe_imu_timestamp_us_{0};
  uint64_t pending_run_period_us_{0};
  uint64_t pending_probe_start_imu_timestamp_us_{0};

  CameraFrameSyncRecording recording_{};
};

#include "camera_frame_sync_impl.hpp"
#include "camera_frame_sync_state_machine.hpp"
