#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机帧同步：按触发边沿给每张图配上 IMU，发布同步帧 / Camera frame sync that pairs every image with the IMU at its trigger edge and publishes synced frames
depends:
- id: QDU-Robomaster/CameraBase
  ref: same-or-dev
- id: QDU-Robomaster/AutoAimTypes
  ref: same-or-dev
- id: QDU-Robomaster/CameraSync
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <Eigen/Core>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>

#include "AutoAimTypes.hpp"
#include "CameraBase.hpp"
#include "FrameMatcher.hpp"
#include "ImuHistory.hpp"
#include "TriggerLink.hpp"
#include "libxr_def.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "transform.hpp"

/// 同步方式 / Sync mode.
enum class SyncMode : uint8_t
{
  TRIGGER,     ///< MCU 外触发，按边沿配对 / MCU trigger, matched by edge
  LATEST_IMU,  ///< 台架自由运行调试：取最新 IMU / Bench free-run: newest IMU
};

/// MCU IMU 数据的坐标轴 / Axes of the MCU IMU data.
enum class ImuAxes : uint8_t
{
  BODY,  ///< 已是机体系 x 右、y 前、z 上 / Already body x right, y forward, z up
  X_FORWARD_Y_LEFT_Z_UP,  ///< x 前、y 左、z 上，转成机体系 / Converted to the body frame
};

/// 同步设置，与 YAML 一一对应 / Sync settings, one-to-one with the YAML.
struct FrameSyncSettings
{
  SyncMode mode;
  uint32_t trigger_period_us;   ///< 须大于曝光 + 读出 / Must exceed exposure + readout
  int32_t offset_us;            ///< 边沿到曝光中点 / From the edge to mid-exposure
  std::string_view mcu_domain;  ///< MCU Topic 所在 domain，空为默认 / Empty = default
  std::string_view gyro_topic;  ///< MCU 发布的 IMU Topic 名 / IMU Topics as the MCU
  std::string_view accl_topic;  ///< publishes them
  std::string_view quat_topic;
  ImuAxes imu_axes;  ///< MCU IMU 的坐标轴 / Axes of the MCU IMU
};

/**
 * @brief 相机帧同步。订阅相机图像、三路 MCU IMU 与 CameraSync 的触发事件，在自己的工作
 *        线程上给每张图配上曝光时刻的 IMU，发布 `<相机名>_synced`。
 *        Camera frame sync. It subscribes to the camera images, the three MCU IMU
 *        streams and CameraSync's trigger events, and on its own worker thread pairs
 *        each image with the IMU at its exposure time and publishes `<camera>_synced`.
 *
 * TRIGGER：启动时让 MCU 停触发再以 `trigger_period_us` 重新开始；图像按帧计数对应边沿，
 * IMU 取边沿时刻加 `offset_us` 处的插值。计数与边沿矛盾（相机漏触发、计数复位、MCU 重启）
 * 时重新同步。LATEST_IMU 只用于没有触发的台架调试：图像到达时取最新 IMU，不保证同步。
 * TRIGGER: at start-up the MCU stops and restarts the trigger at `trigger_period_us`;
 * images are matched to edges by frame counter and get the IMU interpolated at the edge
 * time plus `offset_us`. A contradiction (missed trigger, counter reset, MCU reboot)
 * triggers a resync. LATEST_IMU is only for bench debugging without a trigger: each
 * image gets the newest IMU, with no sync guarantee.
 *
 * Topic 回调注册后不能注销，本模块须与进程同寿命。
 * Topic callbacks cannot be unregistered, so this Module lives as long as the process.
 */
class CameraFrameSync
{
 public:
  using ImuVector = Eigen::Matrix<float, 3, 1>;
  using ImuQuaternion = LibXR::Quaternion<float>;
  using SyncCommand = CameraSyncDetail::SyncCommand;
  using SyncEvent = CameraSyncDetail::SyncEvent;

  /// CameraSync 的命令与事件 Topic / CameraSync command and event Topics.
  static constexpr const char* COMMAND_TOPIC = "camera_sync_command";
  static constexpr const char* EVENT_TOPIC = "camera_sync_result";
  /// 图像等边沿或 IMU 的最长时间 / Longest an image waits for its edge or IMU.
  static constexpr uint64_t PENDING_TIMEOUT_US = 100000;
  /// 最多挂起的图像数：挂起的图像占着相机的图像槽 / Images held while waiting; they
  /// occupy camera slots.
  static constexpr std::size_t MAX_PENDING = 1;
  /// 输入队列容量 / Input queue capacity.
  static constexpr std::size_t QUEUE_CAPACITY = 4096;

  CameraFrameSync(CameraBase& camera, const FrameSyncSettings& settings)
      : camera_(camera),
        mode_(settings.mode),
        imu_axes_(settings.imu_axes),
        offset_us_(settings.offset_us),
        synced_topic_(LibXR::Topic::CreateTopic<const AutoAim::SyncedFrame*>(
            StageTopicName(camera.Name(), AutoAim::STAGE_SYNCED).c_str())),
        link_(settings.trigger_period_us)
  {
    REQUIRE(mode_ == SyncMode::LATEST_IMU || settings.trigger_period_us > 0);
    if (!settings.mcu_domain.empty())
    {
      domain_.emplace(std::string(settings.mcu_domain).c_str());
    }
    LibXR::Topic::Domain* domain = domain_ ? &*domain_ : nullptr;

    Subscribe<ImageTopicPayload>(StageTopicName(camera.Name(), "image"), nullptr,
                                 [](bool, CameraFrameSync* self,
                                    ImageTopicPayload payload) { self->Push(*payload); });
    SubscribeMcu<ImuVector>(std::string(settings.gyro_topic), domain,
                            [](bool, CameraFrameSync* self, LibXR::MicrosecondTimestamp t,
                               ImuVector& v) { self->Push(Gyro{t, self->ToBody(v)}); });
    SubscribeMcu<ImuVector>(std::string(settings.accl_topic), domain,
                            [](bool, CameraFrameSync* self, LibXR::MicrosecondTimestamp t,
                               ImuVector& v) { self->Push(Accl{t, self->ToBody(v)}); });
    SubscribeMcu<ImuQuaternion>(
        std::string(settings.quat_topic), domain,
        [](bool, CameraFrameSync* self, LibXR::MicrosecondTimestamp t, ImuQuaternion& q)
        { self->Push(Quat{t, self->ToBody(q)}); });
    if (mode_ == SyncMode::TRIGGER)
    {
      SubscribeMcu<SyncEvent>(
          EVENT_TOPIC, domain,
          [](bool, CameraFrameSync* self, LibXR::MicrosecondTimestamp t, SyncEvent& e)
          { self->Push(Event{t, e}); });
      command_topic_ = LibXR::Topic::CreateTopic<SyncCommand>(COMMAND_TOPIC, domain);
    }
    running_.store(true);
    worker_ = std::thread([this]() { WorkerLoop(); });
  }

  ~CameraFrameSync()
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      running_.store(false);
    }
    queue_cv_.notify_one();
    worker_.join();
    queue_.clear();  // 放掉排队的图像 / Release queued images
  }

  CameraFrameSync(const CameraFrameSync&) = delete;
  CameraFrameSync& operator=(const CameraFrameSync&) = delete;

  /// 请求切档，任意线程可调用；TRIGGER 下走 STOP → 稳定 → 切档 → START。
  /// Request a view switch from any thread; in TRIGGER it runs STOP → settle → switch
  /// → START.
  void RequestView(View view) { Push(view); }

  /// 请求移动 NARROW 窗口，任意线程可调用；TRIGGER 下走 STOP → 稳定 → 移窗 → START，
  /// 停触发是屏障，START 之后的帧都是新窗口。不在 NARROW 时只记下位置。
  /// Request a NARROW window move from any thread; in TRIGGER it runs STOP → settle →
  /// move → START, with the stopped trigger as the barrier so every frame after START
  /// has the new window. Outside NARROW only the position is stored.
  void RequestMove(NarrowPosition narrow) { Push(narrow); }

  /// 打印周期摘要 / Print the periodic summary.
  void OnMonitor()
  {
    XR_LOG_INFO(
        "%s sync: published=%u no_edge=%u no_imu=%u overflow=%u resync=%u resend=%u "
        "imu_out_of_order=%u",
        camera_.Name().c_str(), stats_.published.exchange(0), stats_.no_edge.exchange(0),
        stats_.no_imu.exchange(0), stats_.overflow.exchange(0), stats_.resync.exchange(0),
        stats_.resend.exchange(0), stats_.imu_out_of_order.exchange(0));
  }

 private:
  struct Gyro
  {
    LibXR::MicrosecondTimestamp t;
    std::array<float, 3> v;
  };
  struct Accl
  {
    LibXR::MicrosecondTimestamp t;
    std::array<float, 3> v;
  };
  struct Quat
  {
    LibXR::MicrosecondTimestamp t;
    std::array<float, 4> wxyz;
  };
  struct Event
  {
    LibXR::MicrosecondTimestamp t;
    SyncEvent event;
  };
  /// 工作线程的输入，按到达顺序处理 / Worker input, handled in arrival order.
  using Input = std::variant<SharedFrame, Gyro, Accl, Quat, Event, View, NarrowPosition>;

  /// 等边沿或 IMU 的图像 / An image waiting for its edge or IMU.
  struct Pending
  {
    SharedFrame image;
    uint64_t arrived_us;
    std::optional<uint64_t> edge_time_us;
  };

  struct Stats
  {
    std::atomic<uint32_t> published{0};
    std::atomic<uint32_t> no_edge{0};
    std::atomic<uint32_t> no_imu{0};
    std::atomic<uint32_t> overflow{0};
    std::atomic<uint32_t> resync{0};
    std::atomic<uint32_t> resend{0};
    std::atomic<uint32_t> imu_out_of_order{0};
  };

  /// x 前、y 左、z 上转机体系：向量取 (-y, x, z)，四元数取 (w, -y, x, z)。
  /// x-forward, y-left, z-up to the body frame: vectors (-y, x, z), quaternions
  /// (w, -y, x, z).
  std::array<float, 3> ToBody(const ImuVector& v) const
  {
    if (imu_axes_ == ImuAxes::X_FORWARD_Y_LEFT_Z_UP)
    {
      return {-v.y(), v.x(), v.z()};
    }
    return {v.x(), v.y(), v.z()};
  }

  std::array<float, 4> ToBody(const ImuQuaternion& q) const
  {
    if (imu_axes_ == ImuAxes::X_FORWARD_Y_LEFT_Z_UP)
    {
      return {q.w(), -q.y(), q.x(), q.z()};
    }
    return {q.w(), q.x(), q.y(), q.z()};
  }

  /// 相机图像 Topic 必须已存在 / The camera image Topic must already exist.
  template <typename Payload, typename Fun>
  void Subscribe(const std::string& name, LibXR::Topic::Domain* domain, Fun fun)
  {
    auto callback = LibXR::Topic::Callback::Create(fun, this);
    AutoAim::RequireTopic<Payload>(name, domain).RegisterCallback(callback);
  }

  /**
   * @brief MCU 的 Topic 由 SharedTopic 按名字转发，它只认已存在的带类型 Topic，所以由这里
   *        创建（已存在则校验类型）；BSP 里本模块排在 SharedTopic 之前。
   *        MCU Topics are forwarded by name by SharedTopic, which only accepts existing
   *        typed Topics, so they are created here (or type-checked when they exist); the
   *        BSP lists this Module before SharedTopic.
   */
  template <typename Payload, typename Fun>
  void SubscribeMcu(const std::string& name, LibXR::Topic::Domain* domain, Fun fun)
  {
    auto callback = LibXR::Topic::Callback::Create(fun, this);
    LibXR::Topic(LibXR::Topic::FindOrCreate<Payload>(name.c_str(), domain))
        .RegisterCallback(callback);
  }

  /// 就地构造进队列，不移动 Input / Construct in the queue in place, without moving an
  /// Input.
  template <typename T>
  void Push(T&& value)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!running_.load())
      {
        return;
      }
      if (queue_.size() >= QUEUE_CAPACITY)
      {
        stats_.overflow.fetch_add(1, std::memory_order_relaxed);
        return;
      }
      queue_.emplace_back(std::in_place_type<std::decay_t<T>>, std::forward<T>(value));
    }
    queue_cv_.notify_one();
  }

  static uint64_t NowUs()
  {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
  }

  void WorkerLoop()
  {
    if (mode_ == SyncMode::TRIGGER)
    {
      Apply(link_.Restart(NowUs()));
    }
    std::deque<Input> batch;
    while (true)
    {
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait_for(lock, std::chrono::milliseconds(5),
                           [this]() { return !queue_.empty() || !running_.load(); });
        if (!running_.load())
        {
          break;
        }
        batch.swap(queue_);
      }
      for (Input& input : batch)
      {
        std::visit([this](auto& value) { Handle(value); }, input);
      }
      batch.clear();
      const uint64_t now = NowUs();
      if (mode_ == SyncMode::TRIGGER)
      {
        Apply(link_.Tick(now));
        stats_.resend.fetch_add(link_.TakeResends(), std::memory_order_relaxed);
        ProcessPending(now);
      }
      stats_.imu_out_of_order.fetch_add(history_.TakeOutOfOrder(),
                                        std::memory_order_relaxed);
    }
    pending_.clear();  // 放掉图像槽 / Release the image slots
  }

  void Handle(SharedFrame& image)
  {
    if (mode_ == SyncMode::LATEST_IMU)
    {
      const std::optional<AutoAim::ImuSample> imu = history_.Latest();
      if (imu)
      {
        Publish(image, *imu);
      }
      else
      {
        stats_.no_imu.fetch_add(1, std::memory_order_relaxed);
      }
      return;
    }
    if (pending_.size() >= MAX_PENDING)
    {
      pending_.pop_front();
      stats_.no_edge.fetch_add(1, std::memory_order_relaxed);
    }
    pending_.push_back({std::move(image), NowUs(), std::nullopt});
    ProcessPending(NowUs());
  }

  void Handle(const Gyro& s)
  {
    history_.AddAngularVelocity(static_cast<uint64_t>(s.t), s.v);
  }
  void Handle(const Accl& s)
  {
    history_.AddLinearAcceleration(static_cast<uint64_t>(s.t), s.v);
  }
  void Handle(const Quat& s) { history_.AddRotation(static_cast<uint64_t>(s.t), s.wxyz); }

  void Handle(const Event& e)
  {
    const uint64_t now = NowUs();
    if (e.event.operation != CameraSyncDetail::Operation::FRAME_TRIGGER)
    {
      Apply(link_.OnEvent(e.event, now));
      return;
    }
    if (!matcher_.OnEdge(e.event, static_cast<uint64_t>(e.t)))
    {
      Resync(now, "edge from another trigger run");
    }
  }

  void Handle(const View& view)
  {
    if (mode_ == SyncMode::LATEST_IMU)
    {
      camera_.SwitchView(view);
      return;
    }
    matcher_.Stop();
    pending_.clear();
    Apply(link_.Restart(NowUs(), view));
  }

  void Handle(const NarrowPosition& narrow)
  {
    if (mode_ == SyncMode::LATEST_IMU)
    {
      camera_.MoveNarrow(narrow);
      return;
    }
    matcher_.Stop();
    pending_.clear();
    Apply(link_.Restart(NowUs(), std::nullopt, narrow));
  }

  /// 先移窗、切档，再发命令，最后按 START 重新开始配对。先移窗：从 WIDE 切到 NARROW
  /// 时用新位置。
  /// Move and switch first, then send, then restart matching on START. The move comes
  /// first so a switch from WIDE to NARROW uses the new position.
  void Apply(const TriggerLink::Actions& actions)
  {
    if (actions.move_narrow)
    {
      camera_.MoveNarrow(*actions.move_narrow);
    }
    if (actions.switch_view)
    {
      camera_.SwitchView(*actions.switch_view);
    }
    if (actions.send)
    {
      SyncCommand command = *actions.send;
      command_topic_.Publish(command);
    }
    if (actions.started)
    {
      pending_.clear();
      matcher_.Reset(*actions.started, link_.PeriodUs());
      XR_LOG_INFO("%s sync: trigger running, period %u us", camera_.Name().c_str(),
                  link_.PeriodUs());
    }
  }

  void Resync(uint64_t now, const char* reason)
  {
    if (!matcher_.Running())
    {
      return;  // 已在重新同步 / Already resyncing
    }
    const uint32_t count = stats_.resync.fetch_add(1, std::memory_order_relaxed) + 1;
    ++total_resyncs_;
    if (AutoAim::ShouldLog(total_resyncs_))
    {
      XR_LOG_WARN("%s sync: resync (%s), %u in this period", camera_.Name().c_str(),
                  reason, count);
    }
    matcher_.Stop();
    pending_.clear();
    Apply(link_.Restart(now));
  }

  void ProcessPending(uint64_t now)
  {
    while (!pending_.empty())
    {
      Pending& p = pending_.front();
      const bool timed_out = now - p.arrived_us > PENDING_TIMEOUT_US;
      if (!p.edge_time_us)
      {
        uint64_t edge_time_us = 0;
        const FrameMatcher::Result result =
            matcher_.Match(p.image->frame_counter,
                           static_cast<uint64_t>(p.image->timestamp_us), edge_time_us);
        if (result == FrameMatcher::Result::RESYNC)
        {
          Resync(now, "frame counter contradicts the edges");
          return;
        }
        if (result == FrameMatcher::Result::WAIT && !timed_out)
        {
          return;
        }
        if (result != FrameMatcher::Result::MATCHED)
        {
          stats_.no_edge.fetch_add(1, std::memory_order_relaxed);
          pending_.pop_front();
          continue;
        }
        p.edge_time_us = edge_time_us;
      }
      AutoAim::ImuSample imu{};
      const int64_t t = static_cast<int64_t>(*p.edge_time_us) + offset_us_;
      const ImuHistory::Lookup lookup = history_.At(static_cast<uint64_t>(t), imu);
      if (lookup == ImuHistory::Lookup::NOT_YET && !timed_out)
      {
        return;
      }
      if (lookup == ImuHistory::Lookup::OK)
      {
        Publish(p.image, imu);
      }
      else
      {
        stats_.no_imu.fetch_add(1, std::memory_order_relaxed);
      }
      pending_.pop_front();
    }
  }

  void Publish(const SharedFrame& image, const AutoAim::ImuSample& imu)
  {
    const AutoAim::SyncedFrame synced{++sequence_, image, imu};
    const AutoAim::SyncedFrame* payload = &synced;
    synced_topic_.Publish(payload);
    stats_.published.fetch_add(1, std::memory_order_relaxed);
  }

  CameraBase& camera_;
  const SyncMode mode_;
  const ImuAxes imu_axes_;
  const int32_t offset_us_;
  std::optional<LibXR::Topic::Domain> domain_;
  LibXR::Topic synced_topic_;
  LibXR::Topic command_topic_;

  // 只在工作线程使用 / Worker thread only.
  TriggerLink link_;
  FrameMatcher matcher_;
  ImuHistory history_;
  std::deque<Pending> pending_;
  uint64_t sequence_ = 0;
  uint64_t total_resyncs_ = 0;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<Input> queue_;
  std::atomic<bool> running_{false};
  Stats stats_;
  std::thread worker_;
};
