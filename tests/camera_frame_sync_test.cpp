// 端到端：假 MCU（1 kHz IMU）+ 真实 CameraSync + 触发驱动的假相机 + CameraFrameSync。
// End to end: a fake MCU (1 kHz IMU), the real CameraSync, a trigger-driven fake camera
// and CameraFrameSync.
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "CameraFrameSync.hpp"
#include "CameraSync.hpp"
#include "libxr.hpp"

namespace
{
void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

constexpr CameraTypes::CameraCalibration CALIBRATION{
    1440,
    1080,
    2328.69,
    2328.67,
    733.36,
    540.62,
    {-0.0918, 0.4640, 0.0026, 0.0010, -0.4751}};
constexpr uint32_t PERIOD_US = 10000;
constexpr int32_t OFFSET_US = 500;
/// 假相机的设备时间 = 边沿时间 + 300 µs / Fake device time = edge time + 300 µs.
constexpr uint64_t DEVICE_DELAY_US = 300;

uint64_t NowUs()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}

/// 假 MCU 当前 IMU 样本的时间，CameraSync 在这条样本的回调里写触发电平。
/// Time of the fake MCU's current IMU sample; CameraSync writes the trigger within it.
std::atomic<uint64_t> g_imu_time{0};
uint64_t g_t0 = 0;

/// 由触发上升沿出图的假相机；帧计数在切档后从 0 起，可指定漏掉下一个触发。
/// Fake camera that produces a frame per rising edge; the counter restarts after a view
/// switch, and it can be told to miss the next trigger.
class FakeCamera : public CameraBase, public LibXR::GPIO
{
 public:
  explicit FakeCamera(const char* name)
      : CameraBase(CALIBRATION, {0.5, 0.5}, name, SlotPolicy::DROP)
  {
    StartCapture();
  }
  ~FakeCamera() override { StopCapture(); }
  void Stop() { StopCapture(); }

  void Write(bool level) override
  {
    if (level && !level_)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (miss_next_.exchange(false))
      {
        ++missed;
      }
      else
      {
        edges_.push_back(g_imu_time.load());
        cv_.notify_one();
      }
    }
    level_ = level;
  }
  bool Read() override { return level_; }
  LibXR::ErrorCode EnableInterrupt() override { return LibXR::ErrorCode::OK; }
  LibXR::ErrorCode DisableInterrupt() override { return LibXR::ErrorCode::OK; }
  LibXR::ErrorCode SetConfig(Configuration) override { return LibXR::ErrorCode::OK; }

  std::atomic<bool> miss_next_{false};
  std::atomic<int> missed{0};
  std::atomic<int> switches{0};

 private:
  bool GrabFrame(ImageFrame& frame) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, std::chrono::milliseconds(20),
                      [this]() { return !edges_.empty(); }))
    {
      return false;
    }
    frame.timestamp_us = LibXR::MicrosecondTimestamp(edges_.front() + DEVICE_DELAY_US);
    edges_.pop_front();
    frame.frame_counter = counter_++;
    return true;
  }

  LibXR::ErrorCode ApplyView(const CameraTypes::FrameGeometry&) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    counter_ = 0;  // 与 Hik 一样，重新开始取流后计数归零 / Restarts like Hik
    edges_.clear();
    switches.fetch_add(1);
    return LibXR::ErrorCode::OK;
  }

  bool level_ = false;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<uint64_t> edges_;
  uint32_t counter_ = 0;
};

struct Synced
{
  uint64_t sequence;
  uint64_t imu_time;
  float gyro_x;
  uint64_t device_time;
  uint32_t counter;
  CameraTypes::FrameGeometry geometry;
};

struct Receiver
{
  std::mutex mutex;
  std::vector<Synced> frames;

  explicit Receiver(const char* topic)
  {
    auto callback = LibXR::Topic::Callback::Create(
        [](bool, Receiver* self, const AutoAim::SyncedFrame* s)
        {
          std::lock_guard<std::mutex> lock(self->mutex);
          self->frames.push_back({s->sequence, static_cast<uint64_t>(s->imu.timestamp_us),
                                  s->imu.angular_velocity_xyz[0],
                                  static_cast<uint64_t>(s->image->timestamp_us),
                                  s->image->frame_counter, s->image->geometry});
        },
        this);
    AutoAim::RequireTopic<const AutoAim::SyncedFrame*>(topic).RegisterCallback(callback);
  }

  std::vector<Synced> Take()
  {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<Synced> out;
    out.swap(frames);
    return out;
  }
};

/// 每帧的 IMU 都取在自己的边沿 + offset，角速度等于该时刻的值。
/// Every frame's IMU is taken at its own edge + offset, and the angular velocity is the
/// value at that time.
void CheckPaired(const std::vector<Synced>& frames, const char* phase)
{
  for (const Synced& f : frames)
  {
    const uint64_t edge = f.device_time - DEVICE_DELAY_US;
    if (f.imu_time != edge + OFFSET_US)
    {
      std::fprintf(stderr, "%s: counter %u imu %llu edge %llu\n", phase, f.counter,
                   static_cast<unsigned long long>(f.imu_time),
                   static_cast<unsigned long long>(edge));
      Expect(false, "IMU taken at the frame's own edge + offset");
    }
    const float expected = static_cast<float>(f.imu_time - g_t0) * 1e-6F;
    Expect(std::fabs(f.gyro_x - expected) < 1e-4F, "interpolated IMU value");
  }
  for (std::size_t i = 1; i < frames.size(); ++i)
  {
    Expect(frames[i].sequence == frames[i - 1].sequence + 1, "sequence increases by one");
  }
}
}  // namespace

int main()
{
  LibXR::PlatformInit();
  g_t0 = NowUs();

  // BSP 里 CFS 排在 SharedTopic 之前，MCU 的 Topic 不存在时由它创建 / In the BSP CFS
  // precedes SharedTopic and creates absent MCU Topics.
  auto* mk_camera = new FakeCamera("mk");
  [[maybe_unused]] auto* mk_sync =
      new CameraFrameSync(*mk_camera, {SyncMode::LATEST_IMU, PERIOD_US, OFFSET_US, "",
                                       "mk_gyro", "mk_accl", "mk_quat", ImuAxes::BODY});
  Expect(LibXR::Topic::Find("mk_gyro") != nullptr &&
             LibXR::Topic::Find("mk_accl") != nullptr &&
             LibXR::Topic::Find("mk_quat") != nullptr,
         "CFS creates the absent MCU Topics");

  auto* camera = new FakeCamera("e2e");
  LibXR::Topic gyro = LibXR::Topic::CreateTopic<CameraFrameSync::ImuVector>("e2e_gyro");
  LibXR::Topic accl = LibXR::Topic::CreateTopic<CameraFrameSync::ImuVector>("e2e_accl");
  LibXR::Topic quat =
      LibXR::Topic::CreateTopic<CameraFrameSync::ImuQuaternion>("e2e_quat");
  auto* mcu = new CameraSync(
      *camera, {"camera_sync_result", "e2e_gyro", PERIOD_US, "camera_sync_command"});
  auto* sync =
      new CameraFrameSync(*camera, {SyncMode::TRIGGER, PERIOD_US, OFFSET_US, "",
                                    "e2e_gyro", "e2e_accl", "e2e_quat", ImuAxes::BODY});
  auto* receiver = new Receiver("e2e_synced");

  std::atomic<bool> imu_running{true};
  std::thread imu_thread(
      [&]()
      {
        auto next = std::chrono::steady_clock::now();
        while (imu_running.load())
        {
          const uint64_t t = NowUs();
          g_imu_time.store(t);
          const LibXR::MicrosecondTimestamp ts(t);
          CameraFrameSync::ImuVector a(0.0F, 0.0F, 9.8F);
          CameraFrameSync::ImuVector w(static_cast<float>(t - g_t0) * 1e-6F, 0.0F, 0.0F);
          CameraFrameSync::ImuQuaternion q(1.0F, 0.0F, 0.0F, 0.0F);
          // MCU 顺序：accl → gyro（CameraSync 在此触发）→ quat / MCU order.
          accl.Publish(a, ts);
          gyro.Publish(w, ts);
          quat.Publish(q, ts);
          next += std::chrono::milliseconds(1);
          std::this_thread::sleep_until(next);
        }
      });

  // 1. 正常运行 / Steady running.
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  auto frames = receiver->Take();
  std::printf("steady: %zu synced frames\n", frames.size());
  Expect(frames.size() >= 20, "frames flow after start-up");
  CheckPaired(frames, "steady");

  // 2. 相机漏一个触发：错位的帧不发布，重新同步后继续 / The camera misses a trigger:
  // the shifted frame is not published, and frames resume after the resync.
  camera->miss_next_.store(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  frames = receiver->Take();
  std::printf("missed trigger: %zu synced frames, missed %d\n", frames.size(),
              camera->missed.load());
  Expect(camera->missed.load() == 1, "one trigger missed");
  Expect(frames.size() >= 20, "frames resume after the resync");
  CheckPaired(frames, "after missed trigger");

  // 3. 切档：STOP → 切档 → START，之后的帧带 NARROW 几何 / View switch.
  sync->RequestView(View::NARROW);
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  frames = receiver->Take();
  std::printf("narrow: %zu synced frames, switches %d\n", frames.size(),
              camera->switches.load());
  Expect(camera->switches.load() == 1, "camera switched once");
  Expect(frames.size() >= 20 && frames.back().geometry.decimation == 1, "NARROW frames");
  CheckPaired(frames, "narrow");
  sync->OnMonitor();

  imu_running.store(false);
  imu_thread.join();
  camera->Stop();
  delete sync;  // 放掉挂起与排队的图像 / Releases pending and queued images
  delete camera;
  UNUSED(mcu);
  UNUSED(receiver);

  // 4. LATEST_IMU：自由运行的台架调试，每帧取最新 IMU / Bench free-run: newest IMU.
  auto* bench = new FakeCamera("bench");
  LibXR::Topic bench_gyro =
      LibXR::Topic::CreateTopic<CameraFrameSync::ImuVector>("bench_gyro");
  LibXR::Topic bench_accl =
      LibXR::Topic::CreateTopic<CameraFrameSync::ImuVector>("bench_accl");
  LibXR::Topic bench_quat =
      LibXR::Topic::CreateTopic<CameraFrameSync::ImuQuaternion>("bench_quat");
  auto* latest = new CameraFrameSync(
      *bench, {SyncMode::LATEST_IMU, 0, 0, "", "bench_gyro", "bench_accl", "bench_quat",
               ImuAxes::X_FORWARD_Y_LEFT_Z_UP});
  auto* bench_receiver = new Receiver("bench_synced");
  for (uint64_t i = 1; i <= 5; ++i)
  {
    const LibXR::MicrosecondTimestamp ts(1000 * i);
    CameraFrameSync::ImuVector a(0.0F, 0.0F, 9.8F);
    // x 前、y 左、z 上的原始数据：绕 y 转 i rad/s，机体系 x 应为 -i / Raw x-forward,
    // y-left, z-up data: i rad/s about y, so body x is -i.
    CameraFrameSync::ImuVector w(0.0F, static_cast<float>(i), 0.0F);
    CameraFrameSync::ImuQuaternion q(1.0F, 0.0F, 0.0F, 0.0F);
    bench_accl.Publish(a, ts);
    bench_gyro.Publish(w, ts);
    bench_quat.Publish(q, ts);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    g_imu_time.store(1000 * i);
    bench->Write(true);
    bench->Write(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  frames = bench_receiver->Take();
  Expect(frames.size() == 5, "every bench frame published");
  for (std::size_t i = 0; i < frames.size(); ++i)
  {
    Expect(frames[i].imu_time == 1000 * (i + 1) &&
               frames[i].gyro_x == -static_cast<float>(i + 1),
           "the newest IMU at arrival, converted to the body frame");
  }
  bench->Stop();
  delete latest;
  delete bench;
  UNUSED(bench_receiver);
  std::puts("camera_frame_sync_test passed");
  return 0;
}
