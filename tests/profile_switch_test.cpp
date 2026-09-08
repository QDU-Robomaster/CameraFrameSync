#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "CameraFrameSync.hpp"
#include "CameraSyncStateMachine.hpp"

namespace
{
constexpr CameraTypes::FrameLayout kLayout{2, 2, 6, CameraTypes::Encoding::BGR8};
using Camera = CameraBase<kLayout>;
using Sync = CameraFrameSync<kLayout>;
using Operation = CameraSync::Operation;
using SyncCommand = CameraSync::SyncCommand;
using SyncEvent = CameraSync::SyncEvent;

constexpr std::string_view kDomainName = "cfs_profile_test_domain";
constexpr std::string_view kCameraName = "cfs_profile_test_camera";
constexpr std::string_view kImageTopicName = "cfs_profile_test_image";
constexpr std::string_view kImuTopicName = "cfs_profile_test_imu";
constexpr std::string_view kCommandTopicName = "cfs_profile_test_command";
constexpr std::string_view kResultTopicName = "cfs_profile_test_result";
constexpr std::string_view kSyncedTopicName = "cfs_profile_test_synced";
constexpr uint64_t kStartupStopAckUs = 1000U;
constexpr uint64_t kSettleUs = 10000U;

[[noreturn]] void Fail(std::string_view message)
{
  std::cerr << "[FAIL] " << message << '\n';
  std::cerr.flush();
  std::cout.flush();
  std::_Exit(EXIT_FAILURE);
}

void Expect(bool condition, std::string_view message)
{
  if (!condition)
  {
    Fail(message);
  }
}

[[noreturn]] void Pass(std::string_view name)
{
  std::cout << "[PASS] " << name << '\n';
  std::cout.flush();
  std::cerr.flush();
  std::_Exit(EXIT_SUCCESS);
}

constexpr Camera::CameraCalibration MakeCalibration()
{
  Camera::CameraCalibration calibration{};
  calibration.native_width = 4U;
  calibration.native_height = 2U;
  calibration.camera_matrix = {1.0, 0.0, 2.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0};
  return calibration;
}

constexpr Camera::FrameGeometry MakeWideGeometry()
{
  return {
      .width = 2U,
      .height = 2U,
      .step = 6U,
      .roi_offset_x_native = 0U,
      .roi_offset_y_native = 0U,
      .decimation_x = 2U,
      .decimation_y = 1U,
      .flags = CameraTypes::FRAME_GEOMETRY_NONE,
      .reserved = 0U,
      .sample_phase_x_native = 0.0F,
      .sample_phase_y_native = 0.0F,
  };
}

constexpr Camera::FrameGeometry MakeNarrowGeometry()
{
  return {
      .width = 2U,
      .height = 2U,
      .step = 6U,
      .roi_offset_x_native = 1U,
      .roi_offset_y_native = 0U,
      .decimation_x = 1U,
      .decimation_y = 1U,
      .flags = CameraTypes::FRAME_GEOMETRY_NONE,
      .reserved = 0U,
      .sample_phase_x_native = 0.0F,
      .sample_phase_y_native = 0.0F,
  };
}

constexpr std::array<Camera::CameraProfile, 2U> kProfiles{{
    {.id = Camera::ProfileId::WIDE,
     .geometry = MakeWideGeometry(),
     .trigger_period_us = 10000U},
    {.id = Camera::ProfileId::NARROW,
     .geometry = MakeNarrowGeometry(),
     .trigger_period_us = 5000U},
}};

enum class TraceEvent : uint8_t
{
  STOP_COMMAND,
  SWITCH_CAMERA,
  START_COMMAND,
};

struct CommandRecorder
{
  Sync* sync{};
  std::vector<SyncCommand> commands{};
  std::vector<Camera::ProfileId> active_profiles{};
  std::vector<TraceEvent>* trace{};

  void Clear()
  {
    commands.clear();
    active_profiles.clear();
  }
};

void OnCommand(bool, CommandRecorder* recorder, const SyncCommand& command)
{
  recorder->commands.push_back(command);
  if (recorder->sync != nullptr)
  {
    recorder->active_profiles.push_back(recorder->sync->ActiveProfile());
  }

  if (recorder->trace == nullptr)
  {
    return;
  }
  if (command.operation == Operation::STOP_TRIGGER)
  {
    recorder->trace->push_back(TraceEvent::STOP_COMMAND);
  }
  else if (command.operation == Operation::START_TRIGGER)
  {
    recorder->trace->push_back(TraceEvent::START_COMMAND);
  }
}

struct ImageRecorder
{
  std::vector<Camera::SharedFrame> frames{};
};

void OnImage(bool, ImageRecorder* recorder, Camera::ImageTopicPayload borrowed)
{
  Expect(borrowed != nullptr && borrowed->Valid(),
         "camera topic must borrow a valid SharedFrame");
  recorder->frames.push_back(*borrowed);
}

struct SyncedFrameRecorder
{
  std::vector<Sync::SyncedFrame> frames{};
};

void OnSyncedFrame(bool, SyncedFrameRecorder* recorder,
                   Sync::SyncedFrameTopicPayload borrowed)
{
  Expect(borrowed != nullptr && borrowed->Valid(),
         "synced topic must borrow a valid SyncedFrame");
  recorder->frames.push_back(*borrowed);
}

struct BlockingSyncedSubscriber
{
  std::atomic<bool> enabled{false};
  std::atomic<bool> entered{false};
  std::atomic<bool> release{false};
};

void OnBlockingSyncedFrame(bool, BlockingSyncedSubscriber* blocker,
                           Sync::SyncedFrameTopicPayload)
{
  if (!blocker->enabled.load(std::memory_order_acquire))
  {
    return;
  }
  blocker->entered.store(true, std::memory_order_release);
  while (!blocker->release.load(std::memory_order_acquire))
  {
    std::this_thread::yield();
  }
}

enum class SwitchBehavior : uint8_t
{
  SUCCEED,
  FAIL,
};

class MockCamera final : public Camera
{
 public:
  MockCamera(LibXR::HardwareContainer& hw, SwitchBehavior behavior,
             std::vector<TraceEvent>* trace, size_t profile_count)
      : Camera(hw, MakeCalibration(), kCameraName, kImageTopicName, kImuTopicName),
        behavior_(behavior),
        trace_(trace),
        profile_count_(profile_count)
  {
    Expect(profile_count_ > 0U && profile_count_ <= kProfiles.size(),
           "mock camera profile count must be one or two");
  }

  void SetExposure(double) override {}
  void SetGain(double) override {}

  std::span<const CameraProfile> Profiles() const noexcept override
  {
    return {kProfiles.data(), profile_count_};
  }

  LibXR::ErrorCode SwitchProfile(ProfileId id, AppliedProfile& applied) override
  {
    ++switch_count_;
    requested_profiles_.push_back(id);
    if (trace_ != nullptr)
    {
      trace_->push_back(TraceEvent::SWITCH_CAMERA);
    }
    if (sync_ != nullptr)
    {
      active_before_switch_.push_back(sync_->ActiveProfile());
    }
    if (switch_hook_)
    {
      switch_hook_();
    }

    if (behavior_ == SwitchBehavior::FAIL)
    {
      return LibXR::ErrorCode::STATE_ERR;
    }
    for (const auto& profile : Profiles())
    {
      if (profile.id == id)
      {
        driver_profile_ = id;
        applied = {.id = id, .geometry = profile.geometry};
        return LibXR::ErrorCode::OK;
      }
    }
    return LibXR::ErrorCode::NOT_SUPPORT;
  }

  void BindSync(Sync& sync) { sync_ = &sync; }
  void SetSwitchHook(std::function<void()> hook) { switch_hook_ = std::move(hook); }

  [[nodiscard]] size_t SwitchCount() const { return switch_count_; }
  [[nodiscard]] ProfileId DriverProfile() const { return driver_profile_; }
  [[nodiscard]] const std::vector<ProfileId>& ActiveBeforeSwitch() const
  {
    return active_before_switch_;
  }

 private:
  SwitchBehavior behavior_;
  std::vector<TraceEvent>* trace_{};
  size_t profile_count_{};
  Sync* sync_{};
  std::function<void()> switch_hook_{};
  size_t switch_count_{};
  ProfileId driver_profile_{ProfileId::WIDE};
  std::vector<ProfileId> requested_profiles_{};
  std::vector<ProfileId> active_before_switch_{};
};

class Harness
{
 public:
  Harness(Sync::SyncMode mode, SwitchBehavior behavior = SwitchBehavior::SUCCEED,
          bool retain_images = false, size_t profile_count = kProfiles.size())
      : hw_(LibXR::Entry<LibXR::RamFS>{ramfs_, {"ramfs"}}),
        domain_(kDomainName.data()),
        command_topic_(
            LibXR::Topic::FindOrCreate<SyncCommand>(kCommandTopicName.data(), &domain_)),
        result_topic_(
            LibXR::Topic::FindOrCreate<SyncEvent>(kResultTopicName.data(), &domain_)),
        gyro_topic_(LibXR::Topic::FindOrCreate<Sync::RawImuVector>(
            "cfs_profile_test_camera_gyro", &domain_)),
        accl_topic_(LibXR::Topic::FindOrCreate<Sync::RawImuVector>(
            "cfs_profile_test_camera_accl", &domain_)),
        quat_topic_(LibXR::Topic::FindOrCreate<Sync::RawQuatSample>(
            "cfs_profile_test_camera_quat", &domain_)),
        image_topic_(LibXR::Topic::FindOrCreate<Camera::ImageTopicPayload>(
            kImageTopicName.data())),
        synced_topic_(LibXR::Topic::FindOrCreate<Sync::SyncedFrameTopicPayload>(
            kSyncedTopicName.data())),
        command_callback_(LibXR::Topic::Callback::Create(OnCommand, &commands_)),
        image_callback_(LibXR::Topic::Callback::Create(OnImage, &images_)),
        synced_callback_(LibXR::Topic::Callback::Create(OnSyncedFrame, &synced_frames_)),
        blocking_synced_callback_(
            LibXR::Topic::Callback::Create(OnBlockingSyncedFrame, &synced_blocker_)),
        camera_(hw_, behavior, &trace_, profile_count)
  {
    commands_.trace = &trace_;
    command_topic_.RegisterCallback(command_callback_);
    synced_topic_.RegisterCallback(synced_callback_);
    synced_topic_.RegisterCallback(blocking_synced_callback_);
    if (retain_images)
    {
      image_topic_.RegisterCallback(image_callback_);
    }

    Sync::RuntimeParam runtime{};
    runtime.mode = mode;
    runtime.host_topic_domain_name = kDomainName;
    runtime.sync_command_topic_name = kCommandTopicName;
    runtime.sync_result_topic_name = kResultTopicName;
    runtime.synced_frame_topic_name = kSyncedTopicName;
    runtime.camera_settle_us = kSettleUs;
    sync_.emplace(hw_, app_, camera_, runtime);
    camera_.BindSync(*sync_);
    commands_.sync = &*sync_;
  }

  Sync& SyncUnderTest() { return *sync_; }
  MockCamera& CameraUnderTest() { return camera_; }
  CommandRecorder& Commands() { return commands_; }
  ImageRecorder& Images() { return images_; }
  SyncedFrameRecorder& SyncedFrames() { return synced_frames_; }
  std::vector<TraceEvent>& Trace() { return trace_; }

  void ClearCommandTrace()
  {
    commands_.Clear();
    trace_.clear();
  }

  void EnableSyncedBlock()
  {
    synced_blocker_.release.store(false, std::memory_order_relaxed);
    synced_blocker_.entered.store(false, std::memory_order_relaxed);
    synced_blocker_.enabled.store(true, std::memory_order_release);
  }

  void WaitForSyncedBlock()
  {
    for (size_t attempt = 0U; attempt < 2000U; ++attempt)
    {
      if (synced_blocker_.entered.load(std::memory_order_acquire))
      {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Fail("timed out waiting for blocked synced callback");
  }

  void ReleaseSyncedBlock()
  {
    synced_blocker_.release.store(true, std::memory_order_release);
  }

  [[nodiscard]] uint64_t LastImuTimestamp() const { return last_imu_timestamp_us_; }

  void PublishImu(uint64_t timestamp_us)
  {
    Sync::RawImuVector vector;
    vector << 0.0F, 0.0F, 0.0F;
    Sync::RawQuatSample quat(1.0F, 0.0F, 0.0F, 0.0F);
    const LibXR::MicrosecondTimestamp timestamp(timestamp_us);
    gyro_topic_.Publish(vector, timestamp);
    accl_topic_.Publish(vector, timestamp);
    quat_topic_.Publish(quat, timestamp);
    last_imu_timestamp_us_ = timestamp_us;
  }

  void PublishGyroOnly(uint64_t timestamp_us)
  {
    Sync::RawImuVector vector;
    vector << 0.0F, 0.0F, 0.0F;
    gyro_topic_.Publish(vector, LibXR::MicrosecondTimestamp(timestamp_us));
    last_imu_timestamp_us_ = timestamp_us;
  }

  void PublishAcclAndQuat(uint64_t timestamp_us)
  {
    Sync::RawImuVector vector;
    vector << 0.0F, 0.0F, 0.0F;
    Sync::RawQuatSample quat(1.0F, 0.0F, 0.0F, 0.0F);
    const LibXR::MicrosecondTimestamp timestamp(timestamp_us);
    accl_topic_.Publish(vector, timestamp);
    quat_topic_.Publish(quat, timestamp);
  }

  void PublishQuatOnly(uint64_t timestamp_us)
  {
    Sync::RawQuatSample quat(1.0F, 0.0F, 0.0F, 0.0F);
    quat_topic_.Publish(quat, LibXR::MicrosecondTimestamp(timestamp_us));
  }

  void PublishEvent(const SyncEvent& event, uint64_t timestamp_us)
  {
    SyncEvent copy = event;
    result_topic_.Publish(copy, LibXR::MicrosecondTimestamp(timestamp_us));
  }

  void PublishFrame(uint64_t timestamp_us, const Camera::FrameGeometry& geometry)
  {
    Camera::ImageFrame* frame = camera_.GetWritableImage();
    Expect(frame != nullptr, "camera must provide a writable frame");
    frame->timestamp_us = LibXR::MicrosecondTimestamp(timestamp_us);
    frame->geometry = geometry;
    frame->data.fill(static_cast<uint8_t>(timestamp_us));
    Expect(camera_.CommitImage(), "camera frame commit must succeed");
  }

 private:
  LibXR::RamFS ramfs_{};
  LibXR::HardwareContainer hw_;
  LibXR::ApplicationManager app_{};
  LibXR::Topic::Domain domain_;
  LibXR::Topic command_topic_;
  LibXR::Topic result_topic_;
  LibXR::Topic gyro_topic_;
  LibXR::Topic accl_topic_;
  LibXR::Topic quat_topic_;
  LibXR::Topic image_topic_;
  LibXR::Topic synced_topic_;
  std::vector<TraceEvent> trace_{};
  CommandRecorder commands_{};
  ImageRecorder images_{};
  SyncedFrameRecorder synced_frames_{};
  BlockingSyncedSubscriber synced_blocker_{};
  LibXR::Topic::Callback command_callback_;
  LibXR::Topic::Callback image_callback_;
  LibXR::Topic::Callback synced_callback_;
  LibXR::Topic::Callback blocking_synced_callback_;
  MockCamera camera_;
  std::optional<Sync> sync_{};
  uint64_t last_imu_timestamp_us_{};
};

SyncEvent AckFor(const SyncCommand& command, uint32_t stop_trigger_sequence = 0U)
{
  return {
      .seq = command.seq,
      .operation = command.operation,
      .active_level = command.active_level,
      .reserved = 0U,
      .effective_period_us =
          command.operation == Operation::START_TRIGGER ? command.trigger_period_us : 0U,
      .trigger_sequence =
          command.operation == Operation::STOP_TRIGGER ? stop_trigger_sequence : 0U,
  };
}

SyncEvent FrameTriggerFor(const SyncCommand& start, uint32_t trigger_sequence)
{
  Expect(start.operation == Operation::START_TRIGGER,
         "FRAME_TRIGGER requires the active START command");
  return {
      .seq = start.seq,
      .operation = Operation::FRAME_TRIGGER,
      .active_level = start.active_level,
      .reserved = 0U,
      .effective_period_us = start.trigger_period_us,
      .trigger_sequence = trigger_sequence,
  };
}

void ExpectCommand(const SyncCommand& command, Operation operation,
                   uint32_t trigger_period_us, std::string_view context)
{
  Expect(command.operation == operation, context);
  Expect(command.active_level == 1U, context);
  Expect(command.seq != 0U, context);
  Expect(command.reserved == 0U, context);
  Expect(command.trigger_period_us == trigger_period_us, context);
}

void ExpectSameCommand(const SyncCommand& actual, const SyncCommand& expected,
                       std::string_view context)
{
  Expect(actual.operation == expected.operation, context);
  Expect(actual.active_level == expected.active_level, context);
  Expect(actual.seq == expected.seq, context);
  Expect(actual.reserved == expected.reserved, context);
  Expect(actual.trigger_period_us == expected.trigger_period_us, context);
}

void ExpectTrace(const std::vector<TraceEvent>& trace,
                 std::span<const TraceEvent> expected, std::string_view context)
{
  Expect(trace.size() == expected.size(), context);
  for (size_t index = 0U; index < expected.size(); ++index)
  {
    Expect(trace[index] == expected[index], context);
  }
}

struct StartupCommands
{
  SyncCommand stop{};
  SyncCommand start{};
};

StartupCommands CompleteStartup(Harness& harness)
{
  Expect(harness.Commands().commands.size() == 1U,
         "TRIGGER construction must publish one STOP");
  const SyncCommand stop = harness.Commands().commands.front();
  ExpectCommand(stop, Operation::STOP_TRIGGER, 0U, "startup STOP fields are wrong");

  harness.PublishEvent(AckFor(stop), kStartupStopAckUs);
  Expect(harness.Commands().commands.size() == 1U,
         "STOP ACK must enter settle without publishing START");
  harness.PublishImu(kStartupStopAckUs + kSettleUs - 1U);
  Expect(harness.Commands().commands.size() == 1U,
         "startup settle must not finish before its deadline");
  harness.PublishImu(kStartupStopAckUs + kSettleUs);
  Expect(harness.Commands().commands.size() == 2U,
         "startup settle deadline must publish START");

  const SyncCommand start = harness.Commands().commands.back();
  ExpectCommand(start, Operation::START_TRIGGER, kProfiles[0].trigger_period_us,
                "startup START fields are wrong");
  Expect(start.seq != stop.seq, "startup START must use a fresh sequence");
  harness.PublishEvent(AckFor(start), kStartupStopAckUs + kSettleUs + 1U);
  return {.stop = stop, .start = start};
}

void PublishEdge(Harness& harness, const SyncCommand& start, uint32_t trigger_sequence,
                 uint64_t timestamp_us)
{
  harness.PublishImu(timestamp_us);
  harness.PublishEvent(FrameTriggerFor(start, trigger_sequence), timestamp_us);
}

void TestStartupFirstTrigger()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  const StartupCommands startup = CompleteStartup(harness);

  PublishEdge(harness, startup.start, 1U, 21000U);
  harness.PublishFrame(100000U, MakeWideGeometry());

  Expect(harness.SyncedFrames().frames.size() == 1U,
         "first real edge after START must publish one frame");
  const Sync::SyncedFrame& frame = harness.SyncedFrames().frames.front();
  Expect(static_cast<uint64_t>(frame.GetImageFrame()->timestamp_us) == 100000U &&
             static_cast<uint64_t>(frame.imu.timestamp_us) == 21000U,
         "synced output must retain camera time and use trigger time for IMU");
  Pass("startup_first_trigger");
}

void TestTriggerStride()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  const StartupCommands startup = CompleteStartup(harness);
  constexpr std::array<uint64_t, 4U> expected_trigger_times{21000U, 41000U, 71000U,
                                                            81000U};
  std::array<uint64_t, expected_trigger_times.size()> actual_trigger_times{};
  size_t published_count = 0U;
  auto check_and_release = [&]()
  {
    Expect(harness.SyncedFrames().frames.size() == 1U,
           "each stride sample must publish one retained frame");
    actual_trigger_times[published_count++] =
        static_cast<uint64_t>(harness.SyncedFrames().frames.front().imu.timestamp_us);
    harness.SyncedFrames().frames.front().image.Reset();
    harness.SyncedFrames().frames.clear();
  };

  PublishEdge(harness, startup.start, 1U, 21000U);
  harness.PublishFrame(100000U, MakeWideGeometry());
  check_and_release();

  PublishEdge(harness, startup.start, 2U, 31000U);
  PublishEdge(harness, startup.start, 3U, 41000U);
  harness.PublishFrame(120000U, MakeWideGeometry());
  check_and_release();

  PublishEdge(harness, startup.start, 4U, 51000U);
  PublishEdge(harness, startup.start, 5U, 61000U);
  PublishEdge(harness, startup.start, 6U, 71000U);
  harness.PublishFrame(150000U, MakeWideGeometry());
  check_and_release();

  PublishEdge(harness, startup.start, 7U, 81000U);
  harness.PublishFrame(160000U, MakeWideGeometry());
  check_and_release();

  Expect(published_count == expected_trigger_times.size() &&
             actual_trigger_times == expected_trigger_times,
         "camera gap must select and consume the target trigger");
  Pass("trigger_stride");
}

void TestInvalidGapResync()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  const StartupCommands startup = CompleteStartup(harness);
  harness.ClearCommandTrace();

  PublishEdge(harness, startup.start, 1U, 21000U);
  harness.PublishFrame(100000U, MakeWideGeometry());
  PublishEdge(harness, startup.start, 2U, 31000U);
  harness.PublishFrame(116000U, MakeWideGeometry());

  Expect(harness.SyncedFrames().frames.size() == 1U,
         "unexplained camera residual must not publish");
  Expect(harness.Commands().commands.size() == 1U,
         "unexplained camera residual must request a same-profile STOP");
  const SyncCommand stop = harness.Commands().commands.front();
  ExpectCommand(stop, Operation::STOP_TRIGGER, 0U,
                "same-profile restart STOP fields are wrong");

  harness.PublishEvent(AckFor(stop, 2U), 32000U);
  harness.PublishImu(32000U + kSettleUs - 1U);
  Expect(harness.Commands().commands.size() == 1U,
         "same-profile restart must honor settle time");
  harness.PublishImu(32000U + kSettleUs);
  Expect(harness.Commands().commands.size() == 2U,
         "same-profile restart must publish START after settle");
  Expect(harness.CameraUnderTest().SwitchCount() == 0U,
         "same-profile restart must not call SwitchProfile");

  const SyncCommand restart = harness.Commands().commands.back();
  ExpectCommand(restart, Operation::START_TRIGGER, kProfiles[0].trigger_period_us,
                "same-profile restart START fields are wrong");
  harness.PublishEvent(AckFor(restart), 42001U);
  PublishEdge(harness, restart, 1U, 52000U);
  harness.PublishFrame(200000U, MakeWideGeometry());
  Expect(harness.SyncedFrames().frames.size() == 2U,
         "same-profile STOP/START must recover on sequence one");
  Pass("invalid_gap_resync");
}

void TestTriggerStreamResync()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  const StartupCommands startup = CompleteStartup(harness);
  harness.ClearCommandTrace();

  PublishEdge(harness, startup.start, 1U, 21000U);
  harness.PublishFrame(100000U, MakeWideGeometry());
  PublishEdge(harness, startup.start, 3U, 31000U);

  Expect(harness.Commands().commands.size() == 1U &&
             harness.Commands().commands.front().operation == Operation::STOP_TRIGGER,
         "non-contiguous trigger sequence must restart synchronization");
  Expect(harness.CameraUnderTest().SwitchCount() == 0U,
         "trigger-stream restart must not switch the camera");
  Pass("trigger_stream_resync");
}

void TestProfileSwitchOrder()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  CompleteStartup(harness);
  harness.ClearCommandTrace();

  bool switching_frame_injected = false;
  harness.CameraUnderTest().SetSwitchHook(
      [&harness, &switching_frame_injected]()
      {
        switching_frame_injected = true;
        harness.PublishFrame(500U, MakeWideGeometry());
        Expect(
            harness.CameraUnderTest().AvailableImageSlots() == Camera::image_slot_count,
            "SWITCHING_CAMERA image must release its SharedFrame");
      });

  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "NARROW request must be admitted from RUNNING");
  Expect(harness.Commands().commands.size() == 1U,
         "profile request must publish one STOP");
  const SyncCommand stop = harness.Commands().commands.front();
  ExpectCommand(stop, Operation::STOP_TRIGGER, 0U, "profile STOP fields are wrong");
  Expect(harness.CameraUnderTest().SwitchCount() == 0U,
         "camera must not switch before STOP ACK");

  harness.PublishEvent(AckFor(stop, 7U), 12000U);
  Expect(harness.CameraUnderTest().SwitchCount() == 0U &&
             harness.Commands().commands.size() == 1U,
         "STOP ACK must only enter settling");
  harness.PublishImu(21999U);
  Expect(harness.CameraUnderTest().SwitchCount() == 0U,
         "camera must not switch before the settle deadline");
  harness.PublishImu(22000U);

  Expect(switching_frame_injected && harness.CameraUnderTest().SwitchCount() == 1U &&
             harness.CameraUnderTest().DriverProfile() == Camera::ProfileId::NARROW,
         "settle deadline must switch the camera once");
  Expect(harness.Commands().commands.size() == 2U,
         "successful camera switch must publish START");
  const SyncCommand start = harness.Commands().commands.back();
  ExpectCommand(start, Operation::START_TRIGGER, kProfiles[1].trigger_period_us,
                "NARROW START fields are wrong");
  Expect(start.seq != stop.seq, "profile START must use a fresh sequence");
  Expect(harness.CameraUnderTest().ActiveBeforeSwitch().size() == 1U &&
             harness.CameraUnderTest().ActiveBeforeSwitch().front() ==
                 Camera::ProfileId::WIDE,
         "SwitchProfile must re-enter ActiveProfile outside the state lock");
  Expect(harness.Commands().active_profiles.size() == 2U &&
             harness.Commands().active_profiles[0] == Camera::ProfileId::WIDE &&
             harness.Commands().active_profiles[1] == Camera::ProfileId::NARROW,
         "command callbacks must re-enter ActiveProfile outside the state lock");

  constexpr std::array expected_trace{TraceEvent::STOP_COMMAND, TraceEvent::SWITCH_CAMERA,
                                      TraceEvent::START_COMMAND};
  ExpectTrace(harness.Trace(), expected_trace,
              "transaction order must be STOP, SwitchProfile, START");

  harness.PublishEvent(AckFor(start), 22001U);
  const size_t command_count = harness.Commands().commands.size();
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "requesting the active profile must be a no-op success");
  Expect(harness.Commands().commands.size() == command_count &&
             harness.CameraUnderTest().SwitchCount() == 1U,
         "same-profile public request must not publish or switch");
  Pass("profile_switch_order");
}

void TestSingleProfile()
{
  Harness harness(Sync::SyncMode::TRIGGER, SwitchBehavior::SUCCEED, false, 1U);
  CompleteStartup(harness);
  harness.ClearCommandTrace();

  Expect(harness.SyncUnderTest().Profiles().size() == 1U &&
             harness.SyncUnderTest().ActiveProfile() == Camera::ProfileId::WIDE,
         "single-profile camera must expose only WIDE");
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::WIDE) ==
             LibXR::ErrorCode::OK,
         "requesting the only profile must be a no-op success");
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::NOT_SUPPORT,
         "single-profile camera must reject NARROW");
  Expect(harness.Commands().commands.empty() &&
             harness.CameraUnderTest().SwitchCount() == 0U,
         "single-profile runtime requests must not publish or switch");
  Pass("single_profile");
}

void TestStartWindow()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  Expect(harness.Commands().commands.size() == 1U, "startup must begin with STOP");
  const SyncCommand stop = harness.Commands().commands.front();
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::WIDE) ==
             LibXR::ErrorCode::STATE_ERR,
         "current-profile request must not report success while startup is stopped");
  Expect(harness.Commands().commands.size() == 1U,
         "rejected startup request must not publish another command");

  harness.PublishFrame(1000U, MakeWideGeometry());
  Expect(harness.CameraUnderTest().AvailableImageSlots() == Camera::image_slot_count,
         "WAIT_STOP_ACK image must release immediately");
  harness.PublishEvent(AckFor(stop), kStartupStopAckUs);
  harness.PublishFrame(2000U, MakeWideGeometry());
  Expect(harness.CameraUnderTest().AvailableImageSlots() == Camera::image_slot_count,
         "SETTLING image must release immediately");

  harness.PublishImu(kStartupStopAckUs + kSettleUs);
  Expect(harness.Commands().commands.size() == 2U,
         "settle completion must publish START");
  const SyncCommand start = harness.Commands().commands.back();
  harness.PublishFrame(3000U, MakeWideGeometry());
  Expect(harness.CameraUnderTest().AvailableImageSlots() == Camera::image_slot_count,
         "WAIT_START_ACK image must release immediately");
  Expect(harness.SyncedFrames().frames.empty(), "pre-START-ACK images must not publish");

  harness.PublishEvent(AckFor(start), 11001U);
  PublishEdge(harness, start, 1U, 21000U);
  harness.PublishFrame(4000U, MakeWideGeometry());
  Expect(harness.SyncedFrames().frames.size() == 1U,
         "post-START first edge sequence one must publish");
  Pass("start_window");
}

void TestAckFilter()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  const SyncCommand stop = harness.Commands().commands.front();

  SyncEvent wrong = AckFor(stop);
  wrong.operation = Operation::START_TRIGGER;
  harness.PublishEvent(wrong, 500U);
  wrong = AckFor(stop);
  ++wrong.seq;
  if (wrong.seq == 0U)
  {
    ++wrong.seq;
  }
  harness.PublishEvent(wrong, 600U);
  harness.PublishEvent(
      FrameTriggerFor(SyncCommand{.operation = Operation::START_TRIGGER,
                                  .active_level = 1U,
                                  .seq = stop.seq,
                                  .reserved = 0U,
                                  .trigger_period_us = kProfiles[0].trigger_period_us},
                      1U),
      700U);
  Expect(harness.Commands().commands.size() == 1U,
         "wrong STOP operation/seq and early edge must be ignored");

  const SyncEvent stop_ack = AckFor(stop, 9U);
  harness.PublishEvent(stop_ack, kStartupStopAckUs);
  harness.PublishEvent(stop_ack, kStartupStopAckUs + 1U);
  harness.PublishImu(kStartupStopAckUs + kSettleUs);
  Expect(harness.Commands().commands.size() == 2U,
         "valid STOP ACK must settle and publish one START");
  const SyncCommand start = harness.Commands().commands.back();

  wrong = AckFor(start);
  wrong.operation = Operation::STOP_TRIGGER;
  harness.PublishEvent(wrong, 11000U);
  wrong = AckFor(start);
  ++wrong.seq;
  if (wrong.seq == 0U)
  {
    ++wrong.seq;
  }
  harness.PublishEvent(wrong, 11000U);
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::STATE_ERR,
         "wrong START operation/seq must leave the transaction busy");

  const SyncEvent start_ack = AckFor(start);
  harness.PublishEvent(start_ack, 11001U);
  harness.PublishEvent(start_ack, 11002U);
  PublishEdge(harness, start, 1U, 21000U);
  harness.PublishFrame(100000U, MakeWideGeometry());
  Expect(harness.Commands().commands.size() == 2U &&
             harness.SyncedFrames().frames.size() == 1U,
         "valid and duplicate START ACKs must have exactly one effect");
  Pass("ack_filter");
}

void TestMalformedStopAck()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  const SyncCommand stop = harness.Commands().commands.front();
  SyncEvent malformed = AckFor(stop);
  malformed.reserved = 1U;
  harness.PublishEvent(malformed, kStartupStopAckUs);
  harness.PublishEvent(AckFor(stop), kStartupStopAckUs + 1U);

  Expect(harness.Commands().commands.size() == 1U &&
             harness.CameraUnderTest().SwitchCount() == 0U,
         "malformed matching STOP ACK must fail closed without START");
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::STATE_ERR,
         "malformed matching STOP ACK must enter FAILED");
  Pass("malformed_stop_ack");
}

void TestMalformedStartAck()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  const SyncCommand stop = harness.Commands().commands.front();
  harness.PublishEvent(AckFor(stop), kStartupStopAckUs);
  harness.PublishImu(kStartupStopAckUs + kSettleUs);
  Expect(harness.Commands().commands.size() == 2U,
         "malformed START test must reach WAIT_START_ACK");
  const SyncCommand start = harness.Commands().commands.back();

  SyncEvent malformed = AckFor(start);
  malformed.trigger_sequence = 1U;
  harness.PublishEvent(malformed, 11001U);
  harness.PublishEvent(AckFor(start), 11002U);

  Expect(harness.Commands().commands.size() == 2U,
         "malformed matching START ACK must not emit another command");
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::STATE_ERR,
         "malformed matching START ACK must enter FAILED");
  Pass("malformed_start_ack");
}

void TestRetry()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  CompleteStartup(harness);
  harness.ClearCommandTrace();

  const uint64_t stop_base = harness.LastImuTimestamp();
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "retry test must admit NARROW");
  const SyncCommand stop = harness.Commands().commands.front();
  harness.PublishGyroOnly(stop_base + 99999U);
  Expect(harness.Commands().commands.size() == 1U, "STOP must not retry before 100 ms");
  harness.PublishGyroOnly(stop_base + 100000U);
  Expect(harness.Commands().commands.size() == 2U, "STOP must retry at 100 ms");
  ExpectSameCommand(harness.Commands().commands[1], stop,
                    "STOP retry must preserve the complete command");
  harness.PublishAcclAndQuat(stop_base + 100000U);
  Expect(harness.Commands().commands.size() == 2U,
         "accl/quat at the same timestamp must not retry again");

  const uint64_t stop_ack_us = stop_base + 100001U;
  harness.PublishEvent(AckFor(stop), stop_ack_us);
  harness.PublishGyroOnly(stop_ack_us + kSettleUs - 1U);
  Expect(harness.CameraUnderTest().SwitchCount() == 0U,
         "retried STOP must still honor settle");
  harness.PublishGyroOnly(stop_ack_us + kSettleUs);
  Expect(harness.Commands().commands.size() == 3U &&
             harness.CameraUnderTest().SwitchCount() == 1U,
         "retried STOP ACK must switch once and publish START");
  const SyncCommand start = harness.Commands().commands[2];

  const uint64_t start_base = harness.LastImuTimestamp();
  harness.PublishGyroOnly(start_base + 99999U);
  Expect(harness.Commands().commands.size() == 3U, "START must not retry before 100 ms");
  harness.PublishGyroOnly(start_base + 100000U);
  Expect(harness.Commands().commands.size() == 4U, "START must retry at 100 ms");
  ExpectSameCommand(harness.Commands().commands[3], start,
                    "START retry must preserve the complete command");
  harness.PublishAcclAndQuat(start_base + 100000U);
  Expect(harness.Commands().commands.size() == 4U,
         "one raw timestamp must produce at most one START retry");

  harness.PublishEvent(AckFor(start), start_base + 100001U);
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "START ACK after retry must return to RUNNING");
  Expect(harness.CameraUnderTest().SwitchCount() == 1U,
         "START retry must not repeat camera switching");
  Pass("retry");
}

void TestRetryExhausted()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  CompleteStartup(harness);
  harness.ClearCommandTrace();

  const uint64_t base = harness.LastImuTimestamp();
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "retry exhaustion test must admit NARROW");
  const SyncCommand stop = harness.Commands().commands.front();
  for (uint64_t retry = 1U; retry <= 4U; ++retry)
  {
    harness.PublishGyroOnly(base + retry * 100000U);
  }

  Expect(harness.Commands().commands.size() == 4U,
         "retry exhaustion must send the initial command plus three retries");
  for (size_t index = 1U; index < harness.Commands().commands.size(); ++index)
  {
    ExpectSameCommand(harness.Commands().commands[index], stop,
                      "bounded retry must preserve every STOP field");
  }
  Expect(harness.CameraUnderTest().SwitchCount() == 0U &&
             harness.SyncUnderTest().ActiveProfile() == Camera::ProfileId::WIDE,
         "retry exhaustion must not switch the camera");
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::STATE_ERR,
         "retry exhaustion must enter FAILED");
  Pass("retry_exhausted");
}

void TestTimestampEpochRunning()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  const StartupCommands startup = CompleteStartup(harness);

  PublishEdge(harness, startup.start, 1U, 21000U);
  harness.PublishFrame(100000U, MakeWideGeometry());
  Expect(harness.SyncedFrames().frames.size() == 1U,
         "RUNNING epoch test must publish the old-epoch frame");
  PublishEdge(harness, startup.start, 2U, 31000U);
  harness.ClearCommandTrace();

  harness.PublishGyroOnly(100U);
  Expect(harness.Commands().commands.size() == 1U,
         "RUNNING timestamp rollback must restart with STOP");
  const SyncCommand stop = harness.Commands().commands.front();
  ExpectCommand(stop, Operation::STOP_TRIGGER, 0U,
                "RUNNING timestamp rollback STOP fields are wrong");
  harness.PublishEvent(AckFor(stop), 101U);
  harness.PublishGyroOnly(10100U);
  Expect(harness.Commands().commands.size() == 1U,
         "new epoch must still honor the complete settle interval");
  harness.PublishGyroOnly(10101U);
  Expect(harness.Commands().commands.size() == 2U,
         "new epoch settle deadline must publish START");
  const SyncCommand start = harness.Commands().commands.back();
  harness.PublishEvent(AckFor(start), 10102U);

  harness.PublishFrame(1000U, MakeWideGeometry());
  Expect(harness.SyncedFrames().frames.size() == 1U,
         "old-epoch trigger queue must be cleared on rollback");
  PublishEdge(harness, start, 1U, 10103U);
  Expect(harness.SyncedFrames().frames.size() == 2U &&
             static_cast<uint64_t>(
                 harness.SyncedFrames().frames.back().imu.timestamp_us) == 10103U,
         "new epoch must publish timestamps below the previous epoch");
  Pass("timestamp_epoch_running");
}

void TestTimestampEpochWaitStopAck()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  CompleteStartup(harness);
  harness.ClearCommandTrace();

  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "WAIT_STOP_ACK epoch test must admit NARROW");
  const SyncCommand stop = harness.Commands().commands.front();
  CameraSyncDetail::StateMachine mcu(kProfiles[0].trigger_period_us);
  mcu.OnCommand(stop);
  const auto completed = mcu.OnImu(5000000U);
  Expect(completed.event_count == 1U, "MCU must execute STOP before clock rollback");
  harness.PublishGyroOnly(100U);
  Expect(harness.Commands().commands.size() == 2U,
         "rollback must send a fresh STOP without waiting for old uptime");
  const SyncCommand fresh_stop = harness.Commands().commands.back();
  Expect(fresh_stop.seq != stop.seq, "recovery STOP must retire the old sequence");
  const auto cached = mcu.OnCommand(stop);
  Expect(cached.event_count == 1U && cached.events[0].timestamp_us == 5000000U,
         "real MCU duplicate must return the previous epoch's cached timestamp");
  harness.PublishEvent(cached.events[0].event, cached.events[0].timestamp_us);
  harness.PublishGyroOnly(101U);
  Expect(harness.Commands().commands.size() == 2U &&
             harness.CameraUnderTest().SwitchCount() == 0U,
         "retired STOP ACK must not advance recovery");
  mcu.OnCommand(fresh_stop);
  const auto fresh = mcu.OnImu(102U);
  Expect(fresh.event_count == 1U && fresh.events[0].timestamp_us == 102U,
         "new STOP must obtain a new-epoch MCU confirmation");
  harness.PublishEvent(fresh.events[0].event, fresh.events[0].timestamp_us);
  harness.PublishGyroOnly(10102U);
  const SyncCommand start = harness.Commands().commands.back();
  ExpectCommand(start, Operation::START_TRIGGER, kProfiles[1].trigger_period_us,
                "WAIT_STOP_ACK epoch recovery START fields are wrong");
  harness.PublishEvent(AckFor(start), 10103U);
  Expect(harness.SyncUnderTest().ActiveProfile() == Camera::ProfileId::NARROW,
         "WAIT_STOP_ACK must recover after a timestamp rollback");
  Pass("timestamp_epoch_wait_stop_ack");
}

void TestTimestampEpochSettling()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  CompleteStartup(harness);
  harness.ClearCommandTrace();

  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "SETTLING epoch test must admit NARROW");
  const SyncCommand stop = harness.Commands().commands.front();
  harness.PublishEvent(AckFor(stop), 500000U);
  harness.PublishGyroOnly(509999U);
  Expect(harness.CameraUnderTest().SwitchCount() == 0U,
         "SETTLING setup must remain before the old deadline");

  harness.PublishGyroOnly(100U);
  Expect(harness.Commands().commands.size() == 2U,
         "settling rollback must request fresh STOP confirmation");
  const SyncCommand fresh_stop = harness.Commands().commands.back();
  Expect(fresh_stop.seq != stop.seq, "settling rollback must use a fresh sequence");
  harness.PublishGyroOnly(20000U);
  Expect(harness.CameraUnderTest().SwitchCount() == 0U,
         "settle time alone cannot replace a fresh STOP ACK");
  harness.PublishEvent(AckFor(fresh_stop), 20001U);
  harness.PublishGyroOnly(30000U);
  Expect(harness.CameraUnderTest().SwitchCount() == 0U,
         "fresh STOP requires the full settling interval");
  harness.PublishGyroOnly(30001U);
  Expect(harness.CameraUnderTest().SwitchCount() == 1U &&
             harness.Commands().commands.size() == 3U,
         "SETTLING must switch and publish START at the new epoch deadline");
  const SyncCommand start = harness.Commands().commands.back();
  harness.PublishEvent(AckFor(start), 30002U);
  Expect(harness.SyncUnderTest().ActiveProfile() == Camera::ProfileId::NARROW,
         "SETTLING must recover after a timestamp rollback");
  Pass("timestamp_epoch_settling");
}

void TestTimestampEpochWaitStartAck()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  CompleteStartup(harness);
  harness.ClearCommandTrace();

  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "WAIT_START_ACK epoch test must admit NARROW");
  const SyncCommand stop = harness.Commands().commands.front();
  harness.PublishEvent(AckFor(stop), 12000U);
  harness.PublishGyroOnly(22000U);
  const SyncCommand start = harness.Commands().commands.back();
  harness.PublishGyroOnly(100U);
  Expect(harness.Commands().commands.size() == 3U,
         "interrupted START must recover through STOP");
  const SyncCommand fresh_stop = harness.Commands().commands.back();
  ExpectCommand(fresh_stop, Operation::STOP_TRIGGER, 0U, "recovery must first STOP");
  harness.PublishEvent(AckFor(start), 22001U);
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::STATE_ERR,
         "retired START ACK must not enter RUNNING");
  harness.PublishEvent(AckFor(fresh_stop), 101U);
  harness.PublishGyroOnly(10101U);
  const SyncCommand fresh_start = harness.Commands().commands.back();
  Expect(fresh_start.seq != start.seq, "recovery must retire old START sequence");
  harness.PublishEvent(AckFor(fresh_start), 10102U);
  Expect(harness.SyncUnderTest().ActiveProfile() == Camera::ProfileId::NARROW &&
             harness.CameraUnderTest().SwitchCount() == 1U,
         "WAIT_START_ACK must recover without repeating the camera switch");
  Pass("timestamp_epoch_wait_start_ack");
}

void TestTimestampEpochBudget()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  CompleteStartup(harness);
  harness.ClearCommandTrace();
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "budget test must admit NARROW");
  const auto base = harness.LastImuTimestamp();
  harness.PublishGyroOnly(base + 100000U);
  harness.PublishGyroOnly(base + 200000U);
  Expect(harness.Commands().commands.size() == 3U, "setup must consume two retries");
  harness.PublishGyroOnly(100U);
  Expect(harness.Commands().commands.size() == 4U, "fresh STOP consumes the last retry");
  harness.PublishGyroOnly(99U);
  harness.PublishGyroOnly(98U);
  harness.PublishGyroOnly(1000000U);
  Expect(harness.Commands().commands.size() == 4U,
         "repeated rollback cannot replenish exhausted recovery attempts");
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::STATE_ERR,
         "exhaustion must remain FAILED");
  Pass("timestamp_epoch_budget");
}

void TestTimestampEpochSwitching()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  CompleteStartup(harness);
  harness.ClearCommandTrace();
  harness.CameraUnderTest().SetSwitchHook(
      [&harness]()
      {
        harness.PublishGyroOnly(100U);
        const auto stop = harness.Commands().commands.back();
        ExpectCommand(stop, Operation::STOP_TRIGGER, 0U, "in-flight reset requires STOP");
        harness.PublishEvent(AckFor(stop), 101U);
        harness.PublishGyroOnly(10101U);
        Expect(harness.CameraUnderTest().SwitchCount() == 1U &&
                   harness.Commands().commands.size() == 2U,
               "settling cannot reenter an outstanding camera call or send START");
      });
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "switching test must admit NARROW");
  harness.PublishEvent(AckFor(harness.Commands().commands.front()), 12000U);
  harness.PublishGyroOnly(22000U);
  Expect(harness.CameraUnderTest().SwitchCount() == 1U &&
             harness.Commands().commands.size() == 3U,
         "completed camera call may START only after fresh STOP and settling");
  const auto start = harness.Commands().commands.back();
  ExpectCommand(start, Operation::START_TRIGGER, kProfiles[1].trigger_period_us,
                "recovery START must use validated target period");
  harness.PublishEvent(AckFor(start), 10102U);
  Pass("timestamp_epoch_switching");
}

void TestDuplicateTimestamp()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  CompleteStartup(harness);
  harness.ClearCommandTrace();
  harness.PublishGyroOnly(harness.LastImuTimestamp());
  Expect(harness.Commands().commands.empty(), "duplicate timestamp is not a rollback");
  Pass("duplicate_timestamp");
}

void TestSwitchFailure()
{
  Harness harness(Sync::SyncMode::TRIGGER, SwitchBehavior::FAIL);
  CompleteStartup(harness);
  harness.ClearCommandTrace();

  harness.PublishFrame(100000U, MakeWideGeometry());
  Expect(harness.CameraUnderTest().AvailableImageSlots() == Camera::image_slot_count - 1U,
         "RUNNING image without trigger must be retained");
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "failing switch must first admit STOP");
  Expect(harness.CameraUnderTest().AvailableImageSlots() == Camera::image_slot_count,
         "profile restart must release CFS pending images");

  const SyncCommand stop = harness.Commands().commands.front();
  harness.PublishEvent(AckFor(stop), 12000U);
  harness.PublishImu(22000U);

  Expect(harness.CameraUnderTest().SwitchCount() == 1U,
         "settle completion must call the failing SwitchProfile once");
  Expect(harness.CameraUnderTest().DriverProfile() == Camera::ProfileId::WIDE &&
             harness.SyncUnderTest().ActiveProfile() == Camera::ProfileId::WIDE,
         "failed switch must preserve the old reported profile");
  Expect(harness.Commands().commands.size() == 1U,
         "failed switch must leave the MCU stopped without START");
  const size_t command_count = harness.Commands().commands.size();
  const size_t switch_count = harness.CameraUnderTest().SwitchCount();
  const LibXR::ErrorCode current_result =
      harness.SyncUnderTest().RequestProfile(Camera::ProfileId::WIDE);
  const LibXR::ErrorCode different_result =
      harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW);
  Expect(current_result == LibXR::ErrorCode::STATE_ERR &&
             different_result == LibXR::ErrorCode::STATE_ERR,
         "failed switch must reject current- and different-profile requests");
  Expect(harness.Commands().commands.size() == command_count &&
             harness.CameraUnderTest().SwitchCount() == switch_count,
         "FAILED profile requests must leave the MCU stopped and camera unchanged");
  Pass("switch_failure");
}

void TestOwnership()
{
  Harness harness(Sync::SyncMode::TRIGGER, SwitchBehavior::SUCCEED, true);
  CompleteStartup(harness);
  harness.ClearCommandTrace();

  harness.PublishFrame(100000U, MakeWideGeometry());
  Expect(harness.Images().frames.size() == 1U && harness.Images().frames.front().Valid(),
         "raw image callback must retain the old WIDE owner");
  const Camera::ImageFrame* old_frame = harness.Images().frames.front().Get();

  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "ownership test must admit NARROW");
  const SyncCommand stop = harness.Commands().commands.front();
  harness.PublishEvent(AckFor(stop), 12000U);
  harness.PublishImu(22000U);
  const SyncCommand start = harness.Commands().commands.back();
  harness.PublishEvent(AckFor(start), 22001U);

  Expect(harness.Images().frames.front().Valid() &&
             harness.Images().frames.front().Get() == old_frame &&
             CameraTypes::SameFrameGeometry(old_frame->geometry, MakeWideGeometry()),
         "old WIDE owner must survive the completed NARROW switch");

  PublishEdge(harness, start, 1U, 27001U);
  harness.PublishFrame(5000U, MakeNarrowGeometry());
  Expect(harness.SyncedFrames().frames.size() == 1U &&
             CameraTypes::SameFrameGeometry(
                 harness.SyncedFrames().frames.front().GetImageFrame()->geometry,
                 MakeNarrowGeometry()) &&
             harness.Images().frames.front().Valid() &&
             harness.Images().frames.front().Get() == old_frame &&
             harness.CameraUnderTest().AvailableImageSlots() == 0U,
         "two-slot pool must retain one old raw owner and one new synced owner");

  harness.Images().frames.front().Reset();
  Expect(harness.CameraUnderTest().AvailableImageSlots() == 1U,
         "releasing the old raw owner must reopen one image slot");

  PublishEdge(harness, start, 2U, 32001U);
  harness.PublishFrame(10000U, MakeWideGeometry());

  Expect(harness.SyncedFrames().frames.size() == 2U,
         "releasing the old owner must allow the next WIDE frame to publish");
  Expect(CameraTypes::SameFrameGeometry(
             harness.SyncedFrames().frames[0].GetImageFrame()->geometry,
             MakeNarrowGeometry()) &&
             CameraTypes::SameFrameGeometry(
                 harness.SyncedFrames().frames[1].GetImageFrame()->geometry,
                 MakeWideGeometry()),
         "CFS must validate each frame, not the active profile identity");
  Expect(harness.CameraUnderTest().SwitchCount() == 1U,
         "retained old owner must not delay SwitchProfile");
  Pass("ownership");
}

void TestClockDomains()
{
  Harness harness(Sync::SyncMode::TRIGGER);
  const StartupCommands startup = CompleteStartup(harness);

  PublishEdge(harness, startup.start, 1U, 21000U);
  harness.PublishFrame(100000U, MakeWideGeometry());
  harness.ClearCommandTrace();

  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "clock-domain test must admit NARROW");
  const SyncCommand stop = harness.Commands().commands.front();
  harness.PublishEvent(AckFor(stop), 22000U);
  harness.PublishImu(32000U);
  const SyncCommand start = harness.Commands().commands.back();
  harness.PublishEvent(AckFor(start), 32001U);

  PublishEdge(harness, start, 1U, 37001U);
  harness.PublishFrame(1000U, MakeNarrowGeometry());

  Expect(harness.SyncedFrames().frames.size() == 2U,
         "fresh trigger time must allow a reset camera clock");
  Expect(static_cast<uint64_t>(
             harness.SyncedFrames().frames[0].GetImageFrame()->timestamp_us) == 100000U &&
             static_cast<uint64_t>(
                 harness.SyncedFrames().frames[1].GetImageFrame()->timestamp_us) == 1000U,
         "camera timestamps may restart across a synchronization transaction");
  Expect(static_cast<uint64_t>(harness.SyncedFrames().frames[0].imu.timestamp_us) ==
                 21000U &&
             static_cast<uint64_t>(harness.SyncedFrames().frames[1].imu.timestamp_us) ==
                 37001U,
         "authoritative trigger timestamps must stay strictly increasing");
  Pass("clock_domains");
}

void TestLatestMode()
{
  Harness harness(Sync::SyncMode::LATEST_IMU);
  Expect(harness.Commands().commands.empty(),
         "LATEST_IMU construction must not control CameraSync");
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::WIDE) ==
             LibXR::ErrorCode::OK,
         "LATEST_IMU current profile must remain a no-op");
  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::NOT_SUPPORT,
         "LATEST_IMU must reject a profile change");
  Expect(harness.SyncUnderTest().RequestProfile(static_cast<Camera::ProfileId>(255U)) ==
             LibXR::ErrorCode::NOT_SUPPORT,
         "unknown profile id must return NOT_SUPPORT");

  harness.PublishImu(1000U);
  harness.PublishFrame(50000U, MakeWideGeometry());
  harness.PublishImu(2000U);
  harness.PublishFrame(60000U, MakeWideGeometry());
  Expect(harness.SyncedFrames().frames.size() == 2U &&
             static_cast<uint64_t>(harness.SyncedFrames().frames[0].imu.timestamp_us) ==
                 1000U &&
             static_cast<uint64_t>(harness.SyncedFrames().frames[1].imu.timestamp_us) ==
                 2000U,
         "LATEST_IMU must publish one image per fresh quaternion sample");
  Expect(harness.CameraUnderTest().SwitchCount() == 0U,
         "LATEST_IMU must never call SwitchProfile");
  Pass("latest_mode");
}

void TestLatestModeIgnoresUnusedImu()
{
  Harness harness(Sync::SyncMode::LATEST_IMU);
  for (uint64_t timestamp_us = 1U; timestamp_us <= 1024U; ++timestamp_us)
  {
    harness.PublishGyroOnly(timestamp_us);
  }
  harness.PublishQuatOnly(2000U);
  harness.PublishGyroOnly(2001U);
  harness.PublishFrame(60000U, MakeWideGeometry());

  Expect(harness.SyncedFrames().frames.size() == 1U &&
             static_cast<uint64_t>(harness.SyncedFrames().frames.front().imu.timestamp_us) ==
                 2000U,
         "LATEST_IMU must retain quaternion history when gyro input is continuous");
  Pass("latest_mode_ignores_unused_imu");
}

void TestRuntimeParamCompat()
{
  constexpr Sync::RuntimeParam current{Sync::SyncMode::TRIGGER,
                                       17,
                                       kDomainName,
                                       kCommandTopicName,
                                       kResultTopicName,
                                       1U,
                                       kSettleUs,
                                       Sync::RawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP,
                                       "current_quat",
                                       kSyncedTopicName};
  constexpr Sync::RuntimeParam legacy{Sync::SyncMode::TRIGGER,
                                      23,
                                      kDomainName,
                                      kCommandTopicName,
                                      kResultTopicName,
                                      3U,
                                      1U,
                                      100.0F,
                                      Sync::RawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP,
                                      "legacy_quat"};
  static_assert(!current.legacy_timing_provided && current.offset_us == 17);
  static_assert(current.raw_quat_topic_name == "current_quat");
  static_assert(legacy.legacy_timing_provided && legacy.offset_us == 23);
  static_assert(legacy.legacy_sync_probe_div == 3U);
  static_assert(legacy.raw_quat_topic_name == "legacy_quat");
  Expect(current.LegacyTimingMatchesProfile(5000U), "current config owns no legacy rate");
  Expect(legacy.LegacyTimingMatchesProfile(10000U), "100 Hz matches 10000 us");
  Expect(!legacy.LegacyTimingMatchesProfile(5000U),
         "legacy rate must not override profile");
  auto replay = legacy;
  replay.mode = Sync::SyncMode::LATEST_IMU;
  replay.legacy_target_trigger_hz = 50.0F;
  Expect(replay.LegacyTimingMatchesProfile(10000U),
         "replay metadata must not start triggers");
  auto invalid = legacy;
  invalid.legacy_sync_probe_div = 4U;
  Expect(!invalid.LegacyTimingMatchesProfile(10000U), "invalid legacy division rejected");
  invalid = legacy;
  invalid.legacy_target_trigger_hz = std::numeric_limits<float>::quiet_NaN();
  Expect(!invalid.LegacyTimingMatchesProfile(10000U), "nonfinite legacy rate rejected");
  Pass("runtime_param_compat");
}

void TestDeferredDispatchRetry(bool rollback = false)
{
  Harness harness(Sync::SyncMode::TRIGGER);
  const StartupCommands startup = CompleteStartup(harness);
  harness.ClearCommandTrace();
  harness.EnableSyncedBlock();

  std::thread publisher(
      [&harness, start = startup.start]()
      {
        PublishEdge(harness, start, 1U, 21000U);
        harness.PublishFrame(100000U, MakeWideGeometry());
      });
  harness.WaitForSyncedBlock();

  Expect(harness.SyncUnderTest().RequestProfile(Camera::ProfileId::NARROW) ==
             LibXR::ErrorCode::OK,
         "blocked synced callback must not hold the state mutex");
  Expect(harness.Commands().commands.empty(),
         "STOP must wait behind the active outbound dispatcher");
  for (uint64_t timestamp_us = 121000U; timestamp_us <= 421000U; timestamp_us += 100000U)
  {
    harness.PublishGyroOnly(timestamp_us);
  }
  Expect(harness.Commands().commands.empty(),
         "undispatched STOP must not consume retries");
  if (rollback)
  {
    for (uint64_t timestamp_us : {500U, 400U, 300U, 200U})
    {
      harness.PublishGyroOnly(timestamp_us);
    }
    Expect(harness.Commands().commands.empty(),
           "retired queued commands must not be published by another dispatcher");
  }

  harness.ReleaseSyncedBlock();
  publisher.join();
  Expect(harness.Commands().commands.size() == 1U &&
             harness.Commands().commands.front().operation == Operation::STOP_TRIGGER,
         "releasing the publisher must dispatch exactly one STOP");

  const SyncCommand stop = harness.Commands().commands.front();
  const uint64_t stop_ack_us = rollback ? 201U : 422000U;
  harness.PublishEvent(AckFor(stop), stop_ack_us);
  harness.PublishGyroOnly(stop_ack_us + kSettleUs);
  Expect(harness.Commands().commands.size() == 2U &&
             harness.Commands().commands.back().operation == Operation::START_TRIGGER,
         "deferred STOP ACK must switch and dispatch START");
  harness.PublishEvent(AckFor(harness.Commands().commands.back()),
                       stop_ack_us + kSettleUs + 1U);
  Expect(harness.SyncUnderTest().ActiveProfile() == Camera::ProfileId::NARROW,
         "deferred transaction must finish on NARROW");
  Pass(rollback ? "deferred_dispatch_epoch" : "deferred_dispatch_retry");
}
}  // namespace

int main(int argc, char** argv)
{
  LibXR::PlatformInit();
  if (argc != 2)
  {
    Fail("usage: camera_frame_sync_profile_switch_test CASE");
  }

  const std::string_view test_case(argv[1]);
  if (test_case == "startup_first_trigger")
  {
    TestStartupFirstTrigger();
  }
  if (test_case == "trigger_stride")
  {
    TestTriggerStride();
  }
  if (test_case == "invalid_gap_resync")
  {
    TestInvalidGapResync();
  }
  if (test_case == "trigger_stream_resync")
  {
    TestTriggerStreamResync();
  }
  if (test_case == "profile_switch_order")
  {
    TestProfileSwitchOrder();
  }
  if (test_case == "single_profile")
  {
    TestSingleProfile();
  }
  if (test_case == "start_window")
  {
    TestStartWindow();
  }
  if (test_case == "ack_filter")
  {
    TestAckFilter();
  }
  if (test_case == "malformed_stop_ack")
  {
    TestMalformedStopAck();
  }
  if (test_case == "malformed_start_ack")
  {
    TestMalformedStartAck();
  }
  if (test_case == "retry")
  {
    TestRetry();
  }
  if (test_case == "retry_exhausted")
  {
    TestRetryExhausted();
  }
  if (test_case == "timestamp_epoch_running")
  {
    TestTimestampEpochRunning();
  }
  if (test_case == "timestamp_epoch_wait_stop_ack")
  {
    TestTimestampEpochWaitStopAck();
  }
  if (test_case == "timestamp_epoch_settling")
  {
    TestTimestampEpochSettling();
  }
  if (test_case == "timestamp_epoch_wait_start_ack")
  {
    TestTimestampEpochWaitStartAck();
  }
  if (test_case == "switch_failure")
  {
    TestSwitchFailure();
  }
  if (test_case == "timestamp_epoch_budget")
  {
    TestTimestampEpochBudget();
  }
  if (test_case == "timestamp_epoch_switching")
  {
    TestTimestampEpochSwitching();
  }
  if (test_case == "duplicate_timestamp")
  {
    TestDuplicateTimestamp();
  }
  if (test_case == "ownership")
  {
    TestOwnership();
  }
  if (test_case == "clock_domains")
  {
    TestClockDomains();
  }
  if (test_case == "latest_mode")
  {
    TestLatestMode();
  }
  if (test_case == "latest_mode_ignores_unused_imu")
  {
    TestLatestModeIgnoresUnusedImu();
  }
  if (test_case == "runtime_param_compat")
  {
    TestRuntimeParamCompat();
  }
  if (test_case == "deferred_dispatch_retry")
  {
    TestDeferredDispatchRetry();
  }
  if (test_case == "deferred_dispatch_epoch")
  {
    TestDeferredDispatchRetry(true);
  }
  Fail("unknown profile-switch test case");
}
