#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "../camera_frame_sync_core.hpp"

namespace
{
using namespace CameraFrameSyncCore;

struct TestGyro
{
  uint64_t sensor_timestamp_us{};
  uint32_t sensor_sequence{};
};

struct TestAccl
{
  uint64_t sensor_timestamp_us{};
  uint32_t sensor_sequence{};
};

struct TestQuat
{
  uint64_t sensor_timestamp_us{};
  uint32_t sensor_sequence{};
};

struct TimedTestGyro
{
  TestGyro sample{};
  uint64_t rx_time_us{};
};

struct TimedTestAccl
{
  TestAccl sample{};
  uint64_t rx_time_us{};
};

struct TimedTestQuat
{
  TestQuat sample{};
  uint64_t rx_time_us{};
};

struct TestImageEvent
{
  uint64_t sensor_timestamp_us{};
  uint32_t image_sequence{};
  uint32_t sync_cmd_id{};
};

struct TimedTestImage
{
  TestImageEvent sample{};
  uint64_t rx_time_us{};
};

struct TestImu
{
  uint64_t sensor_timestamp_us{};
  uint32_t sensor_sequence{};
  uint64_t rx_time_us{};
  uint64_t accl_timestamp_us{};
  uint64_t quat_timestamp_us{};
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

struct NormalImageReference
{
  bool valid{false};
  TimedTestImage image{};
};

class SequenceHarness
{
 public:
  template <typename T, size_t Capacity>
  using Ring = FixedRingBuffer<T, Capacity>;

  void SetOffsetUs(int32_t offset_us) { offset_us_ = offset_us; }

  void SetRelockConfirmFrames(uint32_t frames) { relock_confirm_frames_ = frames; }

  void PushGyro(uint64_t sensor_timestamp_us, uint32_t sequence, uint64_t rx_time_us)
  {
    pending_gyros_.PushBackDropOldest({{sensor_timestamp_us, sequence}, rx_time_us});
  }

  void PushAccl(uint64_t sensor_timestamp_us, uint32_t sequence, uint64_t rx_time_us)
  {
    pending_accls_.PushBackDropOldest({{sensor_timestamp_us, sequence}, rx_time_us});
  }

  void PushQuat(uint64_t sensor_timestamp_us, uint32_t sequence, uint64_t rx_time_us)
  {
    pending_quats_.PushBackDropOldest({{sensor_timestamp_us, sequence}, rx_time_us});
  }

  void PushImage(uint64_t sensor_timestamp_us, uint32_t image_sequence, uint64_t rx_time_us,
                 uint32_t sync_cmd_id = 0)
  {
    pending_images_.PushBackDropOldest(
        {{sensor_timestamp_us, image_sequence, sync_cmd_id}, rx_time_us});
  }

  void Drain()
  {
    while (TryBuildFrontImu(
        pending_gyros_, pending_accls_, pending_quats_, imu_history_,
        relation_.last_imu_sensor_period_us, relation_.last_imu_rx_period_us, kHistoryLimit,
        [](const TestGyro& gyro, const TestAccl& accl, const TestQuat& quat, uint64_t rx_time_us)
        {
          return TestImu{
              .sensor_timestamp_us = gyro.sensor_timestamp_us,
              .sensor_sequence = gyro.sensor_sequence,
              .rx_time_us = rx_time_us,
              .accl_timestamp_us = accl.sensor_timestamp_us,
              .quat_timestamp_us = quat.sensor_timestamp_us,
          };
        }))
    {
    }

    while (!pending_images_.Empty())
    {
      const TimedTestImage image = pending_images_.Front();
      ObserveNormalImagePeriod(image);

      ImageDecision decision = ImageDecision::REJECT;
      if (lock_state_.state == SyncState::LOCKING || lock_state_.state == SyncState::SYNCED)
      {
        decision = TryProcessLockedImage(image);
      }
      else if (image.sample.sync_cmd_id != 0)
      {
        decision = TryLockFromProbe(image);
      }
      else
      {
        pending_images_.PopFront();
        continue;
      }

      if (decision == ImageDecision::WAIT)
      {
        break;
      }

      pending_images_.PopFront();
      if (decision == ImageDecision::REJECT)
      {
        reject_image_sequences_.push_back(image.sample.image_sequence);
        EnterRecovering(lock_state_);
        ResetLockedRelation();
      }
    }
  }

  const std::vector<uint32_t>& PublishedImages() const { return published_images_; }

  const std::vector<uint32_t>& PublishedSyncImuSequences() const
  {
    return published_sync_imu_sequences_;
  }

  const std::vector<uint64_t>& PublishedFinalImuTimestamps() const
  {
    return published_final_imu_timestamps_;
  }

  const std::vector<uint64_t>& PublishedAcclTimestamps() const
  {
    return published_accl_timestamps_;
  }

  const std::vector<uint64_t>& PublishedQuatTimestamps() const
  {
    return published_quat_timestamps_;
  }

  const std::vector<uint32_t>& RejectImageSequences() const { return reject_image_sequences_; }

  SyncState CurrentState() const { return lock_state_.state; }

  uint32_t LockConfirmCount() const { return lock_state_.lock_confirm_count; }

 private:
  enum class ImageDecision : uint8_t
  {
    WAIT = 0,
    ACCEPT = 1,
    REJECT = 2,
  };

  static constexpr size_t kHistoryLimit = 256;

  uint32_t EstimatedStrideSamples() const
  {
    return EstimateStrideSamples(relation_.base_image_rx_period_us, relation_.last_imu_rx_period_us);
  }

  void ResetLockedRelation()
  {
    relation_.stride_samples = relation_.stride_samples != 0 ? relation_.stride_samples
                                                             : EstimatedStrideSamples();
    relation_.last_image_sequence = std::numeric_limits<uint32_t>::max();
    relation_.last_sync_imu_sequence = std::numeric_limits<uint32_t>::max();
    relation_.last_host_skew_us = 0;
  }

  static uint32_t SequenceToleranceSamples(const SyncRelation& relation)
  {
    return std::max<uint32_t>(1U, relation.stride_samples / 4U);
  }

  static uint64_t ProbeSearchWindowUs(const SyncRelation& relation)
  {
    return std::max<uint64_t>(100000ULL, relation.base_image_rx_period_us * 2ULL);
  }

  void ObserveNormalImagePeriod(const TimedTestImage& image)
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

  const TestImu* FindFinalImu(const TestImu& sync_imu) const
  {
    const uint64_t target_timestamp_us = ApplyOffsetUs(sync_imu.sensor_timestamp_us, offset_us_);
    return FindBySensorTimestamp(
        imu_history_, target_timestamp_us,
        OffsetSearchToleranceUs(relation_.last_imu_sensor_period_us, relation_.stride_samples));
  }

  bool NeedMoreImuForOffset(const TestImu& sync_imu) const
  {
    if (imu_history_.Empty())
    {
      return true;
    }

    const uint64_t target_timestamp_us = ApplyOffsetUs(sync_imu.sensor_timestamp_us, offset_us_);
    return target_timestamp_us > imu_history_.Back().sensor_timestamp_us;
  }

  void Publish(const TimedTestImage& image, const TestImu& sync_imu, const TestImu& final_imu)
  {
    published_images_.push_back(image.sample.image_sequence);
    published_sync_imu_sequences_.push_back(sync_imu.sensor_sequence);
    published_final_imu_timestamps_.push_back(final_imu.sensor_timestamp_us);
    published_accl_timestamps_.push_back(final_imu.accl_timestamp_us);
    published_quat_timestamps_.push_back(final_imu.quat_timestamp_us);
  }

  ImageDecision TryLockFromProbe(const TimedTestImage& probe)
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
    if (imu_history_.Back().sensor_sequence < doubled_gap_samples)
    {
      return ImageDecision::WAIT;
    }

    const uint64_t search_window_us = ProbeSearchWindowUs(relation_);
    const int64_t skew_tolerance_us =
        static_cast<int64_t>(SequenceSearchToleranceUs(relation_.last_imu_rx_period_us));

    const TestImu* best_sync_imu = nullptr;
    int64_t best_delta_error_us = std::numeric_limits<int64_t>::max();
    int64_t best_probe_skew_us = std::numeric_limits<int64_t>::max();

    for (size_t i = imu_history_.Size(); i > 0; --i)
    {
      const TestImu& sync_imu = imu_history_[i - 1];
      if (sync_imu.sensor_sequence < doubled_gap_samples)
      {
        break;
      }
      if (AbsDiffUs(sync_imu.rx_time_us, probe.rx_time_us) > search_window_us)
      {
        continue;
      }

      const TestImu* prev_sync_imu =
          FindBySequence(imu_history_, sync_imu.sensor_sequence - doubled_gap_samples,
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
      const int64_t delta_error_us = AbsDiffSigned(probe_skew_us, prev_skew_us);
      const int64_t abs_probe_skew_us = probe_skew_us >= 0 ? probe_skew_us : -probe_skew_us;

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

    const TestImu* final_imu = FindFinalImu(*best_sync_imu);
    if (final_imu == nullptr)
    {
      return NeedMoreImuForOffset(*best_sync_imu) ? ImageDecision::WAIT : ImageDecision::REJECT;
    }

    Publish(probe, *best_sync_imu, *final_imu);
    relation_.stride_samples = stride_samples;
    relation_.last_image_sequence = probe.sample.image_sequence;
    relation_.last_sync_imu_sequence = best_sync_imu->sensor_sequence;
    relation_.last_host_skew_us =
        static_cast<int64_t>(probe.rx_time_us) - static_cast<int64_t>(best_sync_imu->rx_time_us);
    ObserveGoodFrame(lock_state_, relock_confirm_frames_);
    return ImageDecision::ACCEPT;
  }

  ImageDecision TryProcessLockedImage(const TimedTestImage& image)
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
    uint64_t expected_sequence =
        static_cast<uint64_t>(relation_.last_sync_imu_sequence) +
        static_cast<uint64_t>(image_gap) * static_cast<uint64_t>(relation_.stride_samples);
    if (image.sample.sync_cmd_id != 0)
    {
      expected_sequence += relation_.stride_samples;
    }
    if (expected_sequence > std::numeric_limits<uint32_t>::max())
    {
      return ImageDecision::REJECT;
    }

    if (imu_history_.Empty() ||
        imu_history_.Back().sensor_sequence + SequenceToleranceSamples(relation_) <
            expected_sequence)
    {
      return ImageDecision::WAIT;
    }

    const TestImu* sync_imu = FindBySequence(imu_history_, static_cast<uint32_t>(expected_sequence),
                                             SequenceToleranceSamples(relation_));
    if (sync_imu == nullptr)
    {
      return ImageDecision::REJECT;
    }

    const int64_t host_skew_us =
        static_cast<int64_t>(image.rx_time_us) - static_cast<int64_t>(sync_imu->rx_time_us);
    const int64_t host_skew_error_us = AbsDiffSigned(host_skew_us, relation_.last_host_skew_us);
    if (host_skew_error_us >
        static_cast<int64_t>(SequenceSearchToleranceUs(relation_.last_imu_rx_period_us)))
    {
      return ImageDecision::REJECT;
    }

    const TestImu* final_imu = FindFinalImu(*sync_imu);
    if (final_imu == nullptr)
    {
      return NeedMoreImuForOffset(*sync_imu) ? ImageDecision::WAIT : ImageDecision::REJECT;
    }

    Publish(image, *sync_imu, *final_imu);
    relation_.last_image_sequence = image.sample.image_sequence;
    relation_.last_sync_imu_sequence = sync_imu->sensor_sequence;
    relation_.last_host_skew_us = host_skew_us;
    ObserveGoodFrame(lock_state_, relock_confirm_frames_);
    return ImageDecision::ACCEPT;
  }

 private:
  int32_t offset_us_{0};
  uint32_t relock_confirm_frames_{3};
  SyncLockState lock_state_{};
  SyncRelation relation_{};
  NormalImageReference last_normal_image_{};
  Ring<TimedTestGyro, kHistoryLimit> pending_gyros_{};
  Ring<TimedTestAccl, kHistoryLimit> pending_accls_{};
  Ring<TimedTestQuat, kHistoryLimit> pending_quats_{};
  Ring<TimedTestImage, kHistoryLimit> pending_images_{};
  Ring<TestImu, kHistoryLimit> imu_history_{};
  std::vector<uint32_t> published_images_{};
  std::vector<uint32_t> published_sync_imu_sequences_{};
  std::vector<uint64_t> published_final_imu_timestamps_{};
  std::vector<uint64_t> published_accl_timestamps_{};
  std::vector<uint64_t> published_quat_timestamps_{};
  std::vector<uint32_t> reject_image_sequences_{};
};

struct TestFailure : public std::exception
{
  explicit TestFailure(std::string message) : message_(std::move(message)) {}

  const char* what() const noexcept override { return message_.c_str(); }

 private:
  std::string message_;
};

template <typename T>
void ExpectEqual(const T& actual, const T& expected, const std::string& label)
{
  if (!(actual == expected))
  {
    std::ostringstream oss;
    oss << label << " mismatch";
    throw TestFailure(oss.str());
  }
}

void ExpectTrue(bool cond, const std::string& label)
{
  if (!cond)
  {
    throw TestFailure(label);
  }
}

void FeedImuTriplets(SequenceHarness& harness, uint32_t begin_sequence, uint32_t end_sequence,
                     uint64_t sensor_step_us, uint64_t rx_step_us, uint64_t rx_bias_us = 0)
{
  for (uint32_t seq = begin_sequence; seq <= end_sequence; ++seq)
  {
    const uint64_t sensor_timestamp_us = static_cast<uint64_t>(seq) * sensor_step_us;
    const uint64_t rx_time_us = static_cast<uint64_t>(seq) * rx_step_us + rx_bias_us;
    harness.PushGyro(sensor_timestamp_us, seq, rx_time_us);
    harness.PushAccl(sensor_timestamp_us, seq, rx_time_us + 20);
    harness.PushQuat(sensor_timestamp_us, seq, rx_time_us + 40);
    harness.Drain();
  }
}

void TestProbeLockAndTrack()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 24, 1000, 1000);
  harness.PushImage(4000, 0, 4120);
  harness.Drain();
  harness.PushImage(8000, 1, 8120);
  harness.Drain();
  harness.PushImage(16000, 2, 16120, 1);
  harness.Drain();
  harness.PushImage(20000, 3, 20120);
  harness.Drain();
  harness.PushImage(24000, 4, 24120);
  harness.Drain();

  ExpectEqual(harness.PublishedImages(), std::vector<uint32_t>({2, 3, 4}),
              "probe lock publish images");
  ExpectEqual(harness.PublishedSyncImuSequences(), std::vector<uint32_t>({16, 20, 24}),
              "probe lock sync imu sequences");
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "probe lock final state");
}

void TestPositiveOffsetUsesLaterImu()
{
  SequenceHarness harness;
  harness.SetOffsetUs(700);

  FeedImuTriplets(harness, 1, 20, 1000, 1000);
  harness.PushImage(4000, 0, 4120);
  harness.Drain();
  harness.PushImage(8000, 1, 8120);
  harness.Drain();
  harness.PushImage(16000, 2, 16120, 7);
  harness.Drain();

  ExpectEqual(harness.PublishedFinalImuTimestamps(), std::vector<uint64_t>({17000}),
              "positive offset final imu");
}

void TestForwardSearchUsesLaterAcclAndQuat()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 15, 1000, 1000);
  harness.PushGyro(16000, 16, 16000);
  harness.PushAccl(15900, 16, 15900);
  harness.PushAccl(16050, 16, 16050);
  harness.PushQuat(15950, 16, 15950);
  harness.PushQuat(16060, 16, 16060);
  harness.Drain();

  harness.PushImage(4000, 0, 4100);
  harness.Drain();
  harness.PushImage(8000, 1, 8100);
  harness.Drain();
  harness.PushImage(16000, 2, 16100, 3);
  harness.Drain();

  ExpectEqual(harness.PublishedSyncImuSequences(), std::vector<uint32_t>({16}),
              "forward search sync imu");
  ExpectEqual(harness.PublishedAcclTimestamps(), std::vector<uint64_t>({16050}),
              "forward search accl timestamp");
  ExpectEqual(harness.PublishedQuatTimestamps(), std::vector<uint64_t>({16060}),
              "forward search quat timestamp");
  ExpectEqual(harness.PublishedFinalImuTimestamps(), std::vector<uint64_t>({16000}),
              "forward search final imu");
}

void TestProbeSequenceGapRejects()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 20, 1000, 1000);
  harness.PushImage(4000, 0, 4120);
  harness.Drain();
  harness.PushImage(8000, 1, 8120);
  harness.Drain();
  harness.PushImage(16000, 3, 16120, 9);
  harness.Drain();

  ExpectEqual(harness.PublishedImages(), std::vector<uint32_t>({}),
              "gap reject should not publish");
  ExpectEqual(harness.RejectImageSequences(), std::vector<uint32_t>({3}),
              "gap reject image sequence");
  ExpectEqual(harness.CurrentState(), SyncState::RECOVERING, "gap reject state");
}

void TestLockedSkewJumpRejects()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 24, 1000, 1000);
  harness.PushImage(4000, 0, 4120);
  harness.Drain();
  harness.PushImage(8000, 1, 8120);
  harness.Drain();
  harness.PushImage(16000, 2, 16120, 1);
  harness.Drain();
  harness.PushImage(20000, 3, 35000);
  harness.Drain();

  ExpectEqual(harness.PublishedImages(), std::vector<uint32_t>({2}),
              "skew jump only probe image should publish");
  ExpectEqual(harness.RejectImageSequences(), std::vector<uint32_t>({3}),
              "skew jump reject sequence");
  ExpectEqual(harness.CurrentState(), SyncState::RECOVERING, "skew jump state");
}

void TestRecoveringNeedsThreeGoodFramesToRelock()
{
  SequenceHarness harness;
  harness.SetRelockConfirmFrames(3);

  FeedImuTriplets(harness, 1, 48, 1000, 1000);
  harness.PushImage(4000, 0, 4120);
  harness.Drain();
  harness.PushImage(8000, 1, 8120);
  harness.Drain();
  harness.PushImage(16000, 2, 16120, 1);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::LOCKING, "first probe should start locking");

  harness.PushImage(20000, 3, 35000);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::RECOVERING, "bad frame should recover");

  harness.PushImage(24000, 4, 24120);
  harness.Drain();
  harness.PushImage(28000, 5, 28120);
  harness.Drain();
  harness.PushImage(36000, 6, 36120, 2);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::LOCKING, "second probe lock frame 1");
  ExpectEqual(harness.LockConfirmCount(), static_cast<uint32_t>(1), "second probe lock count");

  harness.PushImage(40000, 7, 40120);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::LOCKING, "relock frame 2");
  ExpectEqual(harness.LockConfirmCount(), static_cast<uint32_t>(2), "relock count 2");

  harness.PushImage(44000, 8, 44120);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "relock frame 3 should sync");
  ExpectEqual(harness.LockConfirmCount(), static_cast<uint32_t>(3), "relock count 3");
}

using TestCase = void (*)();

struct NamedCase
{
  const char* name;
  TestCase fn;
};

}  // namespace

int main()
{
  const NamedCase cases[] = {
      {"正确/探针锁定后按固定步长跟踪", TestProbeLockAndTrack},
      {"正确/正offset在IMU域内取后继样本", TestPositiveOffsetUsesLaterImu},
      {"正确/gyro前向找后继accl与quat", TestForwardSearchUsesLaterAcclAndQuat},
      {"错误/探针前图像序列不连续", TestProbeSequenceGapRejects},
      {"错误/锁定后host skew突变", TestLockedSkewJumpRejects},
      {"错误后恢复/三帧重新锁定", TestRecoveringNeedsThreeGoodFramesToRelock},
  };

  bool all_passed = true;
  for (const NamedCase& test_case : cases)
  {
    try
    {
      test_case.fn();
      std::cout << "[PASS] " << test_case.name << '\n';
    }
    catch (const std::exception& ex)
    {
      all_passed = false;
      std::cerr << "[FAIL] " << test_case.name << ": " << ex.what() << '\n';
    }
  }

  return all_passed ? 0 : 1;
}
