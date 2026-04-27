#include <algorithm>
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
};

struct TestAccl
{
  uint64_t sensor_timestamp_us{};
};

struct TestQuat
{
  uint64_t sensor_timestamp_us{};
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
  uint32_t sensor_step_index{};
};

struct TimedTestImage
{
  TestImageEvent sample{};
  uint64_t rx_time_us{};
  uint32_t tag{};
};

struct TestImu
{
  uint64_t sensor_timestamp_us{};
  uint64_t rx_time_us{};
  uint64_t accl_timestamp_us{};
  uint64_t quat_timestamp_us{};
};

struct ImageReference
{
  bool valid{false};
  TimedTestImage image{};
};

struct SyncRelation
{
  uint64_t base_image_rx_period_us{0};
  uint64_t last_imu_sensor_period_us{0};
  uint64_t last_imu_rx_period_us{0};
  uint64_t sync_imu_sensor_period_us{0};
  uint64_t last_image_rx_time_us{0};
  uint64_t last_sync_imu_sensor_timestamp_us{0};
  int64_t last_host_skew_us{0};
};

class SequenceHarness
{
 public:
  template <typename T, size_t Capacity>
  using Ring = FixedRingBuffer<T, Capacity>;

  void SetOffsetUs(int32_t offset_us) { offset_us_ = offset_us; }

  void SetRelockConfirmFrames(uint32_t frames) { relock_confirm_frames_ = frames; }

  void ArmProbe() { probe_pending_ = true; }

  void PushGyro(uint64_t sensor_timestamp_us, uint64_t rx_time_us)
  {
    pending_gyros_.PushBackDropOldest({{sensor_timestamp_us}, rx_time_us});
  }

  void PushAccl(uint64_t sensor_timestamp_us, uint64_t rx_time_us)
  {
    pending_accls_.PushBackDropOldest({{sensor_timestamp_us}, rx_time_us});
  }

  void PushQuat(uint64_t sensor_timestamp_us, uint64_t rx_time_us)
  {
    pending_quats_.PushBackDropOldest({{sensor_timestamp_us}, rx_time_us});
  }

  void PushImage(uint64_t sensor_timestamp_us, uint32_t sensor_step_index, uint64_t rx_time_us,
                 uint32_t tag)
  {
    pending_images_.PushBackDropOldest({{sensor_timestamp_us, sensor_step_index}, rx_time_us, tag});
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
      if (!last_observed_image_.valid)
      {
        RememberObservedImage(image);
        pending_images_.PopFront();
        continue;
      }

      if (image.rx_time_us <= last_observed_image_.image.rx_time_us)
      {
        reject_tags_.push_back(image.tag);
        RememberObservedImage(image);
        pending_images_.PopFront();
        EnterRecovering(lock_state_);
        ResetLockedRelation();
        probe_pending_ = false;
        continue;
      }

      const uint64_t image_gap_rx_us = image.rx_time_us - last_observed_image_.image.rx_time_us;
      const bool probe_pending = probe_pending_;

      ImageDecision decision = ImageDecision::REJECT;
      if (probe_pending)
      {
        decision = IsProbeImageGap(image_gap_rx_us) ? TryLockFromProbe(image, image_gap_rx_us)
                                                    : ImageDecision::REJECT;
      }
      else if (lock_state_.state == SyncState::LOCKING || lock_state_.state == SyncState::SYNCED)
      {
        decision =
            IsNormalImageGap(image_gap_rx_us) ? TryProcessLockedImage(image, image_gap_rx_us)
                                              : ImageDecision::REJECT;
      }
      else
      {
        ObserveNormalImagePeriod(image, image_gap_rx_us);
        RememberObservedImage(image);
        pending_images_.PopFront();
        continue;
      }

      if (decision == ImageDecision::WAIT)
      {
        break;
      }

      pending_images_.PopFront();
      if (decision == ImageDecision::ACCEPT)
      {
        if (probe_pending)
        {
          probe_pending_ = false;
        }
        else
        {
          ObserveNormalImagePeriod(image, image_gap_rx_us);
        }
        RememberObservedImage(image);
        continue;
      }

      reject_tags_.push_back(image.tag);
      if (probe_pending)
      {
        probe_pending_ = false;
      }
      RememberObservedImage(image);
      EnterRecovering(lock_state_);
      ResetLockedRelation();
    }
  }

  const std::vector<uint32_t>& PublishedImageTags() const { return published_image_tags_; }

  const std::vector<uint64_t>& PublishedSyncImuTimestamps() const
  {
    return published_sync_imu_timestamps_;
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

  const std::vector<uint32_t>& RejectTags() const { return reject_tags_; }

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

  void ResetLockedRelation()
  {
    relation_.sync_imu_sensor_period_us = 0;
    relation_.last_image_rx_time_us = 0;
    relation_.last_sync_imu_sensor_timestamp_us = 0;
    relation_.last_host_skew_us = 0;
  }

  uint32_t EstimatedStrideSamples() const
  {
    return EstimateStrideSamples(relation_.base_image_rx_period_us, relation_.last_imu_rx_period_us);
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

  bool IsNormalImageGap(uint64_t image_gap_rx_us) const
  {
    if (relation_.base_image_rx_period_us == 0 || image_gap_rx_us == 0)
    {
      return false;
    }

    return AbsDiffUs(image_gap_rx_us, relation_.base_image_rx_period_us) <=
           ImageGapToleranceUs(relation_.base_image_rx_period_us, relation_.last_imu_rx_period_us);
  }

  bool IsProbeImageGap(uint64_t image_gap_rx_us) const
  {
    if (relation_.base_image_rx_period_us == 0 || image_gap_rx_us == 0)
    {
      return false;
    }

    const uint64_t tolerance_us =
        ImageGapToleranceUs(relation_.base_image_rx_period_us, relation_.last_imu_rx_period_us);
    return AbsDiffUs(image_gap_rx_us, relation_.base_image_rx_period_us * 2ULL) <= tolerance_us;
  }

  uint64_t HostSkewToleranceUs() const
  {
    return CameraFrameSyncCore::HostSkewToleranceUs(relation_.last_imu_rx_period_us);
  }

  uint64_t ProbeSearchWindowUs() const
  {
    return std::max<uint64_t>(100000ULL, relation_.base_image_rx_period_us * 2ULL);
  }

  void RememberObservedImage(const TimedTestImage& image)
  {
    last_observed_image_.valid = true;
    last_observed_image_.image = image;
  }

  void ObserveNormalImagePeriod(const TimedTestImage& image, uint64_t image_gap_rx_us)
  {
    if (image_gap_rx_us == 0)
    {
      return;
    }

    if (relation_.base_image_rx_period_us == 0 || IsNormalImageGap(image_gap_rx_us))
    {
      relation_.base_image_rx_period_us = image_gap_rx_us;
    }

    last_normal_image_.valid = true;
    last_normal_image_.image = image;
  }

  const TestImu* FindFinalImu(const TestImu& sync_imu) const
  {
    const uint64_t target_timestamp_us = ApplyOffsetUs(sync_imu.sensor_timestamp_us, offset_us_);
    return FindBySensorTimestamp(
        imu_history_, target_timestamp_us,
        OffsetSearchToleranceUs(relation_.last_imu_sensor_period_us, EstimatedStrideSamples()));
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
    published_image_tags_.push_back(image.tag);
    published_sync_imu_timestamps_.push_back(sync_imu.sensor_timestamp_us);
    published_final_imu_timestamps_.push_back(final_imu.sensor_timestamp_us);
    published_accl_timestamps_.push_back(final_imu.accl_timestamp_us);
    published_quat_timestamps_.push_back(final_imu.quat_timestamp_us);
  }

  ImageDecision TryLockFromProbe(const TimedTestImage& probe, uint64_t image_gap_rx_us)
  {
    if (!last_normal_image_.valid || !IsProbeImageGap(image_gap_rx_us))
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
        SyncSensorGapToleranceUs(expected_probe_sensor_gap_us, relation_.last_imu_sensor_period_us);
    const uint64_t search_window_us = ProbeSearchWindowUs();
    const uint64_t host_gap_tolerance_us = HostSkewToleranceUs();
    if (imu_history_.Back().rx_time_us + host_gap_tolerance_us < probe.rx_time_us)
    {
      return ImageDecision::WAIT;
    }

    const TestImu* best_sync_imu = nullptr;
    const TestImu* best_prev_sync_imu = nullptr;
    uint64_t best_host_gap_error_us = std::numeric_limits<uint64_t>::max();
    uint64_t best_sensor_gap_error_us = std::numeric_limits<uint64_t>::max();
    int64_t best_probe_skew_us = std::numeric_limits<int64_t>::max();

    for (size_t i = imu_history_.Size(); i > 0; --i)
    {
      const TestImu& sync_imu = imu_history_[i - 1];
      if (AbsDiffUs(sync_imu.rx_time_us, probe.rx_time_us) > search_window_us)
      {
        if (sync_imu.rx_time_us + search_window_us < probe.rx_time_us)
        {
          break;
        }
        continue;
      }
      if (sync_imu.sensor_timestamp_us < expected_probe_sensor_gap_us)
      {
        continue;
      }

      const uint64_t prev_target_sensor_timestamp_us =
          sync_imu.sensor_timestamp_us - expected_probe_sensor_gap_us;
      const TestImu* prev_sync_imu =
          FindBySensorTimestamp(imu_history_, prev_target_sensor_timestamp_us, sensor_gap_tolerance_us);
      if (prev_sync_imu == nullptr || sync_imu.rx_time_us <= prev_sync_imu->rx_time_us)
      {
        continue;
      }

      const uint64_t host_gap_error_us =
          AbsDiffUs(sync_imu.rx_time_us - prev_sync_imu->rx_time_us, image_gap_rx_us);
      if (host_gap_error_us > host_gap_tolerance_us)
      {
        continue;
      }

      const uint64_t sensor_gap_error_us = AbsDiffUs(
          sync_imu.sensor_timestamp_us - prev_sync_imu->sensor_timestamp_us,
          expected_probe_sensor_gap_us);
      const int64_t probe_skew_us =
          static_cast<int64_t>(probe.rx_time_us) - static_cast<int64_t>(sync_imu.rx_time_us);
      const int64_t abs_probe_skew_us = probe_skew_us >= 0 ? probe_skew_us : -probe_skew_us;

      if (host_gap_error_us < best_host_gap_error_us ||
          (host_gap_error_us == best_host_gap_error_us &&
           sensor_gap_error_us < best_sensor_gap_error_us) ||
          (host_gap_error_us == best_host_gap_error_us &&
           sensor_gap_error_us == best_sensor_gap_error_us &&
           abs_probe_skew_us < best_probe_skew_us))
      {
        best_sync_imu = &sync_imu;
        best_prev_sync_imu = prev_sync_imu;
        best_host_gap_error_us = host_gap_error_us;
        best_sensor_gap_error_us = sensor_gap_error_us;
        best_probe_skew_us = abs_probe_skew_us;
      }
    }

    if (best_sync_imu == nullptr || best_prev_sync_imu == nullptr)
    {
      return ImageDecision::REJECT;
    }

    const TestImu* final_imu = FindFinalImu(*best_sync_imu);
    if (final_imu == nullptr)
    {
      return NeedMoreImuForOffset(*best_sync_imu) ? ImageDecision::WAIT : ImageDecision::REJECT;
    }

    Publish(probe, *best_sync_imu, *final_imu);
    relation_.sync_imu_sensor_period_us =
        (best_sync_imu->sensor_timestamp_us - best_prev_sync_imu->sensor_timestamp_us) / 2ULL;
    relation_.last_image_rx_time_us = probe.rx_time_us;
    relation_.last_sync_imu_sensor_timestamp_us = best_sync_imu->sensor_timestamp_us;
    relation_.last_host_skew_us =
        static_cast<int64_t>(probe.rx_time_us) - static_cast<int64_t>(best_sync_imu->rx_time_us);
    ObserveGoodFrame(lock_state_, relock_confirm_frames_);
    return ImageDecision::ACCEPT;
  }

  ImageDecision TryProcessLockedImage(const TimedTestImage& image, uint64_t image_gap_rx_us)
  {
    if (relation_.last_image_rx_time_us == 0 || relation_.last_sync_imu_sensor_timestamp_us == 0)
    {
      return ImageDecision::REJECT;
    }

    if (image.rx_time_us <= relation_.last_image_rx_time_us || !IsNormalImageGap(image_gap_rx_us))
    {
      return ImageDecision::REJECT;
    }

    const uint64_t expected_sync_sensor_gap_us = EstimatedSyncImuSensorPeriodUs();
    if (expected_sync_sensor_gap_us == 0)
    {
      return ImageDecision::WAIT;
    }

    const uint64_t sensor_gap_tolerance_us =
        SyncSensorGapToleranceUs(expected_sync_sensor_gap_us, relation_.last_imu_sensor_period_us);
    const uint64_t host_skew_tolerance_us = HostSkewToleranceUs();
    const uint64_t search_window_us = ProbeSearchWindowUs();
    const int64_t expected_sync_rx_time_us =
        static_cast<int64_t>(image.rx_time_us) - relation_.last_host_skew_us;
    if (expected_sync_rx_time_us > 0 && !imu_history_.Empty() &&
        imu_history_.Back().rx_time_us + host_skew_tolerance_us <
            static_cast<uint64_t>(expected_sync_rx_time_us))
    {
      return ImageDecision::WAIT;
    }

    const TestImu* best_sync_imu = nullptr;
    uint64_t best_host_skew_error_us = std::numeric_limits<uint64_t>::max();
    uint64_t best_sensor_gap_error_us = std::numeric_limits<uint64_t>::max();

    for (size_t i = imu_history_.Size(); i > 0; --i)
    {
      const TestImu& sync_imu = imu_history_[i - 1];
      if (AbsDiffUs(sync_imu.rx_time_us, image.rx_time_us) > search_window_us)
      {
        if (sync_imu.rx_time_us + search_window_us < image.rx_time_us)
        {
          break;
        }
        continue;
      }
      if (sync_imu.sensor_timestamp_us <= relation_.last_sync_imu_sensor_timestamp_us)
      {
        continue;
      }

      const uint64_t sensor_gap_us =
          sync_imu.sensor_timestamp_us - relation_.last_sync_imu_sensor_timestamp_us;
      const uint64_t sensor_gap_error_us =
          AbsDiffUs(sensor_gap_us, expected_sync_sensor_gap_us);
      if (sensor_gap_error_us > sensor_gap_tolerance_us)
      {
        continue;
      }

      const int64_t host_skew_us =
          static_cast<int64_t>(image.rx_time_us) - static_cast<int64_t>(sync_imu.rx_time_us);
      const uint64_t host_skew_error_us = AbsDiffSigned(host_skew_us, relation_.last_host_skew_us);
      if (host_skew_error_us > host_skew_tolerance_us)
      {
        continue;
      }

      if (host_skew_error_us < best_host_skew_error_us ||
          (host_skew_error_us == best_host_skew_error_us &&
           sensor_gap_error_us < best_sensor_gap_error_us))
      {
        best_sync_imu = &sync_imu;
        best_host_skew_error_us = host_skew_error_us;
        best_sensor_gap_error_us = sensor_gap_error_us;
      }
    }

    if (best_sync_imu == nullptr)
    {
      return ImageDecision::REJECT;
    }

    const int64_t host_skew_us =
        static_cast<int64_t>(image.rx_time_us) - static_cast<int64_t>(best_sync_imu->rx_time_us);
    const TestImu* final_imu = FindFinalImu(*best_sync_imu);
    if (final_imu == nullptr)
    {
      return NeedMoreImuForOffset(*best_sync_imu) ? ImageDecision::WAIT : ImageDecision::REJECT;
    }

    Publish(image, *best_sync_imu, *final_imu);
    relation_.sync_imu_sensor_period_us =
        best_sync_imu->sensor_timestamp_us - relation_.last_sync_imu_sensor_timestamp_us;
    relation_.last_image_rx_time_us = image.rx_time_us;
    relation_.last_sync_imu_sensor_timestamp_us = best_sync_imu->sensor_timestamp_us;
    relation_.last_host_skew_us = host_skew_us;
    ObserveGoodFrame(lock_state_, relock_confirm_frames_);
    return ImageDecision::ACCEPT;
  }

 private:
  int32_t offset_us_{0};
  uint32_t relock_confirm_frames_{3};
  bool probe_pending_{false};
  SyncLockState lock_state_{};
  SyncRelation relation_{};
  ImageReference last_observed_image_{};
  ImageReference last_normal_image_{};
  Ring<TimedTestGyro, kHistoryLimit> pending_gyros_{};
  Ring<TimedTestAccl, kHistoryLimit> pending_accls_{};
  Ring<TimedTestQuat, kHistoryLimit> pending_quats_{};
  Ring<TimedTestImage, kHistoryLimit> pending_images_{};
  Ring<TestImu, kHistoryLimit> imu_history_{};
  std::vector<uint32_t> published_image_tags_{};
  std::vector<uint64_t> published_sync_imu_timestamps_{};
  std::vector<uint64_t> published_final_imu_timestamps_{};
  std::vector<uint64_t> published_accl_timestamps_{};
  std::vector<uint64_t> published_quat_timestamps_{};
  std::vector<uint32_t> reject_tags_{};
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

void FeedImuTriplets(SequenceHarness& harness, uint32_t begin_index, uint32_t end_index,
                     uint64_t sensor_step_us, uint64_t rx_step_us, uint64_t rx_bias_us = 0)
{
  for (uint32_t index = begin_index; index <= end_index; ++index)
  {
    const uint64_t sensor_timestamp_us = static_cast<uint64_t>(index) * sensor_step_us;
    const uint64_t rx_time_us = static_cast<uint64_t>(index) * rx_step_us + rx_bias_us;
    harness.PushGyro(sensor_timestamp_us, rx_time_us);
    harness.PushAccl(sensor_timestamp_us, rx_time_us + 20);
    harness.PushQuat(sensor_timestamp_us, rx_time_us + 40);
    harness.Drain();
  }
}

void TestProbeLockAndTrack()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 24, 1000, 1000);
  harness.PushImage(4000, 4, 4120, 0);
  harness.Drain();
  harness.PushImage(8000, 8, 8120, 1);
  harness.Drain();
  harness.ArmProbe();
  harness.PushImage(16000, 16, 16120, 2);
  harness.Drain();
  harness.PushImage(20000, 20, 20120, 3);
  harness.Drain();
  harness.PushImage(24000, 24, 24120, 4);
  harness.Drain();

  ExpectEqual(harness.PublishedImageTags(), std::vector<uint32_t>({2, 3, 4}),
              "probe lock publish tags");
  ExpectEqual(harness.PublishedSyncImuTimestamps(),
              std::vector<uint64_t>({16000, 20000, 24000}),
              "probe lock sync imu timestamps");
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "probe lock final state");
}

void TestPositiveOffsetUsesLaterImu()
{
  SequenceHarness harness;
  harness.SetOffsetUs(700);

  FeedImuTriplets(harness, 1, 20, 1000, 1000);
  harness.PushImage(4000, 4, 4120, 0);
  harness.Drain();
  harness.PushImage(8000, 8, 8120, 1);
  harness.Drain();
  harness.ArmProbe();
  harness.PushImage(16000, 16, 16120, 2);
  harness.Drain();

  ExpectEqual(harness.PublishedFinalImuTimestamps(), std::vector<uint64_t>({17000}),
              "positive offset final imu");
}

void TestForwardSearchUsesLaterAcclAndQuat()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 15, 1000, 1000);
  harness.PushGyro(16000, 16000);
  harness.PushAccl(15900, 15900);
  harness.PushAccl(16050, 16050);
  harness.PushQuat(15950, 15950);
  harness.PushQuat(16060, 16060);
  harness.Drain();

  harness.PushImage(4000, 4, 4100, 0);
  harness.Drain();
  harness.PushImage(8000, 8, 8100, 1);
  harness.Drain();
  harness.ArmProbe();
  harness.PushImage(16000, 16, 16100, 2);
  harness.Drain();

  ExpectEqual(harness.PublishedSyncImuTimestamps(), std::vector<uint64_t>({16000}),
              "forward search sync imu timestamp");
  ExpectEqual(harness.PublishedAcclTimestamps(), std::vector<uint64_t>({16050}),
              "forward search accl timestamp");
  ExpectEqual(harness.PublishedQuatTimestamps(), std::vector<uint64_t>({16060}),
              "forward search quat timestamp");
  ExpectEqual(harness.PublishedFinalImuTimestamps(), std::vector<uint64_t>({16000}),
              "forward search final imu");
}

void TestBadProbeGapRejects()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 20, 1000, 1000);
  harness.PushImage(4000, 4, 4120, 0);
  harness.Drain();
  harness.PushImage(8000, 8, 8120, 1);
  harness.Drain();
  harness.ArmProbe();
  harness.PushImage(12000, 12, 12120, 2);
  harness.Drain();

  ExpectEqual(harness.PublishedImageTags(), std::vector<uint32_t>({}),
              "bad probe gap should not publish");
  ExpectEqual(harness.RejectTags(), std::vector<uint32_t>({2}), "bad probe gap reject tag");
  ExpectEqual(harness.CurrentState(), SyncState::RECOVERING, "bad probe gap state");
}

void TestLockedSkewJumpRejects()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 24, 1000, 1000);
  harness.PushImage(4000, 4, 4120, 0);
  harness.Drain();
  harness.PushImage(8000, 8, 8120, 1);
  harness.Drain();
  harness.ArmProbe();
  harness.PushImage(16000, 16, 16120, 2);
  harness.Drain();
  harness.PushImage(20000, 20, 35000, 3);
  harness.Drain();

  ExpectEqual(harness.PublishedImageTags(), std::vector<uint32_t>({2}),
              "skew jump only probe frame should publish");
  ExpectEqual(harness.RejectTags(), std::vector<uint32_t>({3}), "skew jump reject tag");
  ExpectEqual(harness.CurrentState(), SyncState::RECOVERING, "skew jump state");
}

void TestRecoveringNeedsThreeGoodFramesToRelock()
{
  SequenceHarness harness;
  harness.SetRelockConfirmFrames(3);

  FeedImuTriplets(harness, 1, 48, 1000, 1000);
  harness.PushImage(4000, 4, 4120, 0);
  harness.Drain();
  harness.PushImage(8000, 8, 8120, 1);
  harness.Drain();
  harness.ArmProbe();
  harness.PushImage(16000, 16, 16120, 2);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::LOCKING, "first probe should start locking");

  harness.PushImage(20000, 20, 35000, 3);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::RECOVERING, "bad frame should recover");

  harness.PushImage(24000, 24, 24120, 4);
  harness.Drain();
  harness.PushImage(28000, 28, 28120, 5);
  harness.Drain();
  harness.ArmProbe();
  harness.PushImage(36000, 36, 36120, 6);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::LOCKING, "second probe lock frame 1");
  ExpectEqual(harness.LockConfirmCount(), static_cast<uint32_t>(1), "second probe lock count");

  harness.PushImage(40000, 40, 40120, 7);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::LOCKING, "relock frame 2");
  ExpectEqual(harness.LockConfirmCount(), static_cast<uint32_t>(2), "relock count 2");

  harness.PushImage(44000, 44, 44120, 8);
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
      {"正确/probe锁定后按时间差跟踪", TestProbeLockAndTrack},
      {"正确/正offset在IMU域内取后继样本", TestPositiveOffsetUsesLaterImu},
      {"正确/gyro前向找后继accl与quat", TestForwardSearchUsesLaterAcclAndQuat},
      {"错误/probe图像周期不是2T", TestBadProbeGapRejects},
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
