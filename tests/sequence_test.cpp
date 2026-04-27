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

struct CadenceObservation
{
  RxCadenceState gyro{};
  RxCadenceState accl{};
  RxCadenceState quat{};
  RxCadenceState image{};
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

  void PushGyro(uint64_t sensor_timestamp_us, uint64_t rx_time_us)
  {
    gyro_ingress_.PushBackDropOldest({{sensor_timestamp_us}, rx_time_us});
  }

  void PushAccl(uint64_t sensor_timestamp_us, uint64_t rx_time_us)
  {
    accl_ingress_.PushBackDropOldest({{sensor_timestamp_us}, rx_time_us});
  }

  void PushQuat(uint64_t sensor_timestamp_us, uint64_t rx_time_us)
  {
    quat_ingress_.PushBackDropOldest({{sensor_timestamp_us}, rx_time_us});
  }

  void PushImage(uint64_t sensor_timestamp_us, uint32_t sensor_step_index, uint64_t rx_time_us,
                 uint32_t tag)
  {
    image_ingress_.PushBackDropOldest({{sensor_timestamp_us, sensor_step_index}, rx_time_us, tag});
  }

  void Drain()
  {
    DrainIngressQueues();
    MaybeSendProbe();
    DrainPendingImages();
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

  uint32_t ProbeSentCount() const { return probe_sent_count_; }

 private:
  struct SyncMatch
  {
    const TestImu* sync_imu{nullptr};
    uint64_t next_sync_sensor_period_us{0};
    int64_t host_skew_us{0};

    bool Valid() const
    {
      return sync_imu != nullptr && next_sync_sensor_period_us != 0;
    }
  };

  enum class ImageDecision : uint8_t
  {
    WAIT = 0,
    ACCEPT = 1,
    REJECT = 2,
  };

  static constexpr size_t kQueueLimit = 256;
  static constexpr uint32_t kCadenceStableGaps = 2;
  static constexpr uint64_t kRawCadenceMinToleranceUs = 300ULL;
  static constexpr uint64_t kImageCadenceMinToleranceUs = 1500ULL;

  void ResetLockedRelation()
  {
    relation_.sync_imu_sensor_period_us = 0;
    relation_.last_image_rx_time_us = 0;
    relation_.last_sync_imu_sensor_timestamp_us = 0;
    relation_.last_host_skew_us = 0;
  }

  void ResetImageObservation()
  {
    relation_.base_image_rx_period_us = 0;
    last_observed_image_ = {};
    last_normal_image_ = {};
  }

  void ResetCadenceObservation()
  {
    cadence_ = {};
    ResetImageObservation();
  }

  void SetObservingState()
  {
    lock_state_.state = SyncState::OBSERVING;
    lock_state_.lock_confirm_count = 0;
  }

  void ClearPendingProbe() { probe_pending_ = false; }

  void EnterObserving()
  {
    if (lock_state_.state == SyncState::LOCKING || lock_state_.state == SyncState::SYNCED)
    {
      return;
    }

    SetObservingState();
  }

  void ResetSyncTracking()
  {
    ClearPendingProbe();
    ResetLockedRelation();
    ResetImageObservation();
    SetObservingState();
  }

  bool IsLockingOrSynced() const
  {
    return lock_state_.state == SyncState::LOCKING || lock_state_.state == SyncState::SYNCED;
  }

  bool CadenceReady() const
  {
    return cadence_.gyro.stable && cadence_.accl.stable && cadence_.quat.stable &&
           cadence_.image.stable;
  }

  void HandleCadenceUpdate(CadenceUpdate update)
  {
    switch (update)
    {
      case CadenceUpdate::NO_GAP:
        break;
      case CadenceUpdate::WARMING:
      case CadenceUpdate::STABLE:
        EnterObserving();
        break;
      case CadenceUpdate::BROKEN:
        ResetSyncTracking();
        break;
    }
  }

  void ObserveRawCadence(RxCadenceState& cadence, uint64_t rx_time_us)
  {
    HandleCadenceUpdate(
        ObserveRxCadence(cadence, rx_time_us, kCadenceStableGaps, kRawCadenceMinToleranceUs));
  }

  CadenceUpdate ObserveImageCadence(const TimedTestImage& image)
  {
    // probe 的 2T 图像 gap 是主动扰动，不算 image cadence 失稳。
    if (probe_pending_ && cadence_.image.stable && relation_.base_image_rx_period_us != 0 &&
        cadence_.image.has_last_rx && image.rx_time_us > cadence_.image.last_rx_us)
    {
      const uint64_t image_gap_rx_us = image.rx_time_us - cadence_.image.last_rx_us;
      if (AbsDiffUs(image_gap_rx_us, relation_.base_image_rx_period_us * 2ULL) <=
          ImageGapToleranceUs(relation_.base_image_rx_period_us, relation_.last_imu_rx_period_us))
      {
        cadence_.image.last_rx_us = image.rx_time_us;
        HandleCadenceUpdate(CadenceUpdate::STABLE);
        return CadenceUpdate::STABLE;
      }
    }

    const CadenceUpdate update = ObserveRxCadence(
        cadence_.image, image.rx_time_us, kCadenceStableGaps, kImageCadenceMinToleranceUs);
    HandleCadenceUpdate(update);
    return update;
  }

  void RecordStableImageAnchor(const TimedTestImage& image)
  {
    if (!cadence_.image.stable)
    {
      return;
    }

    relation_.base_image_rx_period_us = cadence_.image.period_us;
    last_normal_image_.valid = true;
    last_normal_image_.image = image;
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

    return AbsDiffUs(image_gap_rx_us, relation_.base_image_rx_period_us * 2ULL) <=
           ImageGapToleranceUs(relation_.base_image_rx_period_us, relation_.last_imu_rx_period_us);
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

  void DrainIngressQueues()
  {
    while (!gyro_ingress_.Empty())
    {
      const TimedTestGyro gyro = gyro_ingress_.Front();
      ObserveRawCadence(cadence_.gyro, gyro.rx_time_us);
      pending_gyros_.PushBackDropOldest(gyro);
      gyro_ingress_.PopFront();
    }

    while (!accl_ingress_.Empty())
    {
      const TimedTestAccl accl = accl_ingress_.Front();
      ObserveRawCadence(cadence_.accl, accl.rx_time_us);
      pending_accls_.PushBackDropOldest(accl);
      accl_ingress_.PopFront();
    }

    while (!quat_ingress_.Empty())
    {
      const TimedTestQuat quat = quat_ingress_.Front();
      ObserveRawCadence(cadence_.quat, quat.rx_time_us);
      pending_quats_.PushBackDropOldest(quat);
      quat_ingress_.PopFront();
    }

    while (!image_ingress_.Empty())
    {
      pending_images_.PushBackDropOldest(image_ingress_.Front());
      image_ingress_.PopFront();
    }

    AssembleImuHistory();
  }

  void AssembleImuHistory()
  {
    while (TryBuildFrontImu(
        pending_gyros_, pending_accls_, pending_quats_, imu_history_,
        relation_.last_imu_sensor_period_us, relation_.last_imu_rx_period_us, kQueueLimit,
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
  }

  void DrainPendingImages()
  {
    while (!pending_images_.Empty())
    {
      const TimedTestImage image = pending_images_.Front();
      const CadenceUpdate cadence_update = ObserveImageCadence(image);
      if (!last_observed_image_.valid)
      {
        RememberObservedImage(image);
        pending_images_.PopFront();
        continue;
      }

      if (image.rx_time_us <= last_observed_image_.image.rx_time_us)
      {
        pending_images_.PopFront();
        ClearPendingProbe();
        ResetLockedRelation();
        ResetCadenceObservation();
        SetObservingState();
        continue;
      }

      const uint64_t image_gap_rx_us = image.rx_time_us - last_observed_image_.image.rx_time_us;
      if (cadence_update == CadenceUpdate::BROKEN)
      {
        pending_images_.PopFront();
        RememberObservedImage(image);
        continue;
      }

      const bool probe_pending = probe_pending_;
      ImageDecision decision = ImageDecision::REJECT;
      if (probe_pending)
      {
        decision = IsProbeImageGap(image_gap_rx_us) ? TryLockFromProbe(image, image_gap_rx_us)
                                                    : ImageDecision::REJECT;
      }
      else if (IsLockingOrSynced())
      {
        decision =
            IsNormalImageGap(image_gap_rx_us) ? TryProcessLockedImage(image, image_gap_rx_us)
                                              : ImageDecision::REJECT;
      }
      else
      {
        RecordStableImageAnchor(image);
        RememberObservedImage(image);
        pending_images_.PopFront();
        MaybeSendProbe();
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
          ClearPendingProbe();
        }
        else
        {
          RecordStableImageAnchor(image);
        }
        RememberObservedImage(image);
        continue;
      }

      reject_tags_.push_back(image.tag);
      if (probe_pending)
      {
        ClearPendingProbe();
      }
      ResetSyncTracking();
      MaybeSendProbe();
    }
  }

  void MaybeSendProbe()
  {
    if (IsLockingOrSynced())
    {
      return;
    }

    if (!CadenceReady() || relation_.base_image_rx_period_us == 0 ||
        EstimatedSyncImuSensorPeriodUs() == 0 || !last_normal_image_.valid)
    {
      return;
    }

    if (probe_pending_)
    {
      return;
    }

    probe_pending_ = true;
    ++probe_sent_count_;
  }

  static bool IsBetterTrackedMatch(uint64_t host_skew_error_us,
                                   uint64_t best_host_skew_error_us,
                                   uint64_t sensor_gap_error_us,
                                   uint64_t best_sensor_gap_error_us)
  {
    return host_skew_error_us < best_host_skew_error_us ||
           (host_skew_error_us == best_host_skew_error_us &&
            sensor_gap_error_us < best_sensor_gap_error_us);
  }

  static bool IsBetterProbeMatch(uint64_t host_gap_error_us,
                                 uint64_t best_host_gap_error_us,
                                 uint64_t sensor_gap_error_us,
                                 uint64_t best_sensor_gap_error_us,
                                 int64_t abs_probe_skew_us,
                                 int64_t best_abs_probe_skew_us)
  {
    return host_gap_error_us < best_host_gap_error_us ||
           (host_gap_error_us == best_host_gap_error_us &&
            sensor_gap_error_us < best_sensor_gap_error_us) ||
           (host_gap_error_us == best_host_gap_error_us &&
            sensor_gap_error_us == best_sensor_gap_error_us &&
            abs_probe_skew_us < best_abs_probe_skew_us);
  }

  void Publish(const TimedTestImage& image, const TestImu& sync_imu, const TestImu& final_imu)
  {
    published_image_tags_.push_back(image.tag);
    published_sync_imu_timestamps_.push_back(sync_imu.sensor_timestamp_us);
    published_final_imu_timestamps_.push_back(final_imu.sensor_timestamp_us);
    published_accl_timestamps_.push_back(final_imu.accl_timestamp_us);
    published_quat_timestamps_.push_back(final_imu.quat_timestamp_us);
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

  ImageDecision PublishMatchedImage(const TimedTestImage& image, const SyncMatch& match)
  {
    const TestImu* final_imu = FindFinalImu(*match.sync_imu);
    if (final_imu == nullptr)
    {
      return NeedMoreImuForOffset(*match.sync_imu) ? ImageDecision::WAIT : ImageDecision::REJECT;
    }

    Publish(image, *match.sync_imu, *final_imu);
    relation_.sync_imu_sensor_period_us = match.next_sync_sensor_period_us;
    relation_.last_image_rx_time_us = image.rx_time_us;
    relation_.last_sync_imu_sensor_timestamp_us = match.sync_imu->sensor_timestamp_us;
    relation_.last_host_skew_us = match.host_skew_us;
    ObserveGoodFrame(lock_state_, relock_confirm_frames_);
    return ImageDecision::ACCEPT;
  }

  SyncMatch BuildTrackedSyncMatch(const TimedTestImage& image, const TestImu& sync_imu,
                                  uint64_t expected_sync_sensor_gap_us,
                                  uint64_t sensor_gap_tolerance_us,
                                  uint64_t host_skew_tolerance_us) const
  {
    if (sync_imu.sensor_timestamp_us <= relation_.last_sync_imu_sensor_timestamp_us)
    {
      return {};
    }

    const uint64_t sync_sensor_gap_us =
        sync_imu.sensor_timestamp_us - relation_.last_sync_imu_sensor_timestamp_us;
    const uint64_t sensor_gap_error_us = AbsDiffUs(sync_sensor_gap_us, expected_sync_sensor_gap_us);
    if (sensor_gap_error_us > sensor_gap_tolerance_us)
    {
      return {};
    }

    const int64_t host_skew_us =
        static_cast<int64_t>(image.rx_time_us) - static_cast<int64_t>(sync_imu.rx_time_us);
    const uint64_t host_skew_error_us = AbsDiffSigned(host_skew_us, relation_.last_host_skew_us);
    if (host_skew_error_us > host_skew_tolerance_us)
    {
      return {};
    }

    return SyncMatch{
        .sync_imu = &sync_imu,
        .next_sync_sensor_period_us = sync_sensor_gap_us,
        .host_skew_us = host_skew_us,
    };
  }

  SyncMatch SelectTrackedSyncMatch(const TimedTestImage& image,
                                   uint64_t expected_sync_sensor_gap_us,
                                   uint64_t sensor_gap_tolerance_us,
                                   uint64_t host_skew_tolerance_us,
                                   uint64_t search_window_us) const
  {
    const uint64_t expected_sync_sensor_timestamp_us =
        relation_.last_sync_imu_sensor_timestamp_us + expected_sync_sensor_gap_us;
    const TestImu* predicted_sync_imu =
        FindBySensorTimestamp(imu_history_, expected_sync_sensor_timestamp_us,
                              sensor_gap_tolerance_us);
    if (predicted_sync_imu != nullptr)
    {
      const SyncMatch predicted_match = BuildTrackedSyncMatch(
          image, *predicted_sync_imu, expected_sync_sensor_gap_us,
          sensor_gap_tolerance_us, host_skew_tolerance_us);
      if (predicted_match.Valid())
      {
        return predicted_match;
      }
    }

    SyncMatch best_match{};
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

      const SyncMatch match = BuildTrackedSyncMatch(
          image, sync_imu, expected_sync_sensor_gap_us,
          sensor_gap_tolerance_us, host_skew_tolerance_us);
      if (!match.Valid())
      {
        continue;
      }

      const uint64_t host_skew_error_us = AbsDiffSigned(match.host_skew_us, relation_.last_host_skew_us);
      const uint64_t sensor_gap_error_us =
          AbsDiffUs(match.next_sync_sensor_period_us, expected_sync_sensor_gap_us);
      if (IsBetterTrackedMatch(host_skew_error_us, best_host_skew_error_us,
                               sensor_gap_error_us, best_sensor_gap_error_us))
      {
        best_match = match;
        best_host_skew_error_us = host_skew_error_us;
        best_sensor_gap_error_us = sensor_gap_error_us;
      }
    }

    return best_match;
  }

  SyncMatch SelectProbeSyncMatch(const TimedTestImage& probe, uint64_t image_gap_rx_us,
                                 uint64_t expected_probe_sensor_gap_us,
                                 uint64_t sensor_gap_tolerance_us,
                                 uint64_t search_window_us,
                                 uint64_t host_gap_tolerance_us) const
  {
    SyncMatch best_match{};
    uint64_t best_host_gap_error_us = std::numeric_limits<uint64_t>::max();
    uint64_t best_sensor_gap_error_us = std::numeric_limits<uint64_t>::max();
    int64_t best_abs_probe_skew_us = std::numeric_limits<int64_t>::max();

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

      const uint64_t sync_sensor_gap_us =
          sync_imu.sensor_timestamp_us - prev_sync_imu->sensor_timestamp_us;
      const uint64_t sensor_gap_error_us =
          AbsDiffUs(sync_sensor_gap_us, expected_probe_sensor_gap_us);
      const int64_t probe_skew_us =
          static_cast<int64_t>(probe.rx_time_us) - static_cast<int64_t>(sync_imu.rx_time_us);
      const int64_t abs_probe_skew_us = probe_skew_us >= 0 ? probe_skew_us : -probe_skew_us;
      if (sync_sensor_gap_us < 2 ||
          !IsBetterProbeMatch(host_gap_error_us, best_host_gap_error_us,
                              sensor_gap_error_us, best_sensor_gap_error_us,
                              abs_probe_skew_us, best_abs_probe_skew_us))
      {
        continue;
      }

      best_match = SyncMatch{
          .sync_imu = &sync_imu,
          .next_sync_sensor_period_us = sync_sensor_gap_us / 2ULL,
          .host_skew_us = probe_skew_us,
      };
      best_host_gap_error_us = host_gap_error_us;
      best_sensor_gap_error_us = sensor_gap_error_us;
      best_abs_probe_skew_us = abs_probe_skew_us;
    }

    return best_match;
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

    const SyncMatch match = SelectProbeSyncMatch(
        probe, image_gap_rx_us, expected_probe_sensor_gap_us,
        sensor_gap_tolerance_us, search_window_us, host_gap_tolerance_us);
    if (!match.Valid())
    {
      return ImageDecision::REJECT;
    }

    return PublishMatchedImage(probe, match);
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

    const SyncMatch match = SelectTrackedSyncMatch(
        image, expected_sync_sensor_gap_us, sensor_gap_tolerance_us,
        host_skew_tolerance_us, search_window_us);
    if (!match.Valid())
    {
      return ImageDecision::REJECT;
    }

    return PublishMatchedImage(image, match);
  }

 private:
  int32_t offset_us_{0};
  uint32_t relock_confirm_frames_{3};
  bool probe_pending_{false};
  uint32_t probe_sent_count_{0};
  SyncLockState lock_state_{};
  CadenceObservation cadence_{};
  SyncRelation relation_{};
  ImageReference last_observed_image_{};
  ImageReference last_normal_image_{};

  Ring<TimedTestGyro, kQueueLimit> gyro_ingress_{};
  Ring<TimedTestAccl, kQueueLimit> accl_ingress_{};
  Ring<TimedTestQuat, kQueueLimit> quat_ingress_{};
  Ring<TimedTestImage, kQueueLimit> image_ingress_{};
  Ring<TimedTestGyro, kQueueLimit> pending_gyros_{};
  Ring<TimedTestAccl, kQueueLimit> pending_accls_{};
  Ring<TimedTestQuat, kQueueLimit> pending_quats_{};
  Ring<TimedTestImage, kQueueLimit> pending_images_{};
  Ring<TestImu, kQueueLimit> imu_history_{};

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

void TestCadenceMustStabilizeBeforeProbe()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 28, 1000, 1000);
  harness.PushImage(4000, 4, 4120, 0);
  harness.Drain();
  harness.PushImage(8000, 8, 8120, 1);
  harness.Drain();

  ExpectEqual(harness.ProbeSentCount(), static_cast<uint32_t>(0), "probe should not arm early");
  ExpectEqual(harness.CurrentState(), SyncState::OBSERVING, "startup observing state");

  harness.PushImage(12000, 12, 12120, 2);
  harness.Drain();
  ExpectEqual(harness.ProbeSentCount(), static_cast<uint32_t>(1), "probe should arm after stable cadence");

  harness.PushImage(20000, 20, 20120, 3);
  harness.Drain();
  harness.PushImage(24000, 24, 24120, 4);
  harness.Drain();
  harness.PushImage(28000, 28, 28120, 5);
  harness.Drain();

  ExpectEqual(harness.PublishedImageTags(), std::vector<uint32_t>({3, 4, 5}),
              "publish tags after startup observe");
  ExpectEqual(harness.PublishedSyncImuTimestamps(),
              std::vector<uint64_t>({20000, 24000, 28000}),
              "sync imu timestamps after startup observe");
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "startup final state");
}

void TestPositiveOffsetUsesLaterImu()
{
  SequenceHarness harness;
  harness.SetOffsetUs(700);

  FeedImuTriplets(harness, 1, 24, 1000, 1000);
  harness.PushImage(4000, 4, 4120, 0);
  harness.Drain();
  harness.PushImage(8000, 8, 8120, 1);
  harness.Drain();
  harness.PushImage(12000, 12, 12120, 2);
  harness.Drain();
  harness.PushImage(20000, 20, 20120, 3);
  harness.Drain();

  ExpectEqual(harness.PublishedFinalImuTimestamps(), std::vector<uint64_t>({21000}),
              "positive offset final imu");
}

void TestForwardSearchUsesLaterAcclAndQuat()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 19, 1000, 1000);
  harness.PushGyro(20000, 20000);
  harness.PushAccl(20050, 20050);
  harness.PushQuat(20060, 20060);
  harness.Drain();

  harness.PushImage(4000, 4, 4100, 0);
  harness.Drain();
  harness.PushImage(8000, 8, 8100, 1);
  harness.Drain();
  harness.PushImage(12000, 12, 12100, 2);
  harness.Drain();
  harness.PushImage(20000, 20, 20100, 3);
  harness.Drain();

  ExpectEqual(harness.PublishedSyncImuTimestamps(), std::vector<uint64_t>({20000}),
              "forward search sync imu timestamp");
  ExpectEqual(harness.PublishedAcclTimestamps(), std::vector<uint64_t>({20050}),
              "forward search accl timestamp");
  ExpectEqual(harness.PublishedQuatTimestamps(), std::vector<uint64_t>({20060}),
              "forward search quat timestamp");
  ExpectEqual(harness.PublishedFinalImuTimestamps(), std::vector<uint64_t>({20000}),
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
  harness.PushImage(12000, 12, 12120, 2);
  harness.Drain();
  ExpectEqual(harness.ProbeSentCount(), static_cast<uint32_t>(1), "probe should arm once");

  harness.PushImage(16000, 16, 16120, 3);
  harness.Drain();

  ExpectEqual(harness.PublishedImageTags(), std::vector<uint32_t>({}),
              "bad probe gap should not publish");
  ExpectEqual(harness.RejectTags(), std::vector<uint32_t>({3}), "bad probe gap reject tag");
  ExpectEqual(harness.CurrentState(), SyncState::OBSERVING, "bad probe gap should reset to observing");
}

void TestStartupJitterKeepsObservingUntilStable()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 40, 1000, 1000);
  harness.PushImage(4000, 4, 4120, 0);
  harness.Drain();
  harness.PushImage(8000, 8, 8120, 1);
  harness.Drain();
  harness.PushImage(14500, 14, 14620, 2);
  harness.Drain();

  ExpectEqual(harness.ProbeSentCount(), static_cast<uint32_t>(0), "jitter should block probe");
  ExpectEqual(harness.PublishedImageTags(), std::vector<uint32_t>({}),
              "jitter should not publish");

  harness.PushImage(18500, 18, 18620, 3);
  harness.Drain();
  harness.PushImage(22500, 22, 22620, 4);
  harness.Drain();
  ExpectEqual(harness.ProbeSentCount(), static_cast<uint32_t>(1), "probe after cadence restabilizes");

  harness.PushImage(30500, 30, 30620, 5);
  harness.Drain();
  harness.PushImage(34500, 34, 34620, 6);
  harness.Drain();
  harness.PushImage(38500, 38, 38620, 7);
  harness.Drain();

  ExpectEqual(harness.PublishedImageTags(), std::vector<uint32_t>({5, 6, 7}),
              "publish tags after startup jitter");
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "startup jitter final state");
}

void TestImageCadenceBreakResetsAndRelocks()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, 1, 58, 1000, 1000);
  harness.PushImage(4000, 4, 4120, 0);
  harness.Drain();
  harness.PushImage(8000, 8, 8120, 1);
  harness.Drain();
  harness.PushImage(12000, 12, 12120, 2);
  harness.Drain();
  harness.PushImage(20000, 20, 20120, 3);
  harness.Drain();
  harness.PushImage(24000, 24, 24120, 4);
  harness.Drain();
  harness.PushImage(28000, 28, 28120, 5);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "initial sync state");

  harness.PushImage(34000, 34, 34120, 6);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::OBSERVING,
              "cadence break should reset to observing");
  ExpectEqual(harness.ProbeSentCount(), static_cast<uint32_t>(1), "break should not send new probe immediately");

  harness.PushImage(38000, 38, 38120, 7);
  harness.Drain();
  harness.PushImage(42000, 42, 42120, 8);
  harness.Drain();
  ExpectEqual(harness.ProbeSentCount(), static_cast<uint32_t>(2), "restabilized cadence should send second probe");

  harness.PushImage(50000, 50, 50120, 9);
  harness.Drain();
  harness.PushImage(54000, 54, 54120, 10);
  harness.Drain();
  harness.PushImage(58000, 58, 58120, 11);
  harness.Drain();

  ExpectEqual(harness.RejectTags(), std::vector<uint32_t>({}),
              "cadence break should reset, not mark match reject");
  ExpectEqual(harness.PublishedImageTags(), std::vector<uint32_t>({3, 4, 5, 9, 10, 11}),
              "publish tags after relock");
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "cadence relock final state");
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
      {"正确/稳定观察后自动发probe并锁定", TestCadenceMustStabilizeBeforeProbe},
      {"正确/正offset在IMU域内取后继样本", TestPositiveOffsetUsesLaterImu},
      {"正确/gyro前向找后继accl与quat", TestForwardSearchUsesLaterAcclAndQuat},
      {"错误/probe图像周期不是2T", TestBadProbeGapRejects},
      {"错误/启动抖动时保持观察直到稳定", TestStartupJitterKeepsObservingUntilStable},
      {"错误后恢复/image cadence 失稳后重新观察并重锁", TestImageCadenceBreakResetsAndRelocks},
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
