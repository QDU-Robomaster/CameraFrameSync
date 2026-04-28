#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
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

struct QueuedTestGyro
{
  TestGyro sample{};
};

struct QueuedTestAccl
{
  TestAccl sample{};
};

struct QueuedTestQuat
{
  TestQuat sample{};
};

struct TestImageEvent
{
  uint64_t sensor_timestamp_us{};
  uint32_t sensor_step_index{};
};

struct QueuedTestImage
{
  TestImageEvent sample{};
  uint32_t tag{};
};

struct TestImu
{
  uint64_t sensor_timestamp_us{};
  uint64_t accl_timestamp_us{};
  uint64_t quat_timestamp_us{};
};

class SequenceHarness
{
 public:
  template <typename T, size_t Capacity>
  using Ring = FixedRingBuffer<T, Capacity>;

  void SetOffsetUs(int32_t offset_us) { offset_us_ = offset_us; }

  void PushGyro(uint64_t sensor_timestamp_us)
  {
    if (gyro_ingress_.PushBackDropOldest({{sensor_timestamp_us}}))
    {
      overflowed_ = true;
    }
  }

  void PushAccl(uint64_t sensor_timestamp_us)
  {
    if (accl_ingress_.PushBackDropOldest({{sensor_timestamp_us}}))
    {
      overflowed_ = true;
    }
  }

  void PushQuat(uint64_t sensor_timestamp_us)
  {
    if (quat_ingress_.PushBackDropOldest({{sensor_timestamp_us}}))
    {
      overflowed_ = true;
    }
  }

  void PushImage(uint64_t sensor_timestamp_us, uint32_t sensor_step_index, uint32_t tag)
  {
    if (image_ingress_.PushBackDropOldest({{sensor_timestamp_us, sensor_step_index}, tag}))
    {
      overflowed_ = true;
    }
  }

  void Drain()
  {
    DrainIngressQueues();
    HandleOverflowRecovery();
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

  uint32_t ProbeSentCount() const { return probe_sent_count_; }

 private:
  struct ImageReference
  {
    bool valid{false};
    TestImageEvent sample{};
  };

  struct PendingImageState
  {
    bool valid{false};
    TestImageEvent sample{};
    uint32_t tag{};
    bool cadence_observed{false};
    bool sync_candidate_valid{false};
    uint64_t sync_candidate_sensor_timestamp_us{0};
    uint64_t sync_candidate_period_us{0};
  };

  struct CadenceObservation
  {
    CadenceState gyro{};
    CadenceState accl{};
    CadenceState quat{};
    CadenceState image{};
  };

  struct SyncRelation
  {
    uint64_t base_image_sensor_period_us{0};
    uint64_t last_imu_sensor_period_us{0};
    uint64_t sync_imu_sensor_period_us{0};
    uint64_t last_image_sensor_timestamp_us{0};
    uint64_t last_sync_imu_sensor_timestamp_us{0};
  };

  struct SyncMatch
  {
    const TestImu* sync_imu{nullptr};
    uint64_t next_sync_sensor_period_us{0};

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

  static constexpr size_t queue_limit = 256;
  static constexpr uint32_t cadence_stable_gaps = 2;
  static constexpr uint64_t raw_cadence_min_tolerance_us = 300ULL;
  static constexpr uint64_t image_cadence_min_tolerance_us = 1500ULL;

  static uint64_t ImageGapToleranceUs(const SyncRelation& relation)
  {
    return CameraFrameSyncCore::ImageGapToleranceUs(relation.base_image_sensor_period_us);
  }

  void ResetLockedRelation()
  {
    relation_.sync_imu_sensor_period_us = 0;
    relation_.last_image_sensor_timestamp_us = 0;
    relation_.last_sync_imu_sensor_timestamp_us = 0;
  }

  void ResetImageObservation()
  {
    relation_.base_image_sensor_period_us = 0;
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
  }

  void ResetSyncTracking()
  {
    ClearPendingProbe();
    ResetLockedRelation();
    ResetImageObservation();
    SetObservingState();
  }

  bool CadenceReady() const
  {
    return cadence_.gyro.stable && cadence_.image.stable;
  }

  void ObserveCadenceUpdate(CadenceUpdate update)
  {
    switch (update)
    {
      case CadenceUpdate::NO_GAP:
      case CadenceUpdate::WARMING:
      case CadenceUpdate::STABLE:
        break;
      case CadenceUpdate::BROKEN:
        ResetSyncTracking();
        break;
    }
  }

  void ObserveSampleCadence(CadenceState& cadence, uint64_t sensor_timestamp_us)
  {
    ObserveCadenceUpdate(ObserveCadence(
        cadence, sensor_timestamp_us, cadence_stable_gaps, raw_cadence_min_tolerance_us));
  }

  CadenceUpdate ObserveImageCadence(const TestImageEvent& image)
  {
    const uint64_t image_timestamp_us = image.sensor_timestamp_us;
    if (probe_pending_ && cadence_.image.stable && relation_.base_image_sensor_period_us != 0 &&
        cadence_.image.has_last_timestamp &&
        image_timestamp_us > cadence_.image.last_timestamp_us)
    {
      const uint64_t image_gap_sensor_us = image_timestamp_us - cadence_.image.last_timestamp_us;
      if (AbsDiffUs(image_gap_sensor_us, relation_.base_image_sensor_period_us * 2ULL) <=
          ImageGapToleranceUs(relation_))
      {
        cadence_.image.last_timestamp_us = image_timestamp_us;
        ObserveCadenceUpdate(CadenceUpdate::STABLE);
        return CadenceUpdate::STABLE;
      }
    }

    const auto update = ObserveCadence(cadence_.image, image_timestamp_us, cadence_stable_gaps,
                                       image_cadence_min_tolerance_us);
    ObserveCadenceUpdate(update);
    return update;
  }

  void RecordStableImageAnchor(const TestImageEvent& image)
  {
    if (!cadence_.image.stable)
    {
      return;
    }

    relation_.base_image_sensor_period_us = cadence_.image.period_us;
    last_normal_image_.valid = true;
    last_normal_image_.sample = image;
  }

  uint32_t EstimatedStrideSamples() const
  {
    return EstimateStrideSamples(relation_.base_image_sensor_period_us,
                                 relation_.last_imu_sensor_period_us);
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

  bool IsNormalImageGap(uint64_t image_gap_sensor_us) const
  {
    if (relation_.base_image_sensor_period_us == 0 || image_gap_sensor_us == 0)
    {
      return false;
    }

    return AbsDiffUs(image_gap_sensor_us, relation_.base_image_sensor_period_us) <=
           ImageGapToleranceUs(relation_);
  }

  bool IsProbeImageGap(uint64_t image_gap_sensor_us) const
  {
    if (relation_.base_image_sensor_period_us == 0 || image_gap_sensor_us == 0)
    {
      return false;
    }

    return AbsDiffUs(image_gap_sensor_us, relation_.base_image_sensor_period_us * 2ULL) <=
           ImageGapToleranceUs(relation_);
  }

  bool IsSynced() const { return lock_state_.state == SyncState::SYNCED; }

  void ClearPendingProbe() { probe_pending_ = false; }

  void ClearPendingImage() { pending_image_ = {}; }

  bool ImuHistoryReached(uint64_t target_timestamp_us) const
  {
    return !imu_history_.Empty() && imu_history_.Back().sensor_timestamp_us >= target_timestamp_us;
  }

  bool ImuHistorySpanReady(uint64_t required_span_us) const
  {
    if (imu_history_.Size() < 2)
    {
      return false;
    }

    return imu_history_.Back().sensor_timestamp_us - imu_history_.Front().sensor_timestamp_us >=
           required_span_us;
  }

  void MaybeSendProbe()
  {
    if (IsSynced())
    {
      return;
    }

    if (!CadenceReady() || relation_.base_image_sensor_period_us == 0 ||
        EstimatedSyncImuSensorPeriodUs() == 0 || !last_normal_image_.valid || probe_pending_)
    {
      return;
    }

    probe_pending_ = true;
    probe_sent_count_++;
  }

  void DrainIngressQueues()
  {
    while (!gyro_ingress_.Empty())
    {
      const QueuedTestGyro gyro = gyro_ingress_.Front();
      gyro_ingress_.PopFront();
      ObserveSampleCadence(cadence_.gyro, gyro.sample.sensor_timestamp_us);
      if (pending_gyros_.PushBackDropOldest(gyro))
      {
        overflowed_ = true;
      }
    }

    while (!accl_ingress_.Empty())
    {
      const QueuedTestAccl accl = accl_ingress_.Front();
      accl_ingress_.PopFront();
      ObserveSampleCadence(cadence_.accl, accl.sample.sensor_timestamp_us);
      if (pending_accls_.PushBackDropOldest(accl))
      {
        overflowed_ = true;
      }
    }

    while (!quat_ingress_.Empty())
    {
      const QueuedTestQuat quat = quat_ingress_.Front();
      quat_ingress_.PopFront();
      ObserveSampleCadence(cadence_.quat, quat.sample.sensor_timestamp_us);
      if (pending_quats_.PushBackDropOldest(quat))
      {
        overflowed_ = true;
      }
    }

    while (!image_ingress_.Empty())
    {
      const QueuedTestImage image = image_ingress_.Front();
      image_ingress_.PopFront();
      if (pending_images_.PushBackDropOldest(image))
      {
        overflowed_ = true;
      }
    }

    AssembleImuHistory();
  }

  void AssembleImuHistory()
  {
    while (TryBuildFrontImu(
        pending_gyros_, pending_accls_, pending_quats_, imu_history_,
        relation_.last_imu_sensor_period_us, queue_limit,
        [](const TestGyro& gyro, const TestAccl& accl, const TestQuat& quat)
        {
          return TestImu{
              .sensor_timestamp_us = gyro.sensor_timestamp_us,
              .accl_timestamp_us = accl.sensor_timestamp_us,
              .quat_timestamp_us = quat.sensor_timestamp_us,
          };
        }))
    {
    }
  }

  void RememberObservedImage(const TestImageEvent& image)
  {
    last_observed_image_.valid = true;
    last_observed_image_.sample = image;
  }

  void HandleOverflowRecovery()
  {
    if (!overflowed_)
    {
      return;
    }

    overflowed_ = false;
    ClearPendingImage();
    ClearPendingProbe();
    ResetLockedRelation();
    ResetCadenceObservation();
    SetObservingState();
  }

  void DrainPendingImages()
  {
    while (pending_image_.valid || !pending_images_.Empty())
    {
      if (!pending_image_.valid)
      {
        pending_image_.valid = true;
        pending_image_.sample = pending_images_.Front().sample;
        pending_image_.tag = pending_images_.Front().tag;
        pending_images_.PopFront();
      }

      PendingImageState& image = pending_image_;
      auto cadence_update = CadenceUpdate::NO_GAP;
      if (!image.cadence_observed)
      {
        cadence_update = ObserveImageCadence(image.sample);
        image.cadence_observed = true;
      }

      if (!last_observed_image_.valid)
      {
        RememberObservedImage(image.sample);
        ClearPendingImage();
        continue;
      }

      const uint64_t image_timestamp_us = image.sample.sensor_timestamp_us;
      const uint64_t last_image_timestamp_us = last_observed_image_.sample.sensor_timestamp_us;
      if (image_timestamp_us <= last_image_timestamp_us)
      {
        ClearPendingImage();
        ClearPendingProbe();
        ResetLockedRelation();
        ResetCadenceObservation();
        SetObservingState();
        continue;
      }

      const uint64_t image_gap_sensor_us = image_timestamp_us - last_image_timestamp_us;
      if (cadence_update == CadenceUpdate::BROKEN)
      {
        RememberObservedImage(image.sample);
        ClearPendingImage();
        continue;
      }

      const bool probe_pending = probe_pending_;
      ImageDecision decision = ImageDecision::REJECT;
      if (image.sync_candidate_valid)
      {
        decision = ResumePendingMatch(image);
      }
      else if (probe_pending)
      {
        decision = IsProbeImageGap(image_gap_sensor_us)
                       ? TryLockFromProbe(image, image_gap_sensor_us)
                       : ImageDecision::REJECT;
      }
      else if (IsSynced())
      {
        decision = IsNormalImageGap(image_gap_sensor_us)
                       ? TryProcessSyncedImage(image, image_gap_sensor_us)
                       : ImageDecision::REJECT;
      }
      else
      {
        RecordStableImageAnchor(image.sample);
        RememberObservedImage(image.sample);
        ClearPendingImage();
        MaybeSendProbe();
        continue;
      }

      if (decision == ImageDecision::WAIT)
      {
        break;
      }

      if (decision == ImageDecision::ACCEPT)
      {
        if (probe_pending)
        {
          ClearPendingProbe();
        }
        else
        {
          RecordStableImageAnchor(image.sample);
        }
        RememberObservedImage(image.sample);
        ClearPendingImage();
        continue;
      }

      reject_tags_.push_back(image.tag);
      if (probe_pending)
      {
        ClearPendingProbe();
      }
      ClearPendingImage();
      ResetSyncTracking();
      MaybeSendProbe();
    }
  }

  ImageDecision PublishMatchedImage(const PendingImageState& image, const SyncMatch& match)
  {
    if (NeedMoreImuForOffset(*match.sync_imu, offset_us_))
    {
      return ImageDecision::WAIT;
    }

    const TestImu* final_imu = FindFinalImu(*match.sync_imu, offset_us_);
    if (final_imu == nullptr)
    {
      return ImageDecision::REJECT;
    }

    published_image_tags_.push_back(image.tag);
    published_sync_imu_timestamps_.push_back(match.sync_imu->sensor_timestamp_us);
    published_final_imu_timestamps_.push_back(final_imu->sensor_timestamp_us);
    published_accl_timestamps_.push_back(final_imu->accl_timestamp_us);
    published_quat_timestamps_.push_back(final_imu->quat_timestamp_us);
    relation_.sync_imu_sensor_period_us = match.next_sync_sensor_period_us;
    relation_.last_image_sensor_timestamp_us = image.sample.sensor_timestamp_us;
    relation_.last_sync_imu_sensor_timestamp_us = match.sync_imu->sensor_timestamp_us;
    ObserveGoodFrame(lock_state_);
    return ImageDecision::ACCEPT;
  }

  ImageDecision PublishOrRememberMatch(PendingImageState& image, const SyncMatch& match)
  {
    const ImageDecision decision = PublishMatchedImage(image, match);
    if (decision == ImageDecision::WAIT)
    {
      image.sync_candidate_valid = true;
      image.sync_candidate_sensor_timestamp_us = match.sync_imu->sensor_timestamp_us;
      image.sync_candidate_period_us = match.next_sync_sensor_period_us;
      return decision;
    }

    image.sync_candidate_valid = false;
    image.sync_candidate_sensor_timestamp_us = 0;
    image.sync_candidate_period_us = 0;
    return decision;
  }

  ImageDecision ResumePendingMatch(PendingImageState& image)
  {
    if (!image.sync_candidate_valid)
    {
      return ImageDecision::REJECT;
    }

    const TestImu* sync_imu =
        FindBySensorTimestamp(imu_history_, image.sync_candidate_sensor_timestamp_us, 0);
    if (sync_imu == nullptr ||
        sync_imu->sensor_timestamp_us != image.sync_candidate_sensor_timestamp_us)
    {
      return ImageDecision::REJECT;
    }

    return PublishOrRememberMatch(
        image, SyncMatch{
                   .sync_imu = sync_imu,
                   .next_sync_sensor_period_us = image.sync_candidate_period_us,
               });
  }

  SyncMatch BuildTrackedSyncMatch(const TestImu& sync_imu,
                                  uint64_t expected_sync_sensor_gap_us,
                                  uint64_t sensor_gap_tolerance_us) const
  {
    if (sync_imu.sensor_timestamp_us <= relation_.last_sync_imu_sensor_timestamp_us)
    {
      return {};
    }

    const uint64_t sync_sensor_gap_us =
        sync_imu.sensor_timestamp_us - relation_.last_sync_imu_sensor_timestamp_us;
    if (AbsDiffUs(sync_sensor_gap_us, expected_sync_sensor_gap_us) > sensor_gap_tolerance_us)
    {
      return {};
    }

    return SyncMatch{
        .sync_imu = &sync_imu,
        .next_sync_sensor_period_us = sync_sensor_gap_us,
    };
  }

  SyncMatch SelectTrackedSyncMatch(uint64_t expected_sync_sensor_gap_us,
                                   uint64_t sensor_gap_tolerance_us) const
  {
    const uint64_t expected_sync_sensor_timestamp_us =
        relation_.last_sync_imu_sensor_timestamp_us + expected_sync_sensor_gap_us;
    const TestImu* predicted_sync_imu =
        FindBySensorTimestamp(imu_history_, expected_sync_sensor_timestamp_us,
                              sensor_gap_tolerance_us);
    return predicted_sync_imu != nullptr
               ? BuildTrackedSyncMatch(
                     *predicted_sync_imu, expected_sync_sensor_gap_us, sensor_gap_tolerance_us)
               : SyncMatch{};
  }

  SyncMatch SelectProbeSyncMatch(uint64_t expected_probe_sensor_gap_us,
                                 uint64_t sensor_gap_tolerance_us) const
  {
    SyncMatch best_match{};
    uint64_t best_sensor_gap_error_us = UINT64_MAX;
    uint64_t best_sync_timestamp_us = 0;

    for (size_t i = imu_history_.Size(); i > 0; --i)
    {
      const TestImu& sync_imu = imu_history_[i - 1];
      if (sync_imu.sensor_timestamp_us < expected_probe_sensor_gap_us)
      {
        continue;
      }

      const uint64_t prev_target_sensor_timestamp_us =
          sync_imu.sensor_timestamp_us - expected_probe_sensor_gap_us;
      const TestImu* prev_sync_imu =
          FindBySensorTimestamp(imu_history_, prev_target_sensor_timestamp_us,
                                sensor_gap_tolerance_us);
      if (prev_sync_imu == nullptr ||
          sync_imu.sensor_timestamp_us <= prev_sync_imu->sensor_timestamp_us)
      {
        continue;
      }

      const uint64_t sync_sensor_gap_us =
          sync_imu.sensor_timestamp_us - prev_sync_imu->sensor_timestamp_us;
      const uint64_t sensor_gap_error_us =
          AbsDiffUs(sync_sensor_gap_us, expected_probe_sensor_gap_us);
      const bool better_match =
          !best_match.Valid() ||
          sensor_gap_error_us < best_sensor_gap_error_us ||
          (sensor_gap_error_us == best_sensor_gap_error_us &&
           sync_imu.sensor_timestamp_us > best_sync_timestamp_us);
      if (sync_sensor_gap_us < 2 || !better_match)
      {
        continue;
      }

      best_match = SyncMatch{
          .sync_imu = &sync_imu,
          .next_sync_sensor_period_us = sync_sensor_gap_us / 2ULL,
      };
      best_sensor_gap_error_us = sensor_gap_error_us;
      best_sync_timestamp_us = sync_imu.sensor_timestamp_us;
    }

    return best_match;
  }

  ImageDecision TryLockFromProbe(PendingImageState& probe, uint64_t image_gap_sensor_us)
  {
    if (!last_normal_image_.valid || !IsProbeImageGap(image_gap_sensor_us))
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
    if (!ImuHistorySpanReady(expected_probe_sensor_gap_us))
    {
      return ImageDecision::WAIT;
    }

    const SyncMatch match =
        SelectProbeSyncMatch(expected_probe_sensor_gap_us, sensor_gap_tolerance_us);
    if (!match.Valid())
    {
      return ImageDecision::REJECT;
    }

    return PublishOrRememberMatch(probe, match);
  }

  ImageDecision TryProcessSyncedImage(PendingImageState& image,
                                      uint64_t image_gap_sensor_us)
  {
    const uint64_t image_timestamp_us = image.sample.sensor_timestamp_us;
    if (relation_.last_image_sensor_timestamp_us == 0 ||
        relation_.last_sync_imu_sensor_timestamp_us == 0)
    {
      return ImageDecision::REJECT;
    }

    if (image_timestamp_us <= relation_.last_image_sensor_timestamp_us ||
        !IsNormalImageGap(image_gap_sensor_us))
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
    const uint64_t expected_sync_sensor_timestamp_us =
        relation_.last_sync_imu_sensor_timestamp_us + expected_sync_sensor_gap_us;
    if (!ImuHistoryReached(expected_sync_sensor_timestamp_us))
    {
      return ImageDecision::WAIT;
    }

    const SyncMatch match =
        SelectTrackedSyncMatch(expected_sync_sensor_gap_us, sensor_gap_tolerance_us);
    return match.Valid() ? PublishOrRememberMatch(image, match) : ImageDecision::REJECT;
  }

  const TestImu* FindFinalImu(const TestImu& sync_imu, int32_t offset_us) const
  {
    const uint64_t target_timestamp_us = ApplyOffsetUs(sync_imu.sensor_timestamp_us, offset_us);
    return FindBySensorTimestamp(imu_history_, target_timestamp_us,
                                 OffsetSearchToleranceUs(relation_.last_imu_sensor_period_us,
                                                         EstimatedStrideSamples()));
  }

  bool NeedMoreImuForOffset(const TestImu& sync_imu, int32_t offset_us) const
  {
    if (imu_history_.Empty())
    {
      return true;
    }

    const uint64_t target_timestamp_us = ApplyOffsetUs(sync_imu.sensor_timestamp_us, offset_us);
    return target_timestamp_us > imu_history_.Back().sensor_timestamp_us;
  }

 private:
  Ring<QueuedTestGyro, queue_limit> gyro_ingress_{};
  Ring<QueuedTestAccl, queue_limit> accl_ingress_{};
  Ring<QueuedTestQuat, queue_limit> quat_ingress_{};
  Ring<QueuedTestImage, queue_limit> image_ingress_{};

  Ring<QueuedTestGyro, queue_limit> pending_gyros_{};
  Ring<QueuedTestAccl, queue_limit> pending_accls_{};
  Ring<QueuedTestQuat, queue_limit> pending_quats_{};
  Ring<QueuedTestImage, queue_limit> pending_images_{};
  Ring<TestImu, queue_limit> imu_history_{};

  int32_t offset_us_{0};
  bool overflowed_{false};
  bool probe_pending_{false};
  uint32_t probe_sent_count_{0};
  SyncLockState lock_state_{};
  CadenceObservation cadence_{};
  SyncRelation relation_{};
  ImageReference last_observed_image_{};
  ImageReference last_normal_image_{};
  PendingImageState pending_image_{};

  std::vector<uint32_t> published_image_tags_{};
  std::vector<uint64_t> published_sync_imu_timestamps_{};
  std::vector<uint64_t> published_final_imu_timestamps_{};
  std::vector<uint64_t> published_accl_timestamps_{};
  std::vector<uint64_t> published_quat_timestamps_{};
  std::vector<uint32_t> reject_tags_{};
};

class StreamDriver
{
 public:
  explicit StreamDriver(SequenceHarness& harness, uint64_t imu_period_us = 2000ULL,
                        uint64_t image_period_us = 10000ULL, uint32_t image_step_stride = 5U)
      : harness_(harness),
        imu_period_us_(imu_period_us),
        image_period_us_(image_period_us),
        image_step_stride_(image_step_stride),
        next_imu_timestamp_us_(imu_period_us),
        next_image_timestamp_us_(image_period_us),
        next_image_step_(image_step_stride),
        scheduled_gap_us_(image_period_us),
        scheduled_step_gap_(image_step_stride)
  {
  }

  void EmitNextImage(uint32_t imu_after_count = 1)
  {
    const uint64_t current_image_timestamp_us = next_image_timestamp_us_;
    const uint32_t current_image_step = next_image_step_;
    const uint32_t current_image_tag = next_image_tag_++;

    PushImuUpTo(current_image_timestamp_us);
    PushExtraImu(imu_after_count);
    harness_.PushImage(current_image_timestamp_us, current_image_step, current_image_tag);
    harness_.Drain();
    UpdateProbeArm();
    last_image_timestamp_us_ = current_image_timestamp_us;
    last_image_step_ = current_image_step;

    next_image_timestamp_us_ = current_image_timestamp_us + scheduled_gap_us_;
    next_image_step_ = current_image_step + scheduled_step_gap_;
    ConsumeDefaultSchedule();
  }

  void OverrideNextGap(uint64_t image_gap_us, uint32_t image_step_gap)
  {
    scheduled_gap_us_ = image_gap_us;
    scheduled_step_gap_ = image_step_gap;
    custom_gap_pending_ = true;
  }

  void ForceNextImageGap(uint64_t image_gap_us, uint32_t image_step_gap)
  {
    next_image_timestamp_us_ = last_image_timestamp_us_ + image_gap_us;
    next_image_step_ = last_image_step_ + image_step_gap;
  }

  uint64_t ImagePeriodUs() const { return image_period_us_; }

  uint64_t ImuPeriodUs() const { return imu_period_us_; }

  uint32_t ImageStepStride() const { return image_step_stride_; }

 private:
  void PushImuUpTo(uint64_t inclusive_timestamp_us)
  {
    while (next_imu_timestamp_us_ <= inclusive_timestamp_us)
    {
      PushRawImu(next_imu_timestamp_us_);
      next_imu_timestamp_us_ += imu_period_us_;
    }
  }

  void PushExtraImu(uint32_t count)
  {
    for (uint32_t i = 0; i < count; ++i)
    {
      PushRawImu(next_imu_timestamp_us_);
      next_imu_timestamp_us_ += imu_period_us_;
    }
  }

  void PushRawImu(uint64_t gyro_timestamp_us)
  {
    harness_.PushGyro(gyro_timestamp_us);
    harness_.PushAccl(gyro_timestamp_us);
    harness_.PushQuat(gyro_timestamp_us);
  }

  void UpdateProbeArm()
  {
    if (harness_.ProbeSentCount() <= last_probe_sent_count_)
    {
      return;
    }

    last_probe_sent_count_ = harness_.ProbeSentCount();
    if (!custom_gap_pending_)
    {
      scheduled_gap_us_ = image_period_us_ * 2ULL;
      scheduled_step_gap_ = image_step_stride_ * 2U;
    }
  }

  void ConsumeDefaultSchedule()
  {
    scheduled_gap_us_ = image_period_us_;
    scheduled_step_gap_ = image_step_stride_;
    custom_gap_pending_ = false;
  }

 private:
  SequenceHarness& harness_;
  uint64_t imu_period_us_{};
  uint64_t image_period_us_{};
  uint32_t image_step_stride_{};
  uint64_t next_imu_timestamp_us_{0};
  uint64_t next_image_timestamp_us_{0};
  uint32_t next_image_tag_{1};
  uint32_t next_image_step_{0};
  uint64_t last_image_timestamp_us_{0};
  uint32_t last_image_step_{0};
  uint64_t scheduled_gap_us_{0};
  uint32_t scheduled_step_gap_{0};
  uint32_t last_probe_sent_count_{0};
  bool custom_gap_pending_{false};
};

[[noreturn]] void Fail(const std::string& message)
{
  throw std::runtime_error(message);
}

void Expect(bool condition, const std::string& message)
{
  if (!condition)
  {
    Fail(message);
  }
}

template <typename Actual, typename Expected>
void ExpectEqual(const Actual& actual, const Expected& expected, const std::string& message)
{
  if (!(actual == expected))
  {
    std::ostringstream oss;
    oss << message;
    if constexpr (std::is_enum_v<Actual> && std::is_enum_v<Expected>)
    {
      oss << " actual=" << static_cast<int>(actual)
          << " expected=" << static_cast<int>(expected);
    }
    else if constexpr (std::is_enum_v<Actual>)
    {
      oss << " actual=" << static_cast<int>(actual) << " expected=" << expected;
    }
    else if constexpr (std::is_enum_v<Expected>)
    {
      oss << " actual=" << actual << " expected=" << static_cast<int>(expected);
    }
    else
    {
      oss << " actual=" << actual << " expected=" << expected;
    }
    Fail(oss.str());
  }
}

template <typename T>
std::string JoinVector(const std::vector<T>& values)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < values.size(); ++i)
  {
    if (i != 0)
    {
      oss << ", ";
    }
    oss << values[i];
  }
  oss << "]";
  return oss.str();
}

template <typename T>
void ExpectVectorEqual(const std::vector<T>& actual, const std::vector<T>& expected,
                       const std::string& message)
{
  if (actual != expected)
  {
    std::ostringstream oss;
    oss << message << " actual=" << JoinVector(actual)
        << " expected=" << JoinVector(expected);
    Fail(oss.str());
  }
}

void ExpectArithmeticProgression(const std::vector<uint64_t>& values, uint64_t period_us,
                                 const std::string& message)
{
  for (size_t i = 1; i < values.size(); ++i)
  {
    if (values[i] - values[i - 1] != period_us)
    {
      std::ostringstream oss;
      oss << message << " values=" << JoinVector(values)
          << " expected_period=" << period_us;
      Fail(oss.str());
    }
  }
}

void EmitStableWarmup(StreamDriver& driver, size_t image_count, uint32_t imu_after_count = 1)
{
  for (size_t i = 0; i < image_count; ++i)
  {
    driver.EmitNextImage(imu_after_count);
  }
}

void TestStableLockAndTrack()
{
  SequenceHarness harness;
  StreamDriver driver(harness);

  EmitStableWarmup(driver, 8, 1);

  if (harness.ProbeSentCount() != 1U)
  {
    std::ostringstream oss;
    oss << "稳定序列 probe 次数异常"
        << " probe_count=" << harness.ProbeSentCount()
        << " state=" << static_cast<int>(harness.CurrentState())
        << " published_tags=" << JoinVector(harness.PublishedImageTags())
        << " reject_tags=" << JoinVector(harness.RejectTags());
    Fail(oss.str());
  }
  ExpectEqual(harness.ProbeSentCount(), 1U, "稳定序列应只发一次 probe");
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "稳定序列应进入 SYNCED");
  ExpectVectorEqual(harness.PublishedImageTags(), std::vector<uint32_t>({4, 5, 6, 7, 8}),
                    "稳定序列应从 probe 图像开始连续发布");
  ExpectArithmeticProgression(harness.PublishedSyncImuTimestamps(), 10000ULL,
                              "稳态同步 imu 时间戳应保持固定步长");
  ExpectVectorEqual(harness.PublishedFinalImuTimestamps(),
                    harness.PublishedSyncImuTimestamps(),
                    "零 offset 时最终 imu 应等于同步 imu");
}

void TestPositiveOffsetSelectsLaterImu()
{
  SequenceHarness harness;
  harness.SetOffsetUs(2000);
  StreamDriver driver(harness);

  EmitStableWarmup(driver, 9, 2);

  const auto& sync_timestamps = harness.PublishedSyncImuTimestamps();
  const auto& final_timestamps = harness.PublishedFinalImuTimestamps();
  Expect(!sync_timestamps.empty(), "正 offset 场景至少应发布一帧");
  ExpectEqual(sync_timestamps.size(), final_timestamps.size(), "同步 imu 与最终 imu 数量必须一致");
  for (size_t i = 0; i < sync_timestamps.size(); ++i)
  {
    ExpectEqual(final_timestamps[i], sync_timestamps[i] + 2000ULL,
                "正 offset 应选择更晚的 imu");
  }
}

void TestNegativeOffsetSelectsEarlierImu()
{
  SequenceHarness harness;
  harness.SetOffsetUs(-2000);
  StreamDriver driver(harness);

  EmitStableWarmup(driver, 8, 1);

  const auto& sync_timestamps = harness.PublishedSyncImuTimestamps();
  const auto& final_timestamps = harness.PublishedFinalImuTimestamps();
  Expect(!sync_timestamps.empty(), "负 offset 场景至少应发布一帧");
  ExpectEqual(sync_timestamps.size(), final_timestamps.size(), "同步 imu 与最终 imu 数量必须一致");
  for (size_t i = 0; i < sync_timestamps.size(); ++i)
  {
    ExpectEqual(sync_timestamps[i], final_timestamps[i] + 2000ULL,
                "负 offset 应选择更早的 imu");
  }
}

void TestProbeWaitsForOffsetTargetImu()
{
  SequenceHarness harness;
  harness.SetOffsetUs(22000);
  StreamDriver driver(harness);

  EmitStableWarmup(driver, 3, 1);
  ExpectEqual(harness.ProbeSentCount(), 1U, "第三张稳定图像后应已发 probe");

  driver.EmitNextImage(0);
  Expect(harness.PublishedImageTags().empty(), "offset 目标 imu 未到时 probe 图像不应立即发布");

  for (int i = 0; i < 4 && harness.PublishedImageTags().empty(); ++i)
  {
    driver.EmitNextImage(1);
  }
  Expect(!harness.PublishedImageTags().empty(), "补到足够的 offset 目标 imu 后 probe 图像应发布");
  ExpectEqual(harness.PublishedImageTags().front(), 4U, "补齐后应优先发布等待中的 probe 图像");
}

void TestTrackedImageWaitsForExpectedImu()
{
  SequenceHarness harness;
  StreamDriver driver(harness);

  EmitStableWarmup(driver, 5, 1);
  ExpectVectorEqual(harness.PublishedImageTags(), std::vector<uint32_t>({4, 5}),
                    "进入稳态前两帧应已发布");

  harness.SetOffsetUs(12000);
  driver.EmitNextImage(0);
  ExpectVectorEqual(harness.PublishedImageTags(), std::vector<uint32_t>({4, 5}),
                    "缺少 offset 目标 imu 时当前普通图像应继续等待");

  for (int i = 0; i < 4 && harness.PublishedImageTags().size() < 4; ++i)
  {
    driver.EmitNextImage(1);
  }
  Expect(harness.PublishedImageTags().size() >= 4,
         "补到足够的 offset 目标 imu 后应继续发布等待中的普通图像");
  ExpectVectorEqual(std::vector<uint32_t>(harness.PublishedImageTags().begin(),
                                          harness.PublishedImageTags().begin() + 4),
                    std::vector<uint32_t>({4, 5, 6, 7}),
                    "补齐后应先补发等待图像，再按顺序继续处理后续图像");
}

void TestBadProbeGapResetsAndRearms()
{
  SequenceHarness harness;
  StreamDriver driver(harness);

  EmitStableWarmup(driver, 3, 1);
  ExpectEqual(harness.ProbeSentCount(), 1U, "应先发出第一次 probe");

  driver.ForceNextImageGap(driver.ImagePeriodUs(), driver.ImageStepStride());
  driver.EmitNextImage(1);
  ExpectVectorEqual(harness.RejectTags(), std::vector<uint32_t>({4}),
                    "错误 probe gap 应拒绝当前图像");
  ExpectEqual(harness.CurrentState(), SyncState::OBSERVING, "错误 probe gap 后应回到 OBSERVING");

  EmitStableWarmup(driver, 3, 1);
  ExpectEqual(harness.ProbeSentCount(), 2U, "重置后应重新发 probe");
}

void TestBadTrackedGapResetsAndRelocks()
{
  SequenceHarness harness;
  StreamDriver driver(harness);

  EmitStableWarmup(driver, 6, 1);
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "进入异常 gap 之前应已同步");

  driver.ForceNextImageGap(driver.ImagePeriodUs() * 2ULL, driver.ImageStepStride() * 2U);
  driver.EmitNextImage(1);
  ExpectVectorEqual(harness.RejectTags(), std::vector<uint32_t>({}),
                    "稳态下错误图像 gap 会先按 cadence 断裂直接重置");
  ExpectEqual(harness.CurrentState(), SyncState::OBSERVING, "错误普通图像 gap 后应重置");

  EmitStableWarmup(driver, 4, 1);
  ExpectEqual(harness.ProbeSentCount(), 2U, "重置后应重新发 probe");
}

void TestRawCadenceBreakResetsAndRelocks()
{
  SequenceHarness harness;
  StreamDriver driver(harness);

  EmitStableWarmup(driver, 6, 1);
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "原始 imu 异常前应已同步");

  harness.PushGyro(40000ULL + 3500ULL);
  harness.PushAccl(40000ULL + 3500ULL);
  harness.PushQuat(40000ULL + 3500ULL);
  driver.EmitNextImage(1);
  ExpectEqual(harness.CurrentState(), SyncState::OBSERVING, "原始 imu 节拍断裂后应回 OBSERVING");

  EmitStableWarmup(driver, 4, 1);
  ExpectEqual(harness.ProbeSentCount(), 2U, "原始 imu 恢复后应重新发 probe");
}

void TestNearestRawImuChannelSelection()
{
  FixedRingBuffer<QueuedTestGyro, 8> gyros{};
  FixedRingBuffer<QueuedTestAccl, 8> accls{};
  FixedRingBuffer<QueuedTestQuat, 8> quats{};
  FixedRingBuffer<TestImu, 8> history{};
  uint64_t last_imu_sensor_period_us = 2000ULL;

  history.PushBackDropOldest({.sensor_timestamp_us = 8000ULL,
                              .accl_timestamp_us = 8000ULL,
                              .quat_timestamp_us = 8000ULL});
  gyros.PushBackDropOldest({{10000ULL}});
  accls.PushBackDropOldest({{9200ULL}});
  accls.PushBackDropOldest({{10800ULL}});
  quats.PushBackDropOldest({{9600ULL}});
  quats.PushBackDropOldest({{11100ULL}});

  const bool built = TryBuildFrontImu(
      gyros, accls, quats, history, last_imu_sensor_period_us, 8,
      [](const TestGyro& gyro, const TestAccl& accl, const TestQuat& quat)
      {
        return TestImu{
            .sensor_timestamp_us = gyro.sensor_timestamp_us,
            .accl_timestamp_us = accl.sensor_timestamp_us,
            .quat_timestamp_us = quat.sensor_timestamp_us,
        };
      });

  Expect(built, "半周期最近匹配场景应成功组装 imu");
  ExpectEqual(history.Back().sensor_timestamp_us, 10000ULL, "应按 gyro 时间戳组装 imu");
  ExpectEqual(history.Back().accl_timestamp_us, 10800ULL,
              "前后等距时 accl 应优先选择更晚样本");
  ExpectEqual(history.Back().quat_timestamp_us, 9600ULL,
              "quat 应选择窗口内距离最近的样本");
}

void TestCompositeRecoverySequence()
{
  SequenceHarness harness;
  StreamDriver driver(harness);

  EmitStableWarmup(driver, 3, 1);
  driver.ForceNextImageGap(driver.ImagePeriodUs(), driver.ImageStepStride());
  driver.EmitNextImage(1);
  ExpectVectorEqual(harness.RejectTags(), std::vector<uint32_t>({4}),
                    "复合场景第一步先走错误 probe gap");

  EmitStableWarmup(driver, 6, 1);
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "复合场景中段应先重新同步");

  harness.PushGyro(90000ULL + 3500ULL);
  harness.PushAccl(90000ULL + 3500ULL);
  harness.PushQuat(90000ULL + 3500ULL);
  driver.EmitNextImage(1);
  ExpectEqual(harness.CurrentState(), SyncState::OBSERVING, "复合场景中的 raw break 应触发重置");

  EmitStableWarmup(driver, 6, 1);
  ExpectEqual(harness.ProbeSentCount(), 3U, "复合场景最后一段应再次发 probe");
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "复合场景最后应重新同步");
}

using TestFn = void (*)();

struct NamedTest
{
  const char* name;
  TestFn fn;
};

}  // namespace

int main()
{
  const std::vector<NamedTest> tests = {
      {"正常/稳定序列锁定并连续跟踪", TestStableLockAndTrack},
      {"正常/正offset选择更晚imu", TestPositiveOffsetSelectsLaterImu},
      {"正常/负offset选择更早imu", TestNegativeOffsetSelectsEarlierImu},
      {"正常/probe缺offset目标imu时等待", TestProbeWaitsForOffsetTargetImu},
      {"正常/稳态缺offset目标imu时等待", TestTrackedImageWaitsForExpectedImu},
      {"错误/probe图像周期错误后重置并重发", TestBadProbeGapResetsAndRearms},
      {"错误/稳态图像周期错误后重置并重锁", TestBadTrackedGapResetsAndRelocks},
      {"正确/原始imu半周期最近匹配", TestNearestRawImuChannelSelection},
      {"复合/等待恢复与重锁串联", TestCompositeRecoverySequence},
  };

  try
  {
    for (const auto& test : tests)
    {
      test.fn();
      std::cout << "[PASS] " << test.name << '\n';
    }
    return 0;
  }
  catch (const std::exception& ex)
  {
    std::cerr << "[FAIL] " << ex.what() << '\n';
    return 1;
  }
}
