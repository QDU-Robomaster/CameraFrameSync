#include <cstdint>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "../camera_frame_sync_core.hpp"

namespace
{
using namespace CameraFrameSyncCore;

struct Raw3
{
  uint64_t sensor_timestamp_us{};
};

struct Raw4
{
  uint64_t sensor_timestamp_us{};
};

struct ImageEvent
{
  uint64_t sensor_timestamp_us{};
  uint32_t tag{};
};

struct Imu
{
  uint64_t sensor_timestamp_us{};
  uint64_t accl_timestamp_us{};
  uint64_t quat_timestamp_us{};
};

enum class SyncMode : uint8_t
{
  RAW_PROBE = 0,
  LATEST_IMU = 1,
};

class SequenceHarness
{
 public:
  template <typename T, size_t Capacity>
  using Ring = SampleHistory<T, Capacity>;

  void SetMode(SyncMode mode)
  {
    mode_ = mode;
    ResetRuntime();
  }

  void SetOffsetUs(int32_t offset_us) { offset_us_ = offset_us; }

  void PushGyro(uint64_t timestamp_us) { gyros_.PushBackDropOldest({timestamp_us}); }
  void PushAccl(uint64_t timestamp_us) { accls_.PushBackDropOldest({timestamp_us}); }
  void PushQuat(uint64_t timestamp_us) { quats_.PushBackDropOldest({timestamp_us}); }
  void PushRawImu(uint64_t timestamp_us)
  {
    PushGyro(timestamp_us);
    PushAccl(timestamp_us);
    PushQuat(timestamp_us);
  }

  void PushImage(uint64_t timestamp_us, uint32_t tag)
  {
    images_.PushBackDropOldest({timestamp_us, tag});
  }

  void PushSyncAck(uint32_t seq, uint64_t timestamp_us)
  {
    if (active_probe_seq_ == 0 || seq != active_probe_seq_)
    {
      return;
    }
    if (probe_ack_seq_ == seq)
    {
      return;
    }

    probe_ack_timestamp_us_ = timestamp_us;
    probe_ack_seq_ = seq;
  }

  void Drain()
  {
    AssembleImuHistory();
    ProcessImages();
  }

  void DropImage(uint64_t timestamp_us)
  {
    AssembleImuHistory();
    ObserveDroppedImage(timestamp_us);
    ProcessImages();
  }

  SyncState State() const { return state_; }
  uint32_t ProbeSentCount() const { return probe_sent_count_; }
  uint32_t LastProbeSeq() const { return last_probe_seq_; }
  uint64_t ImagePeriodUs() const { return periods_.image_us; }
  uint64_t ImuPeriodUs() const { return periods_.imu_us; }

  const std::vector<uint32_t>& PublishedTags() const { return published_tags_; }
  const std::vector<uint64_t>& SyncImuTimestamps() const { return sync_imu_timestamps_; }
  const std::vector<uint64_t>& FinalImuTimestamps() const { return final_imu_timestamps_; }
  const std::vector<uint32_t>& ResetTags() const { return reset_tags_; }
  const std::vector<uint64_t>& HistoryTimestamps() const { return history_timestamps_; }

 private:
  struct PendingMatch
  {
    bool valid{false};
    uint64_t imu_timestamp_us{0};
    uint64_t period_us{0};
  };

  struct PendingFrame
  {
    bool valid{false};
    ImageEvent event{};
    bool cadence_observed{false};
    PendingMatch match{};
  };

  struct ObservedPeriods
  {
    uint64_t image_us{0};
    uint64_t imu_us{0};
  };

  struct LockedSync
  {
    uint64_t period_us{0};
    uint64_t last_imu_timestamp_us{0};
  };

  struct Match
  {
    const Imu* imu{nullptr};
    uint64_t period_us{0};
  };

  enum class Decision : uint8_t
  {
    WAIT = 0,
    DONE = 1,
    RESET = 2,
  };

  static constexpr size_t limit = 64;
  static constexpr uint32_t stable_gaps = 2;
  static constexpr uint64_t imu_tolerance_us = 300;
  static constexpr uint64_t image_tolerance_us = 1500;
  static constexpr uint32_t max_raw_imu_gap_stride = 8;
  static constexpr uint64_t raw_imu_epoch_reset_backward_us = 100000ULL;
  static constexpr uint32_t probe_div = 3;
  static constexpr uint64_t probe_ack_timeout_min_us = 100000;

  void ClearRuntimeState()
  {
    gyros_.Clear();
    accls_.Clear();
    quats_.Clear();
    images_.Clear();
    history_.Clear();
    history_timestamps_.clear();
    pending_frame_ = {};
    image_cadence_ = {};
    imu_cadence_ = {};
    periods_ = {};
    locked_sync_ = {};
    state_ = SyncState::OBSERVING;
    last_image_valid_ = false;
    last_image_timestamp_us_ = 0;
    ClearPendingProbe();
  }

  void ResetRuntime()
  {
    ClearRuntimeState();
    last_probe_seq_ = 0;
    probe_sent_count_ = 0;
  }

  void ResetLock()
  {
    state_ = SyncState::OBSERVING;
    locked_sync_ = {};
    ClearPendingProbe();
  }

  void ResetRawEpoch(const Imu& first_imu)
  {
    ClearRuntimeState();
    AcceptImu(first_imu);
  }

  void ClearPendingProbe()
  {
    active_probe_seq_ = 0;
    probe_ack_seq_ = 0;
    probe_ack_valid_ = false;
    probe_ack_timestamp_us_ = 0;
    probe_start_imu_timestamp_us_ = 0;
  }

  void ResetImageObservation()
  {
    image_cadence_ = {};
    periods_.image_us = 0;
    last_image_valid_ = false;
    last_image_timestamp_us_ = 0;
  }

  void AssembleImuHistory()
  {
    while (TryAssembleOne())
    {
    }
  }

  bool TryAssembleOne()
  {
    if (gyros_.Empty() || accls_.Empty() || quats_.Empty())
    {
      return false;
    }

    const uint64_t gyro_ts = gyros_.Front().sensor_timestamp_us;
    while (!accls_.Empty() && accls_.Front().sensor_timestamp_us < gyro_ts)
    {
      accls_.PopFront();
    }
    while (!quats_.Empty() && quats_.Front().sensor_timestamp_us < gyro_ts)
    {
      quats_.PopFront();
    }
    if (accls_.Empty() || quats_.Empty())
    {
      return false;
    }
    if (accls_.Front().sensor_timestamp_us != gyro_ts ||
        quats_.Front().sensor_timestamp_us != gyro_ts)
    {
      gyros_.PopFront();
      return true;
    }

    const Imu imu{gyro_ts, accls_.Front().sensor_timestamp_us,
                  quats_.Front().sensor_timestamp_us};
    gyros_.PopFront();
    accls_.PopFront();
    quats_.PopFront();

    if (!history_.Empty() && imu.sensor_timestamp_us <= history_.Back().sensor_timestamp_us)
    {
      const uint64_t previous_timestamp_us = history_.Back().sensor_timestamp_us;
      if (imu.sensor_timestamp_us < previous_timestamp_us &&
          previous_timestamp_us - imu.sensor_timestamp_us >=
              raw_imu_epoch_reset_backward_us)
      {
        ResetRawEpoch(imu);
        return true;
      }
      ResetLock();
      return true;
    }

    AcceptImu(imu);
    return true;
  }

  void AcceptImu(const Imu& imu)
  {
    history_.PushBackDropOldest(imu);
    history_timestamps_.push_back(imu.sensor_timestamp_us);
    if (imu_cadence_.stable && imu_cadence_.has_last_timestamp &&
        imu_cadence_.period_us != 0 &&
        imu.sensor_timestamp_us > imu_cadence_.last_timestamp_us)
    {
      const uint64_t gap_us =
          imu.sensor_timestamp_us - imu_cadence_.last_timestamp_us;
      const uint32_t stride = MatchPeriodGapStride(
          gap_us, imu_cadence_.period_us, imu_tolerance_us,
          max_raw_imu_gap_stride);
      if (stride > 1)
      {
        imu_cadence_.last_timestamp_us = imu.sensor_timestamp_us;
        periods_.imu_us = imu_cadence_.period_us;
        return;
      }
    }

    const auto update = ObserveCadence(imu_cadence_, imu.sensor_timestamp_us,
                                       stable_gaps, imu_tolerance_us);
    if (update == CadenceUpdate::BROKEN)
    {
      ResetLock();
    }
    if (imu_cadence_.stable)
    {
      periods_.imu_us = imu_cadence_.period_us;
    }
  }

  void ProcessImages()
  {
    while (pending_frame_.valid || !images_.Empty())
    {
      if (!pending_frame_.valid)
      {
        pending_frame_.valid = true;
        pending_frame_.event = images_.Front();
        images_.PopFront();
      }

      const Decision decision =
          mode_ == SyncMode::LATEST_IMU ? ProcessLatest(pending_frame_)
                                        : ProcessRawProbe(pending_frame_);
      if (decision == Decision::WAIT)
      {
        break;
      }
      if (decision == Decision::RESET)
      {
        reset_tags_.push_back(pending_frame_.event.tag);
        pending_frame_ = {};
        ResetLock();
        continue;
      }
      pending_frame_ = {};
    }
  }

  Decision ProcessLatest(PendingFrame& image)
  {
    if (last_image_valid_ && image.event.sensor_timestamp_us <= last_image_timestamp_us_)
    {
      ResetImageObservation();
      return Decision::DONE;
    }

    const Decision decision = image.match.valid
                                  ? Resume(image)
                                  : PublishOrRemember(image, LatestMatch());
    if (decision != Decision::WAIT)
    {
      RememberImage(image.event.sensor_timestamp_us);
    }
    return decision;
  }

  Decision ProcessRawProbe(PendingFrame& image)
  {
    if (image.match.valid)
    {
      return Resume(image);
    }
    if (!last_image_valid_)
    {
      ObserveImage(image);
      RememberImage(image.event.sensor_timestamp_us);
      MaybeStartProbe();
      return Decision::DONE;
    }

    const uint64_t image_ts = image.event.sensor_timestamp_us;
    if (image_ts <= last_image_timestamp_us_)
    {
      ResetImageObservation();
      ResetLock();
      ObserveImage(image);
      RememberImage(image_ts);
      return Decision::DONE;
    }

    const uint64_t image_gap = image_ts - last_image_timestamp_us_;
    if (state_ == SyncState::PROBE_SENT &&
        (IsProbeGap(image_gap) ||
         (probe_ack_seq_ == last_probe_seq_ && !IsNormalGap(image_gap))))
    {
      if (!image.cadence_observed)
      {
        image_cadence_.last_timestamp_us = image_ts;
        image.cadence_observed = true;
      }
      return TryProbeImage(image);
    }

    const uint32_t image_gap_stride =
        state_ == SyncState::SYNCED ? MatchNormalGapStride(image_gap) : 0;
    auto update = CadenceUpdate::STABLE;
    if (!image.cadence_observed)
    {
      if (image_gap_stride > 1)
      {
        image_cadence_.last_timestamp_us = image_ts;
        image.cadence_observed = true;
      }
      else
      {
        update = ObserveImage(image);
      }
    }
    if (update == CadenceUpdate::BROKEN)
    {
      ResetLock();
      RememberImage(image_ts);
      return Decision::DONE;
    }

    switch (state_)
    {
      case SyncState::OBSERVING:
        RememberImage(image_ts);
        MaybeStartProbe();
        return Decision::DONE;
      case SyncState::PROBE_SENT:
        if (IsNormalGap(image_gap))
        {
          if (ProbeTimedOut())
          {
            ResetLock();
            RememberImage(image_ts);
            MaybeStartProbe();
            return Decision::DONE;
          }
          RememberImage(image_ts);
          return Decision::DONE;
        }
        return Decision::RESET;
      case SyncState::SYNCED:
      {
        return image_gap_stride != 0 ? TrySyncedImage(image, image_gap_stride)
                                     : Decision::RESET;
      }
    }
    return Decision::RESET;
  }

  CadenceUpdate ObserveImage(PendingFrame& image)
  {
    image.cadence_observed = true;
    const auto update = ObserveCadence(image_cadence_, image.event.sensor_timestamp_us,
                                       stable_gaps, image_tolerance_us);
    if (image_cadence_.stable)
    {
      periods_.image_us = image_cadence_.period_us;
    }
    return update;
  }

  void ObserveDroppedImage(uint64_t image_ts)
  {
    if (!last_image_valid_)
    {
      ObserveCadence(image_cadence_, image_ts, stable_gaps, image_tolerance_us);
      RememberImage(image_ts);
      return;
    }

    if (image_ts <= last_image_timestamp_us_)
    {
      ResetImageObservation();
      ObserveCadence(image_cadence_, image_ts, stable_gaps, image_tolerance_us);
      RememberImage(image_ts);
      return;
    }

    const uint64_t image_gap = image_ts - last_image_timestamp_us_;
    const SyncState old_state = state_;
    const uint32_t image_gap_stride =
        old_state == SyncState::SYNCED ? MatchNormalGapStride(image_gap) : 0;
    const bool normal_gap = image_gap_stride == 1;
    const bool dropped_probe_gap =
        old_state == SyncState::PROBE_SENT &&
        (IsProbeGap(image_gap) ||
         (probe_ack_seq_ == last_probe_seq_ && !normal_gap));

    if (dropped_probe_gap)
    {
      ResetLock();
      ResetImageObservation();
      ObserveCadence(image_cadence_, image_ts, stable_gaps, image_tolerance_us);
      RememberImage(image_ts);
      return;
    }

    auto update = CadenceUpdate::STABLE;
    if (image_gap_stride > 1)
    {
      image_cadence_.last_timestamp_us = image_ts;
    }
    else
    {
      update =
          ObserveCadence(image_cadence_, image_ts, stable_gaps, image_tolerance_us);
    }
    if (image_cadence_.stable)
    {
      periods_.image_us = image_cadence_.period_us;
    }

    if (old_state == SyncState::SYNCED && image_gap_stride != 0)
    {
      locked_sync_.last_imu_timestamp_us +=
          locked_sync_.period_us * static_cast<uint64_t>(image_gap_stride);
      RememberImage(image_ts);
      return;
    }

    if (update == CadenceUpdate::BROKEN)
    {
      ResetLock();
    }
    RememberImage(image_ts);
  }

  void MaybeStartProbe()
  {
    if (mode_ != SyncMode::RAW_PROBE || state_ != SyncState::OBSERVING ||
        !image_cadence_.stable || !imu_cadence_.stable || !last_image_valid_)
    {
      return;
    }

    if (EstimatedSyncPeriod() == 0)
    {
      return;
    }

    state_ = SyncState::PROBE_SENT;
    last_probe_seq_ = next_seq_++;
    if (next_seq_ > UINT8_MAX)
    {
      next_seq_ = 1;
    }
    probe_sent_count_++;
    probe_ack_valid_ = false;
    probe_ack_seq_ = 0;
    probe_ack_timestamp_us_ = 0;
    probe_start_imu_timestamp_us_ =
        history_.Empty() ? 0 : history_.Back().sensor_timestamp_us;
    active_probe_seq_ = last_probe_seq_;
  }

  Decision TryProbeImage(PendingFrame& image)
  {
    if (!probe_ack_valid_ && probe_ack_seq_ == last_probe_seq_)
    {
      probe_ack_valid_ = true;
    }
    if (!probe_ack_valid_)
    {
      if (ProbeTimedOut())
      {
        ResetLock();
        ResetImageObservation();
        ObserveImage(image);
        RememberImage(image.event.sensor_timestamp_us);
        return Decision::DONE;
      }
      return Decision::WAIT;
    }
    if (!HistoryReached(probe_ack_timestamp_us_))
    {
      return Decision::WAIT;
    }

    const Imu* sync_imu = FindBySensorTimestamp(
        history_, probe_ack_timestamp_us_, ImuTimestampToleranceUs(periods_.imu_us));
    if (sync_imu == nullptr)
    {
      return Decision::RESET;
    }
    return PublishOrRemember(image, Match{sync_imu, EstimatedSyncPeriod()});
  }

  Match LatestMatch() const
  {
    if (history_.Empty())
    {
      return {};
    }
    return Match{&history_.Back(), periods_.imu_us != 0 ? periods_.imu_us : 1};
  }

  Decision TrySyncedImage(PendingFrame& image, uint32_t image_gap_stride)
  {
    if (image_gap_stride == 0)
    {
      return Decision::RESET;
    }

    const uint64_t expected =
        locked_sync_.last_imu_timestamp_us +
        locked_sync_.period_us * static_cast<uint64_t>(image_gap_stride);
    if (!HistoryReached(expected))
    {
      return Decision::WAIT;
    }
    const Imu* sync_imu =
        FindBySensorTimestamp(history_, expected, ImuTimestampToleranceUs(periods_.imu_us));
    if (sync_imu == nullptr)
    {
      return Decision::RESET;
    }
    return PublishOrRemember(image, Match{sync_imu, locked_sync_.period_us});
  }

  Decision Resume(PendingFrame& image)
  {
    const Imu* sync_imu =
        FindBySensorTimestamp(history_, image.match.imu_timestamp_us, 0);
    if (sync_imu == nullptr)
    {
      return Decision::RESET;
    }
    return PublishOrRemember(image, Match{sync_imu, image.match.period_us});
  }

  Decision PublishOrRemember(PendingFrame& image, Match match)
  {
    if (match.imu == nullptr || match.period_us == 0)
    {
      return Decision::WAIT;
    }

    const uint64_t final_ts = ApplyOffsetUs(match.imu->sensor_timestamp_us, offset_us_);
    if (!HistoryReached(final_ts))
    {
      image.match.valid = true;
      image.match.imu_timestamp_us = match.imu->sensor_timestamp_us;
      image.match.period_us = match.period_us;
      return Decision::WAIT;
    }

    const Imu* final_imu =
        FindBySensorTimestamp(history_, final_ts, ImuTimestampToleranceUs(periods_.imu_us));
    if (final_imu == nullptr)
    {
      return Decision::RESET;
    }

    published_tags_.push_back(image.event.tag);
    sync_imu_timestamps_.push_back(match.imu->sensor_timestamp_us);
    final_imu_timestamps_.push_back(final_imu->sensor_timestamp_us);
    locked_sync_.period_us = match.period_us;
    locked_sync_.last_imu_timestamp_us = match.imu->sensor_timestamp_us;
    state_ = SyncState::SYNCED;
    ClearPendingProbe();
    RememberImage(image.event.sensor_timestamp_us);
    return Decision::DONE;
  }

  uint64_t EstimatedSyncPeriod() const
  {
    const uint32_t stride =
        EstimateStrideSamples(periods_.image_us, periods_.imu_us);
    return stride == 0 ? 0 : periods_.imu_us * static_cast<uint64_t>(stride);
  }

  uint64_t ProbeTimeoutUs() const
  {
    uint64_t image_period_us = periods_.image_us;
    if (image_period_us == 0)
    {
      image_period_us = EstimatedSyncPeriod();
    }
    if (image_period_us == 0)
    {
      return 0;
    }

    const uint64_t probe_gap_us = ProbeImageGapUs(image_period_us, probe_div);
    uint64_t timeout_us = probe_gap_us + image_period_us * 4ULL;
    if (timeout_us < probe_ack_timeout_min_us)
    {
      timeout_us = probe_ack_timeout_min_us;
    }
    return timeout_us;
  }

  bool ProbeTimedOut() const
  {
    if (state_ != SyncState::PROBE_SENT || probe_start_imu_timestamp_us_ == 0 ||
        history_.Empty())
    {
      return false;
    }

    const uint64_t timeout_us = ProbeTimeoutUs();
    if (timeout_us == 0 ||
        history_.Back().sensor_timestamp_us <= probe_start_imu_timestamp_us_)
    {
      return false;
    }
    return history_.Back().sensor_timestamp_us - probe_start_imu_timestamp_us_ >=
           timeout_us;
  }

  bool HistoryReached(uint64_t timestamp_us) const
  {
    return !history_.Empty() && history_.Back().sensor_timestamp_us >= timestamp_us;
  }

  bool IsNormalGap(uint64_t gap_us) const
  {
    return periods_.image_us != 0 && AbsDiffUs(gap_us, periods_.image_us) <=
                                         ImageGapToleranceUs(periods_.image_us);
  }

  uint32_t MatchNormalGapStride(uint64_t gap_us) const
  {
    return MatchImageGapStride(gap_us, periods_.image_us, 8);
  }

  bool IsProbeGap(uint64_t gap_us) const
  {
    const uint64_t expected = ProbeImageGapUs(periods_.image_us, probe_div);
    return expected != 0 && AbsDiffUs(gap_us, expected) <=
                                ImageGapToleranceUs(periods_.image_us);
  }

  void RememberImage(uint64_t timestamp_us)
  {
    last_image_valid_ = true;
    last_image_timestamp_us_ = timestamp_us;
  }

 private:
  Ring<Raw3, limit> gyros_{};
  Ring<Raw3, limit> accls_{};
  Ring<Raw4, limit> quats_{};
  Ring<ImageEvent, limit> images_{};
  Ring<Imu, limit> history_{};

  SyncMode mode_{SyncMode::RAW_PROBE};
  int32_t offset_us_{0};
  SyncState state_{SyncState::OBSERVING};
  CadenceState image_cadence_{};
  CadenceState imu_cadence_{};
  ObservedPeriods periods_{};
  LockedSync locked_sync_{};
  PendingFrame pending_frame_{};

  bool last_image_valid_{false};
  uint64_t last_image_timestamp_us_{0};
  uint32_t next_seq_{1};
  uint32_t last_probe_seq_{0};
  uint32_t probe_sent_count_{0};
  uint32_t active_probe_seq_{0};
  uint32_t probe_ack_seq_{0};
  bool probe_ack_valid_{false};
  uint64_t probe_ack_timestamp_us_{0};
  uint64_t probe_start_imu_timestamp_us_{0};

  std::vector<uint32_t> published_tags_{};
  std::vector<uint64_t> sync_imu_timestamps_{};
  std::vector<uint64_t> final_imu_timestamps_{};
  std::vector<uint32_t> reset_tags_{};
  std::vector<uint64_t> history_timestamps_{};
};

class StreamDriver
{
 public:
  explicit StreamDriver(SequenceHarness& harness, uint64_t imu_period_us = 2000,
                        uint64_t image_period_us = 10000)
      : harness_(harness),
        imu_period_us_(imu_period_us),
        image_period_us_(image_period_us),
        next_imu_timestamp_us_(imu_period_us),
        next_image_timestamp_us_(image_period_us),
        next_gap_us_(image_period_us)
  {
  }

  void EmitImage(uint32_t imu_after_count = 1)
  {
    const uint64_t image_ts = next_image_timestamp_us_;
    const bool emit_ack = probe_gap_armed_;

    PushRawUntil(image_ts);
    for (uint32_t i = 0; i < imu_after_count; ++i)
    {
      PushRaw(next_imu_timestamp_us_);
      next_imu_timestamp_us_ += imu_period_us_;
    }
    if (emit_ack)
    {
      harness_.PushSyncAck(harness_.LastProbeSeq(), image_ts);
    }
    harness_.PushImage(image_ts, next_tag_++);
    harness_.Drain();

    const uint32_t probe_count = harness_.ProbeSentCount();
    probe_gap_armed_ = probe_count != last_probe_count_;
    last_probe_count_ = probe_count;
    next_gap_us_ = probe_gap_armed_ ? ProbeImageGapUs(image_period_us_, 3)
                                    : image_period_us_;
    next_image_timestamp_us_ = image_ts + next_gap_us_;
  }

  void ForceNextGap(uint64_t gap_us)
  {
    next_image_timestamp_us_ = last_image_timestamp_us_ + gap_us;
    next_gap_us_ = gap_us;
  }

  void RememberLastImage(uint64_t timestamp_us) { last_image_timestamp_us_ = timestamp_us; }
  uint64_t NextImageTimestamp() const { return next_image_timestamp_us_; }
  uint64_t NextImuTimestamp() const { return next_imu_timestamp_us_; }
  uint64_t ImuPeriodUs() const { return imu_period_us_; }
  uint64_t ImagePeriodUs() const { return image_period_us_; }

  void InjectRawGap(uint64_t gap_us)
  {
    const uint64_t timestamp_us = next_imu_timestamp_us_ + gap_us;
    PushRaw(timestamp_us);
    next_imu_timestamp_us_ = timestamp_us + imu_period_us_;
    harness_.Drain();
  }

 private:
  void PushRawUntil(uint64_t timestamp_us)
  {
    while (next_imu_timestamp_us_ <= timestamp_us)
    {
      PushRaw(next_imu_timestamp_us_);
      next_imu_timestamp_us_ += imu_period_us_;
    }
    last_image_timestamp_us_ = timestamp_us;
  }

  void PushRaw(uint64_t timestamp_us) { harness_.PushRawImu(timestamp_us); }

 private:
  SequenceHarness& harness_;
  uint64_t imu_period_us_{};
  uint64_t image_period_us_{};
  uint64_t next_imu_timestamp_us_{};
  uint64_t next_image_timestamp_us_{};
  uint64_t next_gap_us_{};
  uint64_t last_image_timestamp_us_{0};
  uint32_t next_tag_{1};
  uint32_t last_probe_count_{0};
  bool probe_gap_armed_{false};
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
    if constexpr (std::is_enum_v<Actual>)
    {
      oss << " actual=" << static_cast<int>(actual);
    }
    else
    {
      oss << " actual=" << actual;
    }
    if constexpr (std::is_enum_v<Expected>)
    {
      oss << " expected=" << static_cast<int>(expected);
    }
    else
    {
      oss << " expected=" << expected;
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
    Fail(message + " actual=" + JoinVector(actual) + " expected=" + JoinVector(expected));
  }
}

void EmitWarmup(StreamDriver& driver, size_t image_count, uint32_t imu_after_count = 1)
{
  for (size_t i = 0; i < image_count; ++i)
  {
    driver.EmitImage(imu_after_count);
  }
}

void TestRawJoinWaitsForAllChannels()
{
  SequenceHarness harness;

  harness.PushGyro(1000);
  harness.Drain();
  Expect(harness.HistoryTimestamps().empty(), "缺少 accl/quat 时不能消耗 gyro");

  harness.PushAccl(1000);
  harness.PushQuat(1000);
  harness.Drain();
  ExpectVectorEqual(harness.HistoryTimestamps(), std::vector<uint64_t>{1000},
                    "三路 timestamp 相同后应组装 IMU");
}

void TestRawJoinRequiresExactGyroTimestamp()
{
  SequenceHarness harness;

  harness.PushGyro(2000);
  harness.PushAccl(2000);
  harness.PushQuat(2000);
  harness.Drain();
  ExpectVectorEqual(harness.HistoryTimestamps(), std::vector<uint64_t>{2000},
                    "当前 DevC raw 三通道使用同一个 gyro timestamp，应 exact join");

  harness.PushGyro(3000);
  harness.PushAccl(3001);
  harness.PushQuat(3000);
  harness.Drain();
  ExpectVectorEqual(harness.HistoryTimestamps(), std::vector<uint64_t>{2000},
                    "非 exact timestamp 不能拼到当前 gyro");

  harness.PushGyro(4000);
  harness.PushAccl(4000);
  harness.PushQuat(4000);
  harness.Drain();
  ExpectVectorEqual(harness.HistoryTimestamps(), std::vector<uint64_t>{2000, 4000},
                    "丢弃无法匹配的 gyro 后应继续组装后续 raw IMU");
}

void TestLatestModePublishesWithoutProbe()
{
  SequenceHarness harness;
  harness.SetMode(SyncMode::LATEST_IMU);

  harness.PushRawImu(1000);
  harness.PushImage(5000, 1);
  harness.Drain();

  ExpectEqual(harness.ProbeSentCount(), 0U, "LATEST_IMU 不应发送 probe");
  ExpectVectorEqual(harness.PublishedTags(), std::vector<uint32_t>{1},
                    "LATEST_IMU 应直接发布第一张图");
  ExpectVectorEqual(harness.SyncImuTimestamps(), std::vector<uint64_t>{1000},
                    "LATEST_IMU 应使用当前最新完整 IMU");
}

void TestLatestModeWaitsForOffsetTarget()
{
  SequenceHarness harness;
  harness.SetMode(SyncMode::LATEST_IMU);
  harness.SetOffsetUs(1000);

  harness.PushRawImu(1000);
  harness.PushImage(5000, 1);
  harness.Drain();
  Expect(harness.PublishedTags().empty(), "offset 目标 IMU 未到时必须等待");

  harness.PushRawImu(2000);
  harness.Drain();
  ExpectVectorEqual(harness.PublishedTags(), std::vector<uint32_t>{1},
                    "offset 目标 IMU 到达后应发布等待图像");
  ExpectVectorEqual(harness.FinalImuTimestamps(), std::vector<uint64_t>{2000},
                    "正 offset 应选择更晚的 IMU");
}

void TestRawProbeLocksFromCameraSyncAck()
{
  SequenceHarness harness;
  StreamDriver driver(harness);

  EmitWarmup(driver, 8);

  ExpectEqual(harness.ProbeSentCount(), 1U, "稳定后应只发一次 probe");
  ExpectEqual(harness.State(), SyncState::SYNCED, "收到 CameraSync ack 后应进入 SYNCED");
  Expect(!harness.PublishedTags().empty(), "probe 图像应被发布");
  ExpectEqual(harness.SyncImuTimestamps().front(), 60000ULL,
              "第一帧同步 IMU 应来自 CameraSync result 的 timestamp");
}

void TestWrongSeqIsIgnoredUntilCorrectAckArrives()
{
  SequenceHarness harness;

  for (uint64_t t = 2000; t <= 30000; t += 2000)
  {
    harness.PushRawImu(t);
  }
  harness.PushImage(10000, 1);
  harness.Drain();
  harness.PushImage(20000, 2);
  harness.Drain();
  harness.PushImage(30000, 3);
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::PROBE_SENT, "第三张稳定图像后应已发 probe");

  for (uint64_t t = 32000; t <= 60000; t += 2000)
  {
    harness.PushRawImu(t);
  }
  harness.PushSyncAck(harness.LastProbeSeq() + 1U, 60000);
  harness.PushImage(60000, 4);
  harness.Drain();
  Expect(harness.PublishedTags().empty(), "错误 seq 的 ack 必须忽略");

  harness.PushSyncAck(harness.LastProbeSeq(), 60000);
  harness.Drain();
  ExpectVectorEqual(harness.PublishedTags(), std::vector<uint32_t>{4},
                    "正确 seq 到达后应发布等待中的 probe 图像");
}

void TestLostProbeCommandRetriesAndLocks()
{
  SequenceHarness harness;
  uint64_t next_raw = 2000;
  auto push_raw_until = [&](uint64_t timestamp_us) {
    while (next_raw <= timestamp_us)
    {
      harness.PushRawImu(next_raw);
      next_raw += 2000;
    }
  };

  for (uint64_t image_ts = 10000; image_ts <= 30000; image_ts += 10000)
  {
    push_raw_until(image_ts);
    harness.PushImage(image_ts, static_cast<uint32_t>(image_ts / 10000));
    harness.Drain();
  }
  ExpectEqual(harness.ProbeSentCount(), 1U, "稳定后应发送第一次 probe");
  ExpectEqual(harness.State(), SyncState::PROBE_SENT, "第一次 probe 应等待回执");

  for (uint64_t image_ts = 40000; image_ts <= 130000; image_ts += 10000)
  {
    push_raw_until(image_ts);
    harness.PushImage(image_ts, static_cast<uint32_t>(image_ts / 10000));
    harness.Drain();
  }
  ExpectEqual(harness.ProbeSentCount(), 2U,
              "probe 命令或回执丢失后应超时重发");
  ExpectEqual(harness.State(), SyncState::PROBE_SENT, "重发后应等待新的回执");

  push_raw_until(160000);
  harness.PushSyncAck(harness.LastProbeSeq(), 160000);
  harness.PushImage(160000, 16);
  harness.Drain();

  ExpectEqual(harness.State(), SyncState::SYNCED,
              "重发 probe 收到回执后应重新锁定同步");
  ExpectVectorEqual(harness.PublishedTags(), std::vector<uint32_t>{16},
                    "重发 probe 的图像应正常发布");
}

void TestProbeWaitsForRawHistory()
{
  SequenceHarness harness;

  for (uint64_t t = 2000; t <= 48000; t += 2000)
  {
    harness.PushRawImu(t);
  }
  harness.PushImage(10000, 1);
  harness.PushImage(20000, 2);
  harness.PushImage(30000, 3);
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::PROBE_SENT, "稳定后应进入 PROBE_SENT");

  harness.PushSyncAck(harness.LastProbeSeq(), 60000);
  harness.PushImage(60000, 4);
  harness.Drain();
  Expect(harness.PublishedTags().empty(), "ack 对应 raw IMU 未到时 probe 图像应等待");

  harness.PushRawImu(50000);
  harness.PushRawImu(52000);
  harness.PushRawImu(54000);
  harness.PushRawImu(56000);
  harness.PushRawImu(58000);
  harness.PushRawImu(60000);
  harness.Drain();
  ExpectVectorEqual(harness.PublishedTags(), std::vector<uint32_t>{4},
                    "raw IMU 到达后应完成 probe 锁定");
}

void TestBadProbeGapResets()
{
  SequenceHarness harness;

  for (uint64_t t = 2000; t <= 50000; t += 2000)
  {
    harness.PushRawImu(t);
  }
  harness.PushImage(10000, 1);
  harness.PushImage(20000, 2);
  harness.PushImage(30000, 3);
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::PROBE_SENT, "稳定后应进入 PROBE_SENT");

  harness.PushImage(45000, 4);
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::OBSERVING, "错误 probe gap 后应重置");
  ExpectVectorEqual(harness.ResetTags(), std::vector<uint32_t>{},
                    "cadence 断裂直接重置，不需要把图像标为业务拒绝");
}

void TestAckedTransitionGapLocks()
{
  SequenceHarness harness;

  for (uint64_t t = 2000; t <= 50000; t += 2000)
  {
    harness.PushRawImu(t);
  }
  harness.PushImage(10000, 1);
  harness.PushImage(20000, 2);
  harness.PushImage(30000, 3);
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::PROBE_SENT, "稳定后应进入 PROBE_SENT");

  harness.PushSyncAck(harness.LastProbeSeq(), 50000);
  harness.PushImage(50000, 4);
  harness.Drain();

  ExpectEqual(harness.State(), SyncState::SYNCED,
              "带回执的分频切换过渡 gap 应完成同步");
  ExpectVectorEqual(harness.PublishedTags(), std::vector<uint32_t>{4},
                    "过渡 gap 对应图像应作为 probe 图像发布");
}

void TestSyncedModeTracksByImuPeriod()
{
  SequenceHarness harness;
  StreamDriver driver(harness);

  EmitWarmup(driver, 8);
  ExpectEqual(harness.State(), SyncState::SYNCED, "warmup 后应同步");
  const size_t first_count = harness.PublishedTags().size();

  driver.EmitImage();
  driver.EmitImage();

  Expect(harness.PublishedTags().size() >= first_count + 2,
         "稳态下后续普通图像应持续发布");
  const auto& sync_ts = harness.SyncImuTimestamps();
  for (size_t i = 1; i < sync_ts.size(); ++i)
  {
    ExpectEqual(sync_ts[i] - sync_ts[i - 1], 10000ULL,
                "稳态同步 IMU 应按图像周期对应的 IMU 周期递推");
  }
}

void TestDroppedPublishedImageKeepsSync()
{
  SequenceHarness harness;

  for (uint64_t t = 2000; t <= 90000; t += 2000)
  {
    harness.PushRawImu(t);
  }

  harness.PushImage(10000, 1);
  harness.PushImage(20000, 2);
  harness.PushImage(30000, 3);
  harness.Drain();
  harness.PushSyncAck(harness.LastProbeSeq(), 60000);
  harness.PushImage(60000, 4);
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::SYNCED, "probe 图像应先锁定同步");

  harness.DropImage(70000);
  harness.PushImage(80000, 5);
  harness.Drain();

  ExpectEqual(harness.State(), SyncState::SYNCED,
              "发布失败的单帧不应打断同步");
  ExpectVectorEqual(harness.PublishedTags(), std::vector<uint32_t>{4, 5},
                    "丢帧不发布，但下一帧应正常发布");
  ExpectVectorEqual(harness.SyncImuTimestamps(),
                    std::vector<uint64_t>{60000, 80000},
                    "丢帧后锁定 IMU 基线应跳过被丢弃的一帧");
}

void TestConsecutiveDroppedImagesKeepSync()
{
  SequenceHarness harness;

  for (uint64_t t = 2000; t <= 110000; t += 2000)
  {
    harness.PushRawImu(t);
  }

  harness.PushImage(10000, 1);
  harness.PushImage(20000, 2);
  harness.PushImage(30000, 3);
  harness.Drain();
  harness.PushSyncAck(harness.LastProbeSeq(), 60000);
  harness.PushImage(60000, 4);
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::SYNCED, "probe 图像应先锁定同步");

  harness.DropImage(70000);
  harness.DropImage(80000);
  harness.PushImage(90000, 5);
  harness.Drain();

  ExpectEqual(harness.State(), SyncState::SYNCED,
              "连续发布失败的图像不应打断同步");
  ExpectVectorEqual(harness.PublishedTags(), std::vector<uint32_t>{4, 5},
                    "连续丢帧不发布，但恢复后的图像应正常发布");
  ExpectVectorEqual(harness.SyncImuTimestamps(),
                    std::vector<uint64_t>{60000, 90000},
                    "连续丢帧后锁定 IMU 基线应逐帧跳过");
}

void TestSkippedPublishedImageGapKeepsSync()
{
  SequenceHarness harness;

  for (uint64_t t = 2000; t <= 110000; t += 2000)
  {
    harness.PushRawImu(t);
  }

  harness.PushImage(10000, 1);
  harness.PushImage(20000, 2);
  harness.PushImage(30000, 3);
  harness.Drain();
  harness.PushSyncAck(harness.LastProbeSeq(), 60000);
  harness.PushImage(60000, 4);
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::SYNCED, "probe 图像应先锁定同步");

  harness.PushImage(80000, 5);
  harness.Drain();

  ExpectEqual(harness.State(), SyncState::SYNCED,
              "同步态整数倍图像 gap 不应打断同步");
  ExpectVectorEqual(harness.PublishedTags(), std::vector<uint32_t>{4, 5},
                    "跳过一帧后下一张成功图像应正常发布");
  ExpectVectorEqual(harness.SyncImuTimestamps(),
                    std::vector<uint64_t>{60000, 80000},
                    "整数倍图像 gap 应按倍数推进 IMU 同步点");
}

void TestRawImuChannelMissDoesNotDropSyncLock()
{
  SequenceHarness harness;
  StreamDriver driver(harness);

  EmitWarmup(driver, 8);
  ExpectEqual(harness.State(), SyncState::SYNCED, "warmup 后应同步");

  const uint64_t missed_ts = driver.NextImuTimestamp();
  harness.PushGyro(missed_ts);
  harness.PushAccl(missed_ts + 1000);
  harness.PushQuat(missed_ts);
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::SYNCED,
              "单个 raw gyro 缺少可匹配辅通道时不应立即打断相机同步锁");

  harness.PushRawImu(missed_ts + driver.ImuPeriodUs());
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::SYNCED,
              "稳定 IMU 周期内的整数倍 raw 缺样不应打断同步锁");
}

void TestRawImuEpochResetClearsHistoryAndRelocks()
{
  SequenceHarness harness;
  StreamDriver driver(harness);

  EmitWarmup(driver, 8);
  ExpectEqual(harness.State(), SyncState::SYNCED, "warmup 后应同步");

  for (size_t i = 0; i < 25; ++i)
  {
    driver.EmitImage();
  }
  ExpectEqual(harness.State(), SyncState::SYNCED, "旧 epoch 稳态下应保持同步");

  harness.PushRawImu(2000);
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::OBSERVING,
              "raw IMU 时间戳大幅回退后应回到观察态");
  ExpectVectorEqual(harness.HistoryTimestamps(), std::vector<uint64_t>{2000},
                    "旧 epoch IMU 历史应清空，只保留新 epoch 第一帧");

  for (uint64_t t = 4000; t <= 50000; t += 2000)
  {
    harness.PushRawImu(t);
  }
  harness.PushImage(10000, 101);
  harness.PushImage(20000, 102);
  harness.PushImage(30000, 103);
  harness.Drain();
  ExpectEqual(harness.State(), SyncState::PROBE_SENT,
              "新 epoch 周期稳定后应重新发送 probe");

  for (uint64_t t = 52000; t <= 60000; t += 2000)
  {
    harness.PushRawImu(t);
  }
  harness.PushSyncAck(harness.LastProbeSeq(), 60000);
  harness.PushImage(60000, 104);
  harness.Drain();

  ExpectEqual(harness.State(), SyncState::SYNCED,
              "新 epoch 收到 CameraSync 回执后应重新同步");
  ExpectEqual(harness.SyncImuTimestamps().back(), 60000ULL,
              "重新同步应使用新 epoch 的 CameraSync timestamp");
}

void TestRawImuSmallRollbackDoesNotResetEpoch()
{
  SequenceHarness harness;

  harness.PushRawImu(100000);
  harness.PushRawImu(102000);
  harness.Drain();

  harness.PushRawImu(101500);
  harness.Drain();

  ExpectVectorEqual(harness.HistoryTimestamps(),
                    std::vector<uint64_t>{100000, 102000},
                    "小幅乱序不能当作设备 epoch 重启清空历史");
}

void TestCompositeResetAndRelock()
{
  SequenceHarness harness;
  StreamDriver driver(harness);

  EmitWarmup(driver, 8);
  ExpectEqual(harness.State(), SyncState::SYNCED, "复合场景第一段应先同步");

  driver.InjectRawGap(3500);
  ExpectEqual(harness.State(), SyncState::OBSERVING, "IMU cadence 断裂后应回 OBSERVING");

  EmitWarmup(driver, 8);
  ExpectEqual(harness.State(), SyncState::SYNCED, "恢复稳定后应重新同步");
  ExpectEqual(harness.ProbeSentCount(), 2U, "复合同步场景应发第二次 probe");
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
      {"raw-join/等待三路齐全", TestRawJoinWaitsForAllChannels},
      {"raw-join/要求gyro精确时间戳", TestRawJoinRequiresExactGyroTimestamp},
      {"latest/无需probe直接发布", TestLatestModePublishesWithoutProbe},
      {"latest/等待offset目标imu", TestLatestModeWaitsForOffsetTarget},
      {"raw-probe/使用CameraSync回执锁定", TestRawProbeLocksFromCameraSyncAck},
      {"raw-probe/错误seq忽略", TestWrongSeqIsIgnoredUntilCorrectAckArrives},
      {"raw-probe/probe丢失后重发", TestLostProbeCommandRetriesAndLocks},
      {"raw-probe/等待raw历史覆盖ack", TestProbeWaitsForRawHistory},
      {"raw-probe/错误probe gap重置", TestBadProbeGapResets},
      {"raw-probe/带回执过渡gap可锁定", TestAckedTransitionGapLocks},
      {"synced/按imu周期递推", TestSyncedModeTracksByImuPeriod},
      {"synced/发布失败丢帧保持同步", TestDroppedPublishedImageKeepsSync},
      {"synced/连续发布失败丢帧保持同步", TestConsecutiveDroppedImagesKeepSync},
      {"synced/成功图像整数倍gap保持同步", TestSkippedPublishedImageGapKeepsSync},
      {"synced/raw imu单点缺样保持同步", TestRawImuChannelMissDoesNotDropSyncLock},
      {"raw-imu/epoch重启后重锁", TestRawImuEpochResetClearsHistoryAndRelocks},
      {"raw-imu/小幅乱序不清epoch", TestRawImuSmallRollbackDoesNotResetEpoch},
      {"composite/断裂后重锁", TestCompositeResetAndRelock},
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
