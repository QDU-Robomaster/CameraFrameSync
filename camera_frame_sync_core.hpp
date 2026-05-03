#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace CameraFrameSyncCore
{

template <typename T, size_t Capacity>
class FixedRingBuffer
{
 public:
  using ValueType = T;

  static_assert(Capacity > 0, "FixedRingBuffer requires non-zero capacity");

  bool Empty() const { return size_ == 0; }
  size_t Size() const { return size_; }
  const T& Front() const { return storage_[head_]; }
  const T& Back() const { return storage_[PhysicalIndex(size_ - 1)]; }

  T& operator[](size_t index) { return storage_[PhysicalIndex(index)]; }
  const T& operator[](size_t index) const { return storage_[PhysicalIndex(index)]; }

  void Clear()
  {
    head_ = 0;
    size_ = 0;
  }

  bool PushBackDropOldest(const T& value)
  {
    if (size_ < Capacity)
    {
      storage_[PhysicalIndex(size_)] = value;
      ++size_;
      return false;
    }

    storage_[head_] = value;
    head_ = Increment(head_);
    return true;
  }

  void PopFront()
  {
    if (size_ == 0)
    {
      return;
    }

    head_ = Increment(head_);
    --size_;
  }

 private:
  size_t PhysicalIndex(size_t logical_index) const
  {
    return (head_ + logical_index) % Capacity;
  }

  size_t Increment(size_t index) const { return (index + 1) % Capacity; }

 private:
  std::array<T, Capacity> storage_{};
  size_t head_{0};
  size_t size_{0};
};

enum class SyncState : uint8_t
{
  OBSERVING = 0,
  PROBE_SENT = 1,
  SYNCED = 2,
};

enum class CadenceUpdate : uint8_t
{
  NO_GAP = 0,
  WARMING = 1,
  STABLE = 2,
  BROKEN = 3,
};

struct CadenceState
{
  bool has_last_timestamp{false};
  bool stable{false};
  uint64_t last_timestamp_us{0};
  uint64_t period_us{0};
  uint32_t stable_count{0};
};

inline uint64_t AbsDiffUs(uint64_t lhs, uint64_t rhs)
{
  return lhs >= rhs ? lhs - rhs : rhs - lhs;
}

inline uint64_t ApplyOffsetUs(uint64_t base_us, int32_t offset_us)
{
  if (offset_us >= 0)
  {
    return base_us + static_cast<uint64_t>(offset_us);
  }

  const uint64_t abs_offset = static_cast<uint64_t>(-static_cast<int64_t>(offset_us));
  return base_us > abs_offset ? base_us - abs_offset : 0ULL;
}

inline uint32_t EstimateStrideSamples(uint64_t image_period_us, uint64_t imu_period_us)
{
  if (image_period_us == 0 || imu_period_us == 0)
  {
    return 0;
  }

  const uint64_t rounded = (image_period_us + imu_period_us / 2ULL) / imu_period_us;
  return static_cast<uint32_t>(std::max<uint64_t>(1ULL, rounded));
}

inline uint64_t ProbeImageGapUs(uint64_t image_period_us, uint32_t div)
{
  if (image_period_us == 0 || div == 0)
  {
    return 0;
  }

  const uint64_t multiplier = static_cast<uint64_t>(div) + 1ULL;
  if (image_period_us > std::numeric_limits<uint64_t>::max() / multiplier)
  {
    return std::numeric_limits<uint64_t>::max();
  }

  // CameraSync 拉长的是一个反向半周期；正常一帧是两个半周期。
  return (image_period_us * multiplier) / 2ULL;
}

inline uint64_t ImageGapToleranceUs(uint64_t image_period_us)
{
  const uint64_t ratio = image_period_us != 0 ? image_period_us / 4ULL : 0ULL;
  return std::max<uint64_t>(1500ULL, ratio);
}

inline uint64_t ImuTimestampToleranceUs(uint64_t imu_period_us)
{
  if (imu_period_us == 0)
  {
    return 1000ULL;
  }
  return std::max<uint64_t>(100ULL, imu_period_us / 2ULL);
}

inline CadenceUpdate ObserveCadence(CadenceState& cadence, uint64_t timestamp_us,
                                    uint32_t required_stable_gaps,
                                    uint64_t min_tolerance_us)
{
  if (!cadence.has_last_timestamp)
  {
    cadence.has_last_timestamp = true;
    cadence.last_timestamp_us = timestamp_us;
    return CadenceUpdate::NO_GAP;
  }

  if (timestamp_us <= cadence.last_timestamp_us)
  {
    const bool was_stable = cadence.stable;
    cadence.has_last_timestamp = true;
    cadence.last_timestamp_us = timestamp_us;
    cadence.stable = false;
    cadence.period_us = 0;
    cadence.stable_count = 0;
    return was_stable ? CadenceUpdate::BROKEN : CadenceUpdate::WARMING;
  }

  const uint64_t gap_us = timestamp_us - cadence.last_timestamp_us;
  cadence.last_timestamp_us = timestamp_us;

  if (cadence.period_us == 0)
  {
    cadence.period_us = gap_us;
    cadence.stable_count = 1;
    cadence.stable = required_stable_gaps <= 1;
    return cadence.stable ? CadenceUpdate::STABLE : CadenceUpdate::WARMING;
  }

  const uint64_t tolerance_us = std::max<uint64_t>(min_tolerance_us,
                                                   cadence.period_us / 4ULL);
  if (AbsDiffUs(gap_us, cadence.period_us) <= tolerance_us)
  {
    cadence.period_us = gap_us;
    if (cadence.stable_count < required_stable_gaps)
    {
      ++cadence.stable_count;
    }
    cadence.stable = cadence.stable_count >= required_stable_gaps;
    return cadence.stable ? CadenceUpdate::STABLE : CadenceUpdate::WARMING;
  }

  const bool was_stable = cadence.stable;
  cadence.stable = false;
  cadence.period_us = 0;
  cadence.stable_count = 0;
  return was_stable ? CadenceUpdate::BROKEN : CadenceUpdate::WARMING;
}

template <typename ImuHistoryContainer>
const typename ImuHistoryContainer::ValueType* FindBySensorTimestamp(
    const ImuHistoryContainer& imu_history, uint64_t expected_timestamp_us,
    uint64_t tolerance_us)
{
  const typename ImuHistoryContainer::ValueType* best = nullptr;
  uint64_t best_error = std::numeric_limits<uint64_t>::max();

  for (size_t i = imu_history.Size(); i > 0; --i)
  {
    const auto& imu = imu_history[i - 1];
    const uint64_t error = AbsDiffUs(imu.sensor_timestamp_us, expected_timestamp_us);
    if (error < best_error)
    {
      best = &imu;
      best_error = error;
    }
    if (imu.sensor_timestamp_us + tolerance_us < expected_timestamp_us)
    {
      break;
    }
  }

  return best != nullptr && best_error <= tolerance_us ? best : nullptr;
}

}  // namespace CameraFrameSyncCore
