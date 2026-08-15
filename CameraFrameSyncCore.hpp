#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace CameraFrameSyncCore
{

template <typename T, std::size_t Capacity>
class SampleHistory
{
 public:
  using ValueType = T;

  static_assert(Capacity > 0U, "SampleHistory requires non-zero capacity");

  [[nodiscard]] bool Empty() const { return size_ == 0U; }
  [[nodiscard]] std::size_t Size() const { return size_; }

  [[nodiscard]] T& Front() { return storage_[head_]; }
  [[nodiscard]] const T& Front() const { return storage_[head_]; }

  [[nodiscard]] T& Back() { return (*this)[size_ - 1U]; }
  [[nodiscard]] const T& Back() const { return (*this)[size_ - 1U]; }

  [[nodiscard]] T& operator[](std::size_t index)
  {
    return storage_[(head_ + index) % Capacity];
  }

  [[nodiscard]] const T& operator[](std::size_t index) const
  {
    return storage_[(head_ + index) % Capacity];
  }

  void Clear()
  {
    head_ = 0U;
    size_ = 0U;
  }

  [[nodiscard]] bool PushBackDropOldest(const T& value)
  {
    if (size_ < Capacity)
    {
      storage_[(head_ + size_) % Capacity] = value;
      ++size_;
      return false;
    }

    storage_[head_] = value;
    head_ = (head_ + 1U) % Capacity;
    return true;
  }

  void PopFront()
  {
    if (size_ == 0U)
    {
      return;
    }
    head_ = (head_ + 1U) % Capacity;
    --size_;
  }

 private:
  std::array<T, Capacity> storage_{};
  std::size_t head_{0U};
  std::size_t size_{0U};
};

[[nodiscard]] inline uint64_t AbsDiffUs(uint64_t lhs, uint64_t rhs)
{
  return lhs >= rhs ? lhs - rhs : rhs - lhs;
}

[[nodiscard]] inline uint64_t ApplyOffsetUs(uint64_t base_us, int32_t offset_us)
{
  if (offset_us >= 0)
  {
    const uint64_t positive = static_cast<uint64_t>(offset_us);
    return base_us > std::numeric_limits<uint64_t>::max() - positive
               ? std::numeric_limits<uint64_t>::max()
               : base_us + positive;
  }

  const uint64_t magnitude = static_cast<uint64_t>(-static_cast<int64_t>(offset_us));
  return base_us > magnitude ? base_us - magnitude : 0U;
}

/**
 * Camera timestamps are in a device-local clock. Only adjacent gaps are used;
 * no absolute comparison with the MCU clock is valid.
 */
[[nodiscard]] inline uint64_t ImageGapToleranceUs(uint64_t trigger_period_us)
{
  const uint64_t relative = trigger_period_us / 4U;
  return std::max<uint64_t>(1500U, relative);
}

/**
 * Returns round(gap / period) when the residual is within the camera-gap
 * tolerance. A zero result means the gap cannot be explained by the active
 * fixed profile.
 */
[[nodiscard]] inline uint32_t MatchImageGapStride(uint64_t gap_us,
                                                  uint64_t trigger_period_us,
                                                  uint32_t max_stride)
{
  if (trigger_period_us == 0U || max_stride == 0U)
  {
    return 0U;
  }

  const uint64_t stride = (gap_us + trigger_period_us / 2U) / trigger_period_us;
  if (stride == 0U || stride > max_stride ||
      trigger_period_us > std::numeric_limits<uint64_t>::max() / stride)
  {
    return 0U;
  }

  const uint64_t expected_gap = trigger_period_us * stride;
  return AbsDiffUs(gap_us, expected_gap) <= ImageGapToleranceUs(trigger_period_us)
             ? static_cast<uint32_t>(stride)
             : 0U;
}

template <typename History>
[[nodiscard]] const typename History::ValueType* FindBySensorTimestamp(
    const History& history, uint64_t expected_timestamp_us, uint64_t tolerance_us)
{
  const typename History::ValueType* best = nullptr;
  uint64_t best_error = std::numeric_limits<uint64_t>::max();

  for (std::size_t index = history.Size(); index > 0U; --index)
  {
    const auto& sample = history[index - 1U];
    const uint64_t error = AbsDiffUs(sample.sensor_timestamp_us, expected_timestamp_us);
    if (error < best_error)
    {
      best = &sample;
      best_error = error;
    }
    if (sample.sensor_timestamp_us < expected_timestamp_us &&
        expected_timestamp_us - sample.sensor_timestamp_us > tolerance_us)
    {
      break;
    }
  }

  return best != nullptr && best_error <= tolerance_us ? best : nullptr;
}

}  // namespace CameraFrameSyncCore
