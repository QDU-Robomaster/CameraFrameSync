#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace CameraFrameSyncCore
{
// 这里只保留同步器和序列测试共享的“纯时序工具”。
// 具体状态机仍放在 CameraFrameSync 里，避免 helper 继续膨胀成另一套框架。

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

  void PopFrontN(size_t count)
  {
    if (count >= size_)
    {
      head_ = 0;
      size_ = 0;
      return;
    }

    head_ = PhysicalIndex(count);
    size_ -= count;
  }

 private:
  size_t PhysicalIndex(size_t logical_index) const
  {
    return (head_ + logical_index) % Capacity;
  }

  size_t Increment(size_t index) const
  {
    return (index + 1) % Capacity;
  }

 private:
  std::array<T, Capacity> storage_{};
  size_t head_{0};
  size_t size_{0};
};

enum class SyncState : uint8_t
{
  UNSYNCED = 0,
  LOCKING = 1,
  SYNCED = 2,
  RECOVERING = 3,
};

struct SyncLockState
{
  SyncState state{SyncState::UNSYNCED};
  uint32_t lock_confirm_count{0};
};

inline uint64_t AbsDiffUs(uint64_t lhs, uint64_t rhs)
{
  return lhs >= rhs ? (lhs - rhs) : (rhs - lhs);
}

inline int64_t AbsDiffSigned(int64_t lhs, int64_t rhs)
{
  return lhs >= rhs ? (lhs - rhs) : (rhs - lhs);
}

inline uint64_t ApplyOffsetUs(uint64_t base_us, int32_t offset_us)
{
  if (offset_us >= 0)
  {
    return base_us + static_cast<uint64_t>(offset_us);
  }

  const uint64_t abs_offset = static_cast<uint64_t>(-static_cast<int64_t>(offset_us));
  return base_us > abs_offset ? (base_us - abs_offset) : 0ULL;
}

inline uint32_t EstimateStrideSamples(uint64_t image_period_us, uint64_t imu_period_us)
{
  if (image_period_us == 0 || imu_period_us == 0)
  {
    return 0;
  }

  const uint64_t half = imu_period_us / 2;
  const uint64_t rounded = (image_period_us + half) / imu_period_us;
  return static_cast<uint32_t>(std::max<uint64_t>(1ULL, rounded));
}

inline uint64_t HostSkewToleranceUs(uint64_t imu_rx_period_us)
{
  return std::max<uint64_t>(3000ULL,
                            imu_rx_period_us != 0 ? imu_rx_period_us * 4 : 12000ULL);
}

inline uint64_t ImageGapToleranceUs(uint64_t image_period_us, uint64_t imu_rx_period_us)
{
  const uint64_t upper =
      image_period_us != 0 ? std::min<uint64_t>(image_period_us / 3U,
                                                HostSkewToleranceUs(imu_rx_period_us))
                           : 3000ULL;
  return std::max<uint64_t>(1500ULL, upper);
}

inline uint64_t OffsetSearchToleranceUs(uint64_t imu_sensor_period_us, uint32_t stride_samples)
{
  const uint64_t base =
      imu_sensor_period_us != 0 ? imu_sensor_period_us * std::max<uint32_t>(2U, stride_samples)
                                : static_cast<uint64_t>(std::max<uint32_t>(2U, stride_samples)) *
                                      2000ULL;
  return std::max<uint64_t>(4000ULL, base);
}

inline uint64_t SyncSensorGapToleranceUs(uint64_t sync_sensor_gap_us,
                                         uint64_t imu_sensor_period_us)
{
  const uint64_t base = std::max<uint64_t>(sync_sensor_gap_us / 4U,
                                           imu_sensor_period_us != 0 ? imu_sensor_period_us * 4U
                                                                     : 4000ULL);
  return std::max<uint64_t>(4000ULL, base);
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

  return (best != nullptr && best_error <= tolerance_us) ? best : nullptr;
}

inline void ObserveGoodFrame(SyncLockState& lock_state, uint32_t relock_confirm_frames)
{
  switch (lock_state.state)
  {
    case SyncState::UNSYNCED:
    case SyncState::RECOVERING:
      lock_state.state = SyncState::LOCKING;
      lock_state.lock_confirm_count = 1;
      break;
    case SyncState::LOCKING:
      lock_state.lock_confirm_count++;
      if (lock_state.lock_confirm_count >= relock_confirm_frames)
      {
        lock_state.state = SyncState::SYNCED;
      }
      break;
    case SyncState::SYNCED:
      break;
  }
}

inline void EnterRecovering(SyncLockState& lock_state)
{
  lock_state.state = SyncState::RECOVERING;
  lock_state.lock_confirm_count = 0;
}

template <typename PendingGyroContainer, typename PendingAcclContainer,
          typename PendingQuatContainer, typename ImuHistoryContainer,
          typename MakeImuFn>
bool TryBuildFrontImu(PendingGyroContainer& pending_gyros,
                      PendingAcclContainer& pending_accls,
                      PendingQuatContainer& pending_quats,
                      ImuHistoryContainer& imu_history,
                      uint64_t& last_imu_sensor_period_us,
                      uint64_t& last_imu_rx_period_us, size_t history_limit,
                      MakeImuFn&& make_imu)
{
  if (pending_gyros.Empty())
  {
    return false;
  }

  const auto& gyro = pending_gyros.Front();
  size_t accl_index = pending_accls.Size();
  for (size_t i = 0; i < pending_accls.Size(); ++i)
  {
    if (pending_accls[i].sample.sensor_timestamp_us >= gyro.sample.sensor_timestamp_us)
    {
      accl_index = i;
      break;
    }
  }

  size_t quat_index = pending_quats.Size();
  for (size_t i = 0; i < pending_quats.Size(); ++i)
  {
    if (pending_quats[i].sample.sensor_timestamp_us >= gyro.sample.sensor_timestamp_us)
    {
      quat_index = i;
      break;
    }
  }

  if (accl_index == pending_accls.Size() || quat_index == pending_quats.Size())
  {
    return false;
  }

  if (!imu_history.Empty())
  {
    if (gyro.sample.sensor_timestamp_us > imu_history.Back().sensor_timestamp_us)
    {
      last_imu_sensor_period_us =
          gyro.sample.sensor_timestamp_us - imu_history.Back().sensor_timestamp_us;
    }
    if (gyro.rx_time_us > imu_history.Back().rx_time_us)
    {
      last_imu_rx_period_us = gyro.rx_time_us - imu_history.Back().rx_time_us;
    }
  }

  if (imu_history.Size() >= history_limit)
  {
    imu_history.PopFront();
  }
  imu_history.PushBackDropOldest(
      make_imu(gyro.sample, pending_accls[accl_index].sample, pending_quats[quat_index].sample,
               gyro.rx_time_us));

  pending_gyros.PopFront();
  pending_accls.PopFrontN(accl_index + 1);
  pending_quats.PopFrontN(quat_index + 1);
  return true;
}

}  // namespace CameraFrameSyncCore
