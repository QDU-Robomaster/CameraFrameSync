#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>

namespace CameraFrameSyncCore
{
// 这里只保留“按时间序列对齐图像和 IMU”相关的纯逻辑。
// 生产代码和独立序列测试共用这一份，避免两边各写一套。

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

struct SyncCadenceState
{
  uint32_t last_image_sequence{std::numeric_limits<uint32_t>::max()};
  uint64_t last_image_timestamp_us{0};
  uint64_t last_synced_imu_timestamp_us{0};
  uint64_t last_imu_period_us{0};
  uint64_t last_image_period_us{0};
};

inline uint64_t AbsDiffUs(uint64_t lhs, uint64_t rhs)
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

template <typename ImageEventLike>
uint64_t ImageEventTargetTimestampUs(const ImageEventLike& image_event, int32_t offset_us)
{
  return ApplyOffsetUs(image_event.sensor_timestamp_us, offset_us);
}

template <typename ImageEventLike>
bool NeedMoreImuForImage(const ImageEventLike& image_event, bool has_imu_history,
                         uint64_t newest_imu_timestamp_us, int32_t offset_us)
{
  if (!has_imu_history)
  {
    return true;
  }

  return newest_imu_timestamp_us < ImageEventTargetTimestampUs(image_event, offset_us);
}

inline uint64_t MatchToleranceUs(uint64_t last_imu_period_us)
{
  return std::max<uint64_t>(1000ULL,
                            last_imu_period_us != 0 ? last_imu_period_us * 2 : 10000ULL);
}

inline uint64_t CadenceToleranceUs(uint64_t last_imu_period_us)
{
  return std::max<uint64_t>(2000ULL,
                            last_imu_period_us != 0 ? last_imu_period_us * 2 : 2000ULL);
}

template <typename ImageEventLike, typename ImuHistoryContainer>
const typename ImuHistoryContainer::value_type* FindMatchedImu(
    const ImageEventLike& image_event, const ImuHistoryContainer& imu_history,
    int32_t offset_us, uint64_t last_imu_period_us)
{
  if (imu_history.empty())
  {
    return nullptr;
  }

  const uint64_t target_us = ImageEventTargetTimestampUs(image_event, offset_us);
  const uint64_t tolerance_us = MatchToleranceUs(last_imu_period_us);

  const typename ImuHistoryContainer::value_type* best = nullptr;
  uint64_t best_error = std::numeric_limits<uint64_t>::max();
  for (auto it = imu_history.rbegin(); it != imu_history.rend(); ++it)
  {
    const uint64_t error = AbsDiffUs(it->timestamp_us, target_us);
    if (error < best_error)
    {
      best = &(*it);
      best_error = error;
    }
    if (it->timestamp_us + tolerance_us < target_us)
    {
      break;
    }
  }

  return (best != nullptr && best_error <= tolerance_us) ? best : nullptr;
}

template <typename ImageEventLike, typename ImuLike>
bool IsCadenceStable(const ImageEventLike& image_event, const ImuLike& imu,
                     const SyncCadenceState& state)
{
  if (state.last_image_timestamp_us == 0)
  {
    return true;
  }

  if (image_event.sensor_timestamp_us <= state.last_image_timestamp_us ||
      imu.timestamp_us <= state.last_synced_imu_timestamp_us)
  {
    return false;
  }

  const uint64_t image_dt = image_event.sensor_timestamp_us - state.last_image_timestamp_us;
  const uint64_t imu_dt = imu.timestamp_us - state.last_synced_imu_timestamp_us;
  const uint64_t tolerance_us = CadenceToleranceUs(state.last_imu_period_us);

  if (state.last_image_period_us != 0 &&
      AbsDiffUs(image_dt, state.last_image_period_us) > tolerance_us)
  {
    return false;
  }
  if (state.last_image_sequence != std::numeric_limits<uint32_t>::max() &&
      image_event.image_sequence != state.last_image_sequence + 1)
  {
    return false;
  }
  if (AbsDiffUs(imu_dt, image_dt) > tolerance_us)
  {
    return false;
  }

  return true;
}

template <typename ImageEventLike, typename ImuLike>
void AcceptMatch(const ImageEventLike& image_event, const ImuLike& imu,
                 SyncCadenceState& cadence_state, SyncLockState& lock_state,
                 uint32_t relock_confirm_frames)
{
  if (cadence_state.last_image_timestamp_us != 0)
  {
    cadence_state.last_image_period_us =
        image_event.sensor_timestamp_us - cadence_state.last_image_timestamp_us;
  }

  cadence_state.last_image_timestamp_us = image_event.sensor_timestamp_us;
  cadence_state.last_synced_imu_timestamp_us = imu.timestamp_us;
  cadence_state.last_image_sequence = image_event.image_sequence;

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
  if (lock_state.state != SyncState::UNSYNCED)
  {
    lock_state.state = SyncState::RECOVERING;
    lock_state.lock_confirm_count = 0;
  }
}

inline void ResetCadence(SyncCadenceState& cadence_state)
{
  cadence_state.last_image_sequence = std::numeric_limits<uint32_t>::max();
  cadence_state.last_image_timestamp_us = 0;
  cadence_state.last_synced_imu_timestamp_us = 0;
  cadence_state.last_image_period_us = 0;
}

inline void EnterRecovering(SyncLockState& lock_state,
                            SyncCadenceState& cadence_state)
{
  EnterRecovering(lock_state);
  ResetCadence(cadence_state);
}

template <typename PendingGyroContainer, typename PendingAcclContainer,
          typename PendingQuatContainer, typename ImuHistoryContainer,
          typename MakeImuFn>
bool TryBuildFrontImu(PendingGyroContainer& pending_gyros,
                      PendingAcclContainer& pending_accls,
                      PendingQuatContainer& pending_quats,
                      ImuHistoryContainer& imu_history,
                      uint64_t& last_imu_period_us, size_t history_limit,
                      MakeImuFn&& make_imu)
{
  if (pending_gyros.empty())
  {
    return false;
  }

  const auto& gyro = pending_gyros.front();
  auto accl_it = std::find_if(
      pending_accls.begin(), pending_accls.end(),
      [&](const auto& sample)
      { return sample.sensor_timestamp_us >= gyro.sensor_timestamp_us; });
  auto quat_it = std::find_if(
      pending_quats.begin(), pending_quats.end(),
      [&](const auto& sample)
      { return sample.sensor_timestamp_us >= gyro.sensor_timestamp_us; });
  if (accl_it == pending_accls.end() || quat_it == pending_quats.end())
  {
    return false;
  }

  if (!imu_history.empty() && gyro.sensor_timestamp_us > imu_history.back().timestamp_us)
  {
    last_imu_period_us = gyro.sensor_timestamp_us - imu_history.back().timestamp_us;
  }

  imu_history.push_back(make_imu(gyro, *accl_it, *quat_it));
  while (imu_history.size() > history_limit)
  {
    imu_history.pop_front();
  }

  pending_gyros.pop_front();
  pending_accls.erase(pending_accls.begin(), std::next(accl_it));
  pending_quats.erase(pending_quats.begin(), std::next(quat_it));
  return true;
}

}  // namespace CameraFrameSyncCore
