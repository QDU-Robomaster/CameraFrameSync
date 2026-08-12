#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace CameraFrameSyncCore
{

/** @brief Result of validating a frame geometry against the process-lifetime lock. */
enum class GeometryLockOutcome : uint8_t
{
  LOCKED = 0,
  MATCHED = 1,
  REJECT_INVALID = 2,
  REJECT_CHANGED = 3,
};

/** @brief First accepted geometry retained independently from synchronization resets. */
template <typename Geometry>
struct GeometryLockState
{
  bool locked{false};
  Geometry geometry{};
};

/**
 * @brief Validate and compare a geometry candidate, locking the first valid value.
 *
 * Rejected candidates never mutate an existing lock.
 */
template <typename Geometry, typename SameGeometry>
[[nodiscard]] constexpr GeometryLockOutcome CheckAndLockGeometry(
    GeometryLockState<Geometry>& lock, const Geometry& candidate, bool valid,
    SameGeometry same_geometry)
{
  if (!valid)
  {
    return GeometryLockOutcome::REJECT_INVALID;
  }
  if (!lock.locked)
  {
    lock.geometry = candidate;
    lock.locked = true;
    return GeometryLockOutcome::LOCKED;
  }
  if (!same_geometry(candidate, lock.geometry))
  {
    return GeometryLockOutcome::REJECT_CHANGED;
  }
  return GeometryLockOutcome::MATCHED;
}

/**
 * @brief 固定容量样本缓存，满时丢弃最旧样本。
 */
template <typename T, size_t Capacity>
class SampleHistory
{
 public:
  using ValueType = T;

  static_assert(Capacity > 0, "SampleHistory requires non-zero capacity");

  /**
   * @brief 是否为空。
   */
  bool Empty() const { return size_ == 0; }

  /**
   * @brief 当前元素数量。
   */
  size_t Size() const { return size_; }

  /**
   * @brief 返回最旧元素。
   */
  const T& Front() const { return storage_[0]; }
  T& Front() { return storage_[0]; }

  /**
   * @brief 返回最新元素。
   */
  const T& Back() const { return storage_[size_ - 1]; }
  T& Back() { return storage_[size_ - 1]; }

  /**
   * @brief 按从旧到新的逻辑下标访问元素。
   */
  T& operator[](size_t index) { return storage_[index]; }

  /**
   * @brief 按从旧到新的逻辑下标访问元素。
   */
  const T& operator[](size_t index) const { return storage_[index]; }

  /**
   * @brief 清空缓存。
   */
  void Clear() { size_ = 0; }

  /**
   * @brief 追加元素，容量满时丢弃最旧元素。
   *
   * @return true 表示发生了丢弃。
   */
  bool PushBackDropOldest(const T& value)
  {
    if (size_ < Capacity)
    {
      storage_[size_] = value;
      ++size_;
      return false;
    }

    for (size_t i = 1; i < Capacity; ++i)
    {
      storage_[i - 1] = storage_[i];
    }
    storage_[Capacity - 1] = value;
    return true;
  }

  /**
   * @brief 丢弃最旧元素。
   */
  void PopFront()
  {
    if (size_ == 0)
    {
      return;
    }

    for (size_t i = 1; i < size_; ++i)
    {
      storage_[i - 1] = storage_[i];
    }
    --size_;
  }

 private:
  std::array<T, Capacity> storage_{};
  size_t size_{0};
};

/**
 * @brief RAW_PROBE 同步状态。
 */
enum class SyncState : uint8_t
{
  OBSERVING = 0,   ///< 正在观察稳定图像周期和 IMU 周期。
  PROBE_SENT = 1,  ///< 已发送同步探针，等待 probe 图像和回执。
  SYNCED = 2,      ///< 已锁定同步关系。
};

/**
 * @brief 周期观察结果。
 */
enum class CadenceUpdate : uint8_t
{
  NO_GAP = 0,   ///< 第一条样本，还没有周期信息。
  WARMING = 1,  ///< 周期尚未稳定。
  STABLE = 2,   ///< 周期稳定。
  BROKEN = 3,   ///< 已稳定周期被新样本打破。
};

/**
 * @brief 单路传感器或图像发布周期的观察状态。
 */
struct CadenceState
{
  static constexpr size_t max_period_window_size = 5;

  bool has_last_timestamp{false};  ///< 是否已经收到上一条时间戳。
  bool stable{false};              ///< 当前周期是否稳定。
  uint64_t last_timestamp_us{0};   ///< 最近一次时间戳。
  uint64_t period_us{0};           ///< 当前估计周期。
  uint32_t stable_count{0};        ///< 连续满足容差的 gap 数量。
  std::array<uint64_t, max_period_window_size> period_gaps_us{};  ///< 周期估计窗口。
  size_t period_gap_count{0};       ///< 窗口内有效 gap 数量。
  size_t period_gap_next_index{0};  ///< 下一个 gap 的写入位置。
  size_t period_window_size{0};     ///< 当前周期估计窗口大小。
};

/**
 * @brief 计算两个微秒时间差的绝对值。
 */
inline uint64_t AbsDiffUs(uint64_t lhs, uint64_t rhs)
{
  return lhs >= rhs ? lhs - rhs : rhs - lhs;
}

/**
 * @brief 在同一个时间轴内施加有符号 offset。
 */
inline uint64_t ApplyOffsetUs(uint64_t base_us, int32_t offset_us)
{
  if (offset_us >= 0)
  {
    return base_us + static_cast<uint64_t>(offset_us);
  }

  const uint64_t abs_offset = static_cast<uint64_t>(-static_cast<int64_t>(offset_us));
  return base_us > abs_offset ? base_us - abs_offset : 0ULL;
}

/**
 * @brief 根据图像周期和 IMU 周期估计每帧图像跨过的 IMU 样本数。
 */
inline uint32_t EstimateStrideSamples(uint64_t image_period_us, uint64_t imu_period_us)
{
  if (image_period_us == 0 || imu_period_us == 0)
  {
    return 0;
  }

  const uint64_t rounded = (image_period_us + imu_period_us / 2ULL) / imu_period_us;
  return static_cast<uint32_t>(std::max<uint64_t>(1ULL, rounded));
}

/**
 * @brief 根据 IMU 周期和目标触发频率计算 CameraSync 运行分频。
 */
inline uint8_t TargetRunTriggerDiv(uint64_t imu_period_us, float target_trigger_hz)
{
  if (imu_period_us == 0 || !std::isfinite(target_trigger_hz) ||
      target_trigger_hz <= 0.0F)
  {
    return 1;
  }

  const double imu_hz = 1000000.0 / static_cast<double>(imu_period_us);
  const double rounded_div = imu_hz / static_cast<double>(target_trigger_hz) + 0.5;
  if (rounded_div <= 1.0)
  {
    return 1;
  }

  constexpr uint8_t maximum_div = std::numeric_limits<uint8_t>::max();
  if (rounded_div >= static_cast<double>(maximum_div))
  {
    return maximum_div;
  }
  return static_cast<uint8_t>(rounded_div);
}

/**
 * @brief CameraSync 探针分频后，Host 应观察到的 probe 图像间隔。
 */
inline uint64_t ProbeImageGapUs(uint64_t image_period_us, uint32_t div)
{
  if (image_period_us == 0 || div == 0)
  {
    return 0;
  }

  const uint64_t multiplier = static_cast<uint64_t>(div);
  if (image_period_us > std::numeric_limits<uint64_t>::max() / multiplier)
  {
    return std::numeric_limits<uint64_t>::max();
  }

  return image_period_us * multiplier;
}

/**
 * @brief 图像周期抖动容差。
 */
inline uint64_t ImageGapToleranceUs(uint64_t image_period_us)
{
  const uint64_t ratio = image_period_us != 0 ? image_period_us / 4ULL : 0ULL;
  return std::max<uint64_t>(1500ULL, ratio);
}

/**
 * @brief 判断图像 gap 是否为正常周期的整数倍。
 *
 * 返回值为匹配到的周期倍数；返回 0 表示不匹配。
 */
inline uint32_t MatchImageGapStride(uint64_t gap_us, uint64_t image_period_us,
                                    uint32_t max_stride)
{
  if (image_period_us == 0 || max_stride == 0)
  {
    return 0;
  }

  const uint64_t stride = (gap_us + image_period_us / 2ULL) / image_period_us;
  if (stride == 0 || stride > max_stride)
  {
    return 0;
  }
  if (image_period_us > std::numeric_limits<uint64_t>::max() / stride)
  {
    return 0;
  }

  const uint64_t expected = image_period_us * stride;
  return AbsDiffUs(gap_us, expected) <= ImageGapToleranceUs(image_period_us)
             ? static_cast<uint32_t>(stride)
             : 0;
}

/**
 * @brief 判断 gap 是否为给定周期的整数倍。
 *
 * tolerance_us 由调用者按数据源类型提供；IMU 不能复用图像的大容差。
 */
inline uint32_t MatchPeriodGapStride(uint64_t gap_us, uint64_t period_us,
                                     uint64_t tolerance_us, uint32_t max_stride)
{
  if (period_us == 0 || max_stride == 0)
  {
    return 0;
  }

  const uint64_t stride = (gap_us + period_us / 2ULL) / period_us;
  if (stride == 0 || stride > max_stride)
  {
    return 0;
  }
  if (period_us > std::numeric_limits<uint64_t>::max() / stride)
  {
    return 0;
  }

  const uint64_t expected = period_us * stride;
  return AbsDiffUs(gap_us, expected) <= tolerance_us ? static_cast<uint32_t>(stride) : 0;
}

/**
 * @brief IMU 时间戳匹配容差。
 */
inline uint64_t ImuTimestampToleranceUs(uint64_t imu_period_us)
{
  if (imu_period_us == 0)
  {
    return 1000ULL;
  }
  return std::max<uint64_t>(100ULL, imu_period_us / 2ULL);
}

/**
 * @brief 清空周期估计，但保留最近时间戳作为下一条 gap 的起点。
 */
inline void ResetCadenceEstimate(CadenceState& cadence)
{
  cadence.stable = false;
  cadence.period_us = 0;
  cadence.stable_count = 0;
  cadence.period_gap_count = 0;
  cadence.period_gap_next_index = 0;
  cadence.period_window_size = 0;
}

/**
 * @brief 将一个 gap 写入固定窗口，并用窗口中位数更新周期估计。
 */
inline void AddCadenceGap(CadenceState& cadence, uint64_t gap_us,
                          size_t period_window_size)
{
  period_window_size =
      std::clamp<size_t>(period_window_size, 1, CadenceState::max_period_window_size);
  if (cadence.period_window_size != period_window_size)
  {
    cadence.stable = false;
    cadence.stable_count = 0;
    cadence.period_gap_count = 0;
    cadence.period_gap_next_index = 0;
    cadence.period_window_size = period_window_size;
  }

  cadence.period_gaps_us[cadence.period_gap_next_index] = gap_us;
  cadence.period_gap_next_index =
      (cadence.period_gap_next_index + 1) % period_window_size;
  cadence.period_gap_count = std::min(cadence.period_gap_count + 1, period_window_size);

  auto sorted_gaps = cadence.period_gaps_us;
  constexpr uint64_t unused_gap = std::numeric_limits<uint64_t>::max();
  switch (cadence.period_gap_count)
  {
    case 1:
      sorted_gaps[1] = unused_gap;
      [[fallthrough]];
    case 2:
      sorted_gaps[2] = unused_gap;
      [[fallthrough]];
    case 3:
      sorted_gaps[3] = unused_gap;
      [[fallthrough]];
    case 4:
      sorted_gaps[4] = unused_gap;
      [[fallthrough]];
    case 5:
      break;
    default:
      cadence.period_us = gap_us;
      return;
  }

  const auto sort_pair = [](uint64_t& first, uint64_t& second)
  {
    if (first > second)
    {
      const uint64_t temporary = first;
      first = second;
      second = temporary;
    }
  };
  sort_pair(sorted_gaps[0], sorted_gaps[1]);
  sort_pair(sorted_gaps[1], sorted_gaps[2]);
  sort_pair(sorted_gaps[2], sorted_gaps[3]);
  sort_pair(sorted_gaps[3], sorted_gaps[4]);
  sort_pair(sorted_gaps[0], sorted_gaps[1]);
  sort_pair(sorted_gaps[1], sorted_gaps[2]);
  sort_pair(sorted_gaps[2], sorted_gaps[3]);
  sort_pair(sorted_gaps[0], sorted_gaps[1]);
  sort_pair(sorted_gaps[1], sorted_gaps[2]);
  sort_pair(sorted_gaps[0], sorted_gaps[1]);

  switch (cadence.period_gap_count)
  {
    case 1:
      cadence.period_us = sorted_gaps[0];
      break;
    case 2:
      cadence.period_us = sorted_gaps[0] + (sorted_gaps[1] - sorted_gaps[0]) / 2ULL;
      break;
    case 3:
      cadence.period_us = sorted_gaps[1];
      break;
    case 4:
      cadence.period_us = sorted_gaps[1] + (sorted_gaps[2] - sorted_gaps[1]) / 2ULL;
      break;
    case 5:
      cadence.period_us = sorted_gaps[2];
      break;
    default:
      break;
  }
}

/**
 * @brief 锁定一个已知的单 gap 周期，并初始化后续滚动估计所需的元数据。
 */
inline void LockSingleGapCadence(CadenceState& cadence, uint64_t timestamp_us,
                                 uint64_t period_us, uint32_t stable_count)
{
  cadence.has_last_timestamp = true;
  cadence.last_timestamp_us = timestamp_us;
  ResetCadenceEstimate(cadence);
  AddCadenceGap(cadence, period_us, 1);
  cadence.stable_count = stable_count;
  cadence.stable = true;
}

/**
 * @brief 观察一条新时间戳并更新周期稳定性。
 *
 * period_window_size 为 1 时保持末次 gap 语义；大于 1 时使用滚动中位数。
 */
inline CadenceUpdate ObserveCadence(CadenceState& cadence, uint64_t timestamp_us,
                                    uint32_t required_stable_gaps,
                                    uint64_t min_tolerance_us,
                                    size_t period_window_size = 1)
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
    ResetCadenceEstimate(cadence);
    return was_stable ? CadenceUpdate::BROKEN : CadenceUpdate::WARMING;
  }

  const uint64_t gap_us = timestamp_us - cadence.last_timestamp_us;
  cadence.last_timestamp_us = timestamp_us;

  const size_t effective_window_size =
      std::clamp<size_t>(period_window_size, 1, CadenceState::max_period_window_size);
  if (cadence.period_window_size != effective_window_size)
  {
    ResetCadenceEstimate(cadence);
  }

  if (cadence.period_us == 0)
  {
    AddCadenceGap(cadence, gap_us, effective_window_size);
    cadence.stable_count = 1;
    cadence.stable = required_stable_gaps <= 1 &&
                     cadence.period_gap_count >= cadence.period_window_size;
    return cadence.stable ? CadenceUpdate::STABLE : CadenceUpdate::WARMING;
  }

  const uint64_t tolerance_us =
      std::max<uint64_t>(min_tolerance_us, cadence.period_us / 4ULL);
  if (AbsDiffUs(gap_us, cadence.period_us) <= tolerance_us)
  {
    AddCadenceGap(cadence, gap_us, effective_window_size);
    if (cadence.stable_count < required_stable_gaps)
    {
      ++cadence.stable_count;
    }
    cadence.stable = cadence.stable_count >= required_stable_gaps &&
                     cadence.period_gap_count >= cadence.period_window_size;
    return cadence.stable ? CadenceUpdate::STABLE : CadenceUpdate::WARMING;
  }

  const bool was_stable = cadence.stable;
  ResetCadenceEstimate(cadence);
  return was_stable ? CadenceUpdate::BROKEN : CadenceUpdate::WARMING;
}

/**
 * @brief 在 IMU 样本缓存中查找 sensor timestamp 最接近期望值的样本。
 *
 * @return 找到且误差不超过 tolerance_us 时返回样本指针，否则返回 nullptr。
 */
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
