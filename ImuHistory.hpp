#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "AutoAimTypes.hpp"

/**
 * @brief 三路 IMU（角速度、加速度、姿态）各自的时间环形缓冲，按任意时刻插值查询。
 *        Separate time rings for the three IMU streams (angular velocity,
 *        acceleration, attitude), queried at any time by interpolation.
 *
 * 三路由 MCU 分别发布，到达顺序不限；查询时每一路都须有覆盖该时刻的两条样本。
 * The MCU publishes the three streams separately and in any order; a query needs two
 * samples around the time in every stream.
 */
class ImuHistory
{
 public:
  /// 每路保留的样本数：1 kHz 下约 0.5 s / Samples kept per stream: ~0.5 s at 1 kHz.
  static constexpr std::size_t CAPACITY = 512;

  enum class Lookup : uint8_t
  {
    OK,
    NOT_YET,  ///< 有一路还没有覆盖该时刻的样本 / A stream has not reached the time yet
    TOO_OLD,  ///< 该时刻已移出缓冲 / The time has left the buffer
  };

  void AddAngularVelocity(uint64_t t_us, const std::array<float, 3>& v)
  {
    angular_velocity_.Add(t_us, v);
  }
  void AddLinearAcceleration(uint64_t t_us, const std::array<float, 3>& v)
  {
    linear_acceleration_.Add(t_us, v);
  }
  void AddRotation(uint64_t t_us, const std::array<float, 4>& wxyz)
  {
    rotation_.Add(t_us, wxyz);
  }

  /// 在 t 时刻插值：向量线性插值，姿态归一化线性插值 / Interpolate at t: vectors
  /// linearly, the attitude by normalised linear interpolation.
  Lookup At(uint64_t t_us, AutoAim::ImuSample& out) const
  {
    const Lookup w = angular_velocity_.At(t_us, out.angular_velocity_xyz);
    const Lookup a = linear_acceleration_.At(t_us, out.linear_acceleration_xyz);
    const Lookup q = rotation_.At(t_us, out.rotation_wxyz);
    for (Lookup l : {Lookup::TOO_OLD, Lookup::NOT_YET})
    {
      if (w == l || a == l || q == l)
      {
        return l;
      }
    }
    Normalize(out.rotation_wxyz);
    out.timestamp_us = LibXR::MicrosecondTimestamp(t_us);
    return Lookup::OK;
  }

  /// 各路最新样本，时间戳取姿态的 / The newest sample of each stream, stamped with the
  /// attitude's time.
  std::optional<AutoAim::ImuSample> Latest() const
  {
    if (angular_velocity_.Empty() || linear_acceleration_.Empty() || rotation_.Empty())
    {
      return std::nullopt;
    }
    return AutoAim::ImuSample{LibXR::MicrosecondTimestamp(rotation_.NewestTime()),
                              rotation_.Newest(), angular_velocity_.Newest(),
                              linear_acceleration_.Newest()};
  }

  /// 时间倒退而被丢弃的样本数 / Samples dropped because time went backwards.
  uint32_t TakeOutOfOrder()
  {
    const uint32_t n = angular_velocity_.out_of_order +
                       linear_acceleration_.out_of_order + rotation_.out_of_order;
    angular_velocity_.out_of_order = 0;
    linear_acceleration_.out_of_order = 0;
    rotation_.out_of_order = 0;
    return n;
  }

 private:
  template <std::size_t N>
  class Ring
  {
   public:
    using Value = std::array<float, N>;

    void Add(uint64_t t_us, const Value& value)
    {
      if (count_ > 0 && t_us <= NewestTime())
      {
        ++out_of_order;
        return;
      }
      head_ = (head_ + 1) % CAPACITY;
      times_[head_] = t_us;
      values_[head_] = value;
      count_ = count_ < CAPACITY ? count_ + 1 : CAPACITY;
    }

    Lookup At(uint64_t t_us, Value& out) const
    {
      if (count_ == 0 || t_us > NewestTime())
      {
        return Lookup::NOT_YET;
      }
      // 从新往旧找第一条不晚于 t 的样本 / Walk back to the first sample not after t.
      for (std::size_t i = 0; i < count_; ++i)
      {
        const std::size_t at = (head_ + CAPACITY - i) % CAPACITY;
        if (times_[at] <= t_us)
        {
          if (i == 0)
          {
            out = values_[at];
            return Lookup::OK;
          }
          const std::size_t next = (at + 1) % CAPACITY;
          const float s = static_cast<float>(t_us - times_[at]) /
                          static_cast<float>(times_[next] - times_[at]);
          Interpolate(values_[at], values_[next], s, out);
          return Lookup::OK;
        }
      }
      return Lookup::TOO_OLD;
    }

    bool Empty() const { return count_ == 0; }
    uint64_t NewestTime() const { return times_[head_]; }
    const Value& Newest() const { return values_[head_]; }

    uint32_t out_of_order = 0;

   private:
    static void Interpolate(const Value& a, Value b, float s, Value& out)
    {
      if constexpr (N == 4)
      {
        // 四元数取同一半球再插值 / Keep quaternions in one hemisphere.
        float dot = 0.0F;
        for (std::size_t k = 0; k < N; ++k)
        {
          dot += a[k] * b[k];
        }
        if (dot < 0.0F)
        {
          for (float& x : b)
          {
            x = -x;
          }
        }
      }
      for (std::size_t k = 0; k < N; ++k)
      {
        out[k] = a[k] + (b[k] - a[k]) * s;
      }
    }

    std::array<uint64_t, CAPACITY> times_{};
    std::array<Value, CAPACITY> values_{};
    std::size_t head_ = CAPACITY - 1;
    std::size_t count_ = 0;
  };

  static void Normalize(std::array<float, 4>& q)
  {
    const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 0.0F)
    {
      for (float& x : q)
      {
        x /= n;
      }
    }
  }

  Ring<3> angular_velocity_;
  Ring<3> linear_acceleration_;
  Ring<4> rotation_;
};
