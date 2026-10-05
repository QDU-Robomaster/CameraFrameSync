#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>

#include "CameraSyncStateMachine.hpp"

/**
 * @brief 按相机帧计数把图像对应到触发边沿。START 生效后的第一张图计数为 c0，对应边沿 1；
 *        计数 c 对应边沿 c − c0 + 1。
 *        Matches images to trigger edges by the camera frame counter. The first image
 *        after START has counter c0 and belongs to edge 1; counter c belongs to edge
 *        c − c0 + 1.
 *
 * 主机侧丢帧只让计数跳号，对应仍然准确。相机漏掉触发会让计数与边沿错位，表现为两张图的
 * 设备时间间隔与对应两个边沿的间隔不一致，此时要求重新同步。
 * A frame dropped on the host only skips a counter value, and matching stays exact. A
 * trigger missed by the camera shifts counters against edges; it shows as two images
 * whose device-time interval differs from that of their edges, and resync is requested.
 */
class FrameMatcher
{
 public:
  using SyncEvent = CameraSyncDetail::SyncEvent;

  /// 记住的边沿数 / Edges remembered.
  static constexpr uint32_t EDGE_CAPACITY = 64;
  /// 图像最多领先最新边沿几个，再多即判定相机多出图 / How far an image may run ahead
  /// of the newest edge before the camera is taken to produce extra frames.
  static constexpr uint32_t MAX_AHEAD = 3;
  /// 间隔比对的最小容差 / Minimum tolerance of the interval check.
  static constexpr uint64_t MIN_TOLERANCE_US = 1500;

  enum class Result : uint8_t
  {
    MATCHED,
    WAIT,    ///< 边沿还没到 / The edge has not arrived yet
    DROP,    ///< 边沿已丢失或未启动 / The edge was lost, or not running
    RESYNC,  ///< 计数与边沿矛盾 / Counters contradict the edges
  };

  /// START 生效，用该序号的边沿重新开始 / START took effect; restart with its edges.
  void Reset(uint8_t start_seq, uint32_t period_us)
  {
    *this = FrameMatcher{};
    running_ = true;
    start_seq_ = start_seq;
    tolerance_us_ = std::max<uint64_t>(period_us / 4, MIN_TOLERANCE_US);
  }

  /// 未启动时调用，之后的图像一律丢弃 / Stop matching; later images are dropped.
  void Stop() { running_ = false; }

  /**
   * @brief 记下一个边沿。
   *        Remember one edge.
   * @return 序号不属于当前 START（如 MCU 重启后的默认运行）时返回 false，需重新同步。
   *         False when the edge belongs to another START (e.g. the MCU's power-up run
   *         after a reboot); resync is needed.
   */
  bool OnEdge(const SyncEvent& edge, uint64_t edge_time_us)
  {
    if (!running_)
    {
      return true;
    }
    if (edge.seq != start_seq_ || edge.trigger_sequence == 0)
    {
      return false;
    }
    Slot& slot = edges_[edge.trigger_sequence % EDGE_CAPACITY];
    slot = {edge.trigger_sequence, edge_time_us};
    newest_edge_ = std::max(newest_edge_, edge.trigger_sequence);
    return true;
  }

  /**
   * @param counter 相机帧计数 / Camera frame counter
   * @param image_time_us 相机设备时间 / Camera device time
   * @param edge_time_us 匹配成功时写入边沿时间（MCU 时间）/ Edge time on success
   */
  Result Match(uint32_t counter, uint64_t image_time_us, uint64_t& edge_time_us)
  {
    if (!running_)
    {
      return Result::DROP;
    }
    if (!have_c0_)
    {
      have_c0_ = true;
      c0_ = counter;
    }
    if (counter < c0_ || (matched_any_ && counter <= last_counter_))
    {
      return Result::RESYNC;  // 相机重新开始计数 / The camera restarted its counter
    }
    const uint32_t k = counter - c0_ + 1;
    const Slot& slot = edges_[k % EDGE_CAPACITY];
    if (slot.sequence != k)
    {
      if (k <= newest_edge_)
      {
        return Result::DROP;  // 边沿消息丢了 / The edge message was lost
      }
      return k > newest_edge_ + MAX_AHEAD ? Result::RESYNC : Result::WAIT;
    }
    if (matched_any_)
    {
      const int64_t image_dt = static_cast<int64_t>(image_time_us - last_image_time_us_);
      const int64_t edge_dt = static_cast<int64_t>(slot.time_us - last_edge_time_us_);
      if (static_cast<uint64_t>(std::llabs(image_dt - edge_dt)) > tolerance_us_)
      {
        return Result::RESYNC;  // 相机漏了触发 / The camera missed a trigger
      }
    }
    matched_any_ = true;
    last_counter_ = counter;
    last_image_time_us_ = image_time_us;
    last_edge_time_us_ = slot.time_us;
    edge_time_us = slot.time_us;
    return Result::MATCHED;
  }

  bool Running() const { return running_; }

 private:
  struct Slot
  {
    uint32_t sequence = 0;
    uint64_t time_us = 0;
  };

  bool running_ = false;
  uint8_t start_seq_ = 0;
  uint64_t tolerance_us_ = MIN_TOLERANCE_US;
  std::array<Slot, EDGE_CAPACITY> edges_{};
  uint32_t newest_edge_ = 0;
  bool have_c0_ = false;
  uint32_t c0_ = 0;
  bool matched_any_ = false;
  uint32_t last_counter_ = 0;
  uint64_t last_image_time_us_ = 0;
  uint64_t last_edge_time_us_ = 0;
};
