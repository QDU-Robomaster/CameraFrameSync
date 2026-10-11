#pragma once

#include <cstdint>
#include <optional>

#include "CameraBase.hpp"
#include "CameraSyncStateMachine.hpp"

/**
 * @brief 与 MCU 上 CameraSync 的触发链路：STOP → 等 ACK → 稳定 → （移窗、切档）→ START
 *        → 等 ACK → 运行。只是状态机，不收发 Topic，时间由调用方给出。
 *        Trigger link to CameraSync on the MCU: STOP → ACK → settle → (window move,
 *        view switch) → START → ACK → running. A state machine only; it sends and
 *        receives no Topics, and the caller supplies the time.
 *
 * 命令带非零序号；MCU 对同一序号的重复命令重发同一个 ACK，所以等 ACK 时按固定间隔重发。
 * Commands carry a non-zero sequence number; the MCU answers a repeated command with
 * the same ACK, so commands are resent at a fixed interval while waiting.
 */
class TriggerLink
{
 public:
  using SyncCommand = CameraSyncDetail::SyncCommand;
  using SyncEvent = CameraSyncDetail::SyncEvent;
  using Operation = CameraSyncDetail::Operation;

  /// 停触发后等在途图像到齐 / Wait for images in flight after the trigger stops.
  static constexpr uint64_t SETTLE_US = 10000;
  /// 等 ACK 时的重发间隔 / Resend interval while waiting for an ACK.
  static constexpr uint64_t RESEND_US = 100000;
  /// 触发有效电平 / Trigger active level.
  static constexpr uint8_t ACTIVE_LEVEL = 1;

  enum class State : uint8_t
  {
    STOPPING,  ///< 已发 STOP，等 ACK / STOP sent, waiting for the ACK
    SETTLING,  ///< 已停，等在途图像 / Stopped, waiting for images in flight
    STARTING,  ///< 已发 START，等 ACK / START sent, waiting for the ACK
    RUNNING,
  };

  /// 一次调用要调用方做的事 / What one call asks the caller to do.
  struct Actions
  {
    std::optional<SyncCommand> send;            ///< 发给 MCU / Send to the MCU
    std::optional<NarrowPosition> move_narrow;  ///< 现在移窗 / Move the window now
    std::optional<View> switch_view;            ///< 现在切档 / Switch the view now
    std::optional<uint8_t> started;  ///< 触发已启动，值为 START 序号 / Running, START seq
  };

  explicit TriggerLink(uint32_t period_us) : period_us_(period_us) {}

  /**
   * @brief 重新开始一轮 STOP/START，可同时切档、移窗；任何状态下都可调用，未完成的请求
   *        并入这一轮。
   *        Start a new STOP/START round, optionally switching the view and moving the
   *        window; valid in any state, and unfinished requests join this round.
   */
  Actions Restart(uint64_t now_us, std::optional<View> view = std::nullopt,
                  std::optional<NarrowPosition> move = std::nullopt)
  {
    if (view)
    {
      pending_view_ = view;
    }
    if (move)
    {
      pending_move_ = move;
    }
    state_ = State::STOPPING;
    command_ = {Operation::STOP_TRIGGER, ACTIVE_LEVEL, NextSeq(), 0, 0};
    return Send(now_us);
  }

  Actions OnEvent(const SyncEvent& event, uint64_t now_us)
  {
    Actions actions;
    const bool ack = event.seq == command_.seq && event.operation == command_.operation;
    if (state_ == State::STOPPING && ack)
    {
      state_ = State::SETTLING;
      settle_until_us_ = now_us + SETTLE_US;
    }
    else if (state_ == State::STARTING && ack)
    {
      state_ = State::RUNNING;
      actions.started = command_.seq;
    }
    return actions;
  }

  Actions Tick(uint64_t now_us)
  {
    if ((state_ == State::STOPPING || state_ == State::STARTING) &&
        now_us - sent_at_us_ >= RESEND_US)
    {
      ++resends_;
      return Send(now_us);
    }
    if (state_ == State::SETTLING && now_us >= settle_until_us_)
    {
      state_ = State::STARTING;
      command_ = {Operation::START_TRIGGER, ACTIVE_LEVEL, NextSeq(), 0, period_us_};
      Actions actions = Send(now_us);
      actions.move_narrow = pending_move_;
      actions.switch_view = pending_view_;
      pending_move_.reset();
      pending_view_.reset();
      return actions;
    }
    return {};
  }

  State CurrentState() const { return state_; }
  uint32_t PeriodUs() const { return period_us_; }
  /// 重发次数（读后清零）/ Resend count, cleared on read.
  uint32_t TakeResends()
  {
    const uint32_t n = resends_;
    resends_ = 0;
    return n;
  }

 private:
  Actions Send(uint64_t now_us)
  {
    sent_at_us_ = now_us;
    Actions actions;
    actions.send = command_;
    return actions;
  }

  /// 序号 1–255 循环，0 留给 MCU 上电时的默认运行 / Sequence 1–255; 0 is the MCU's
  /// power-up run.
  uint8_t NextSeq()
  {
    seq_ = seq_ == 255 ? 1 : seq_ + 1;
    return seq_;
  }

  const uint32_t period_us_;
  State state_ = State::STOPPING;
  SyncCommand command_{};
  uint8_t seq_ = 0;
  uint64_t sent_at_us_ = 0;
  uint64_t settle_until_us_ = 0;
  std::optional<View> pending_view_;
  std::optional<NarrowPosition> pending_move_;
  uint32_t resends_ = 0;
};
