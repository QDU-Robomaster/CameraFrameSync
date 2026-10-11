// ImuHistory、TriggerLink、FrameMatcher 的单元测试 / Unit tests of the three parts.
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "FrameMatcher.hpp"
#include "ImuHistory.hpp"
#include "TriggerLink.hpp"

namespace
{
void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

bool Near(float a, float b, float eps = 1e-5F) { return std::fabs(a - b) < eps; }

using Operation = CameraSyncDetail::Operation;
using SyncEvent = CameraSyncDetail::SyncEvent;

void FillImu(ImuHistory& h, uint64_t t, float x)
{
  h.AddAngularVelocity(t, {x, 0.0F, 0.0F});
  h.AddLinearAcceleration(t, {0.0F, 0.0F, 9.8F});
}

void TestImuInterpolation()
{
  ImuHistory h;
  FillImu(h, 1000, 1.0F);
  FillImu(h, 2000, 3.0F);
  h.AddRotation(1000, {1.0F, 0.0F, 0.0F, 0.0F});
  h.AddRotation(2000, {0.0F, 0.0F, 0.0F, 1.0F});
  AutoAim::ImuSample s{};
  Expect(h.At(1500, s) == ImuHistory::Lookup::OK, "inside the history");
  Expect(Near(s.angular_velocity_xyz[0], 2.0F), "linear interpolation");
  Expect(static_cast<uint64_t>(s.timestamp_us) == 1500, "sample time is the query time");
  const float r = std::sqrt(0.5F);
  Expect(Near(s.rotation_wxyz[0], r) && Near(s.rotation_wxyz[3], r),
         "nlerp is normalised");
  Expect(h.At(2000, s) == ImuHistory::Lookup::OK && Near(s.angular_velocity_xyz[0], 3.0F),
         "exact newest sample");
  Expect(h.At(2001, s) == ImuHistory::Lookup::NOT_YET, "after the newest sample");
  Expect(h.At(999, s) == ImuHistory::Lookup::TOO_OLD, "before the oldest sample");
}

void TestImuStreamsAndOrder()
{
  ImuHistory h;
  FillImu(h, 1000, 1.0F);
  FillImu(h, 2000, 1.0F);
  h.AddRotation(1000, {1.0F, 0.0F, 0.0F, 0.0F});
  AutoAim::ImuSample s{};
  // 姿态还没到 1500 / The attitude has not reached 1500 yet.
  Expect(h.At(1500, s) == ImuHistory::Lookup::NOT_YET, "one stream behind");
  // 两个四元数在相反半球：插值前翻转 / Opposite hemispheres flip before interpolating.
  h.AddRotation(2000, {-1.0F, 0.0F, 0.0F, 0.0F});
  Expect(h.At(1500, s) == ImuHistory::Lookup::OK &&
             Near(std::fabs(s.rotation_wxyz[0]), 1.0F),
         "q and -q are the same attitude");
  h.AddRotation(1500, {1.0F, 0.0F, 0.0F, 0.0F});
  Expect(h.TakeOutOfOrder() == 1 && h.TakeOutOfOrder() == 0, "late sample dropped");
  const auto latest = h.Latest();
  Expect(latest && static_cast<uint64_t>(latest->timestamp_us) == 2000, "latest sample");
  ImuHistory empty;
  Expect(!empty.Latest(), "no latest without data");
}

void TestTriggerLink()
{
  TriggerLink link(10000);
  auto a = link.Restart(0, View::NARROW);
  Expect(a.send && a.send->operation == Operation::STOP_TRIGGER && a.send->seq == 1,
         "STOP with seq 1");
  Expect(!link.Tick(50000).send, "no resend before the interval");
  a = link.Tick(100000);
  Expect(a.send && a.send->seq == 1 && link.TakeResends() == 1, "resend the same STOP");
  // 错误序号的 ACK 不算 / An ACK with another seq does not count.
  link.OnEvent({7, Operation::STOP_TRIGGER, 1, 0, 0, 0}, 100100);
  Expect(link.CurrentState() == TriggerLink::State::STOPPING, "wrong ACK ignored");
  link.OnEvent({1, Operation::STOP_TRIGGER, 1, 0, 0, 0}, 100200);
  Expect(link.CurrentState() == TriggerLink::State::SETTLING, "STOP acknowledged");
  Expect(!link.Tick(105000).send, "still settling");
  a = link.Tick(110200);
  Expect(a.send && a.send->operation == Operation::START_TRIGGER && a.send->seq == 2 &&
             a.send->trigger_period_us == 10000,
         "START with seq 2 and the period");
  Expect(a.switch_view && *a.switch_view == View::NARROW, "switch together with START");
  a = link.OnEvent({2, Operation::START_TRIGGER, 1, 0, 10000, 0}, 111000);
  Expect(
      a.started && *a.started == 2 && link.CurrentState() == TriggerLink::State::RUNNING,
      "running after the START ACK");
  Expect(!link.Tick(500000).send, "nothing to send while running");
  a = link.Restart(600000);
  Expect(a.send->seq == 3, "next round, next seq");
  link.OnEvent({3, Operation::STOP_TRIGGER, 1, 0, 0, 0}, 600100);
  Expect(!link.Tick(610100).switch_view, "no view switch unless requested");
  // 移窗与切档并入同一轮 / A move and a switch join the same round.
  link.Restart(700000, std::nullopt, NarrowPosition{0.25, 0.75});
  link.Restart(700050, View::WIDE);
  link.OnEvent({6, Operation::STOP_TRIGGER, 1, 0, 0, 0}, 700100);
  a = link.Tick(710100);
  Expect(a.move_narrow && a.move_narrow->u == 0.25 && a.move_narrow->v == 0.75 &&
             a.switch_view && *a.switch_view == View::WIDE,
         "move and switch together with START");
  link.Restart(800000);
  link.OnEvent({8, Operation::STOP_TRIGGER, 1, 0, 0, 0}, 800100);
  Expect(!link.Tick(810100).move_narrow, "no move unless requested");
}

void TestTriggerLinkSeqWrap()
{
  TriggerLink link(10000);
  uint8_t seq = 0;
  for (int i = 0; i < 300; ++i)
  {
    seq = link.Restart(static_cast<uint64_t>(i) * 1000).send->seq;
    Expect(seq != 0, "seq 0 is never used");
  }
  Expect(seq == 300 % 255, "seq wraps 255 -> 1");
}

SyncEvent Edge(uint8_t seq, uint32_t k)
{
  return {seq, Operation::FRAME_TRIGGER, 1, 0, 10000, k};
}

void TestMatcherNormalAndHostDrop()
{
  FrameMatcher m;
  uint64_t t = 0;
  Expect(m.Match(5, 0, t) == FrameMatcher::Result::DROP, "not running");
  m.Reset(4, 10000);
  for (uint32_t k = 1; k <= 5; ++k)
  {
    Expect(m.OnEdge(Edge(4, k), 1000000 + 10000ULL * k), "own edge");
  }
  // 相机计数从 100 起，图像设备时间 = 边沿 + 任意常数 / Camera counter from 100.
  Expect(m.Match(100, 7000 + 10000, t) == FrameMatcher::Result::MATCHED && t == 1010000,
         "c0 -> edge 1");
  // 主机丢了计数 101 的图 / The host dropped the image with counter 101.
  Expect(m.Match(102, 7000 + 30000, t) == FrameMatcher::Result::MATCHED && t == 1030000,
         "counter skip keeps the match exact");
  Expect(m.Match(106, 7000 + 70000, t) == FrameMatcher::Result::WAIT, "edge 7 not yet");
  m.OnEdge(Edge(4, 7), 1070000);
  Expect(m.Match(106, 7000 + 70000, t) == FrameMatcher::Result::MATCHED && t == 1070000,
         "matched once the edge arrives");
}

void TestMatcherLostEdgeAndContradictions()
{
  FrameMatcher m;
  uint64_t t = 0;
  m.Reset(9, 10000);
  m.OnEdge(Edge(9, 1), 10000);
  m.OnEdge(Edge(9, 3), 30000);  // 边沿 2 的消息丢了 / Edge 2's message was lost
  Expect(m.Match(0, 500, t) == FrameMatcher::Result::MATCHED, "edge 1");
  Expect(m.Match(1, 10500, t) == FrameMatcher::Result::DROP, "lost edge drops the image");
  Expect(m.Match(2, 20500, t) == FrameMatcher::Result::MATCHED, "edge 3 still exact");

  // 相机漏了边沿 4：计数 3 的图实际属于边沿 5 / The camera missed edge 4.
  m.OnEdge(Edge(9, 4), 40000);
  m.OnEdge(Edge(9, 5), 50000);
  Expect(m.Match(3, 50500, t) == FrameMatcher::Result::RESYNC, "missed trigger detected");

  m.Reset(10, 10000);
  m.OnEdge(Edge(10, 1), 0);
  Expect(m.Match(50, 0, t) == FrameMatcher::Result::MATCHED, "fresh start");
  Expect(m.Match(50, 10000, t) == FrameMatcher::Result::RESYNC,
         "counter did not advance");
  m.Reset(11, 10000);
  m.OnEdge(Edge(11, 1), 0);
  Expect(m.Match(10, 0, t) == FrameMatcher::Result::MATCHED, "c0 = 10");
  Expect(m.Match(20, 100000, t) == FrameMatcher::Result::RESYNC,
         "far ahead of the edges");
  Expect(!m.OnEdge(Edge(0, 1), 0), "edge of the MCU's power-up run");
  m.Stop();
  Expect(m.OnEdge(Edge(0, 1), 0), "edges ignored while stopped");
}
}  // namespace

int main()
{
  TestImuInterpolation();
  TestImuStreamsAndOrder();
  TestTriggerLink();
  TestTriggerLinkSeqWrap();
  TestMatcherNormalAndHostDrop();
  TestMatcherLostEdgeAndContradictions();
  std::puts("camera_frame_sync_parts_test passed");
  return 0;
}
