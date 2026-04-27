#include <cstdint>
#include <deque>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../camera_frame_sync_core.hpp"

namespace
{
using namespace CameraFrameSyncCore;

struct TestGyro
{
  uint64_t sensor_timestamp_us{};
};

struct TestAccl
{
  uint64_t sensor_timestamp_us{};
};

struct TestQuat
{
  uint64_t sensor_timestamp_us{};
};

struct TestImageEvent
{
  uint64_t sensor_timestamp_us{};
  uint32_t image_sequence{};
};

struct TestImu
{
  uint64_t timestamp_us{};
  uint64_t accl_timestamp_us{};
  uint64_t quat_timestamp_us{};
};

class SequenceHarness
{
 public:
  void SetOffsetUs(int32_t offset_us) { offset_us_ = offset_us; }

  void SetRelockConfirmFrames(uint32_t frames) { relock_confirm_frames_ = frames; }

  void PushGyro(uint64_t timestamp_us) { pending_gyros_.push_back({timestamp_us}); }

  void PushAccl(uint64_t timestamp_us) { pending_accls_.push_back({timestamp_us}); }

  void PushQuat(uint64_t timestamp_us) { pending_quats_.push_back({timestamp_us}); }

  void PushImage(uint64_t timestamp_us, uint32_t sequence)
  {
    pending_image_events_.push_back({timestamp_us, sequence});
  }

  void Drain()
  {
    while (TryBuildFrontImu(
        pending_gyros_, pending_accls_, pending_quats_, imu_history_,
        cadence_state_.last_imu_period_us, kHistoryLimit,
        [](const TestGyro& gyro, const TestAccl& accl, const TestQuat& quat)
        {
          return TestImu{
              .timestamp_us = gyro.sensor_timestamp_us,
              .accl_timestamp_us = accl.sensor_timestamp_us,
              .quat_timestamp_us = quat.sensor_timestamp_us,
          };
        }))
    {
    }

    while (!pending_image_events_.empty())
    {
      const TestImageEvent image_event = pending_image_events_.front();
      const bool has_imu_history = !imu_history_.empty();
      const uint64_t newest_imu_ts =
          has_imu_history ? imu_history_.back().timestamp_us : 0ULL;
      if (NeedMoreImuForImage(image_event, has_imu_history, newest_imu_ts, offset_us_))
      {
        break;
      }

      pending_image_events_.pop_front();
      const TestImu* imu =
          FindMatchedImu(image_event, imu_history_, offset_us_,
                         cadence_state_.last_imu_period_us);
      if (imu == nullptr)
      {
        no_match_timestamps_.push_back(image_event.sensor_timestamp_us);
        EnterRecovering(lock_state_, cadence_state_);
        continue;
      }
      if (!IsCadenceStable(image_event, *imu, cadence_state_))
      {
        cadence_reject_timestamps_.push_back(image_event.sensor_timestamp_us);
        EnterRecovering(lock_state_, cadence_state_);
        continue;
      }

      published_images_.push_back(image_event.sensor_timestamp_us);
      published_imus_.push_back(*imu);
      AcceptMatch(image_event, *imu, cadence_state_, lock_state_,
                  relock_confirm_frames_);
    }
  }

  size_t PendingImageCount() const { return pending_image_events_.size(); }

  size_t PendingGyroCount() const { return pending_gyros_.size(); }

  size_t PendingAcclCount() const { return pending_accls_.size(); }

  size_t PendingQuatCount() const { return pending_quats_.size(); }

  const std::vector<uint64_t>& PublishedImages() const { return published_images_; }

  const std::vector<TestImu>& PublishedImus() const { return published_imus_; }

  const std::vector<uint64_t>& CadenceRejects() const
  {
    return cadence_reject_timestamps_;
  }

  const std::vector<uint64_t>& NoMatchRejects() const { return no_match_timestamps_; }

  SyncState CurrentState() const { return lock_state_.state; }

  uint32_t LockConfirmCount() const { return lock_state_.lock_confirm_count; }

 private:
  static constexpr size_t kHistoryLimit = 2048;

  int32_t offset_us_{0};
  uint32_t relock_confirm_frames_{3};
  SyncLockState lock_state_{};
  SyncCadenceState cadence_state_{};
  std::deque<TestGyro> pending_gyros_{};
  std::deque<TestAccl> pending_accls_{};
  std::deque<TestQuat> pending_quats_{};
  std::deque<TestImageEvent> pending_image_events_{};
  std::deque<TestImu> imu_history_{};
  std::vector<uint64_t> published_images_{};
  std::vector<TestImu> published_imus_{};
  std::vector<uint64_t> cadence_reject_timestamps_{};
  std::vector<uint64_t> no_match_timestamps_{};
};

struct TestFailure : public std::exception
{
  explicit TestFailure(std::string message) : message_(std::move(message)) {}

  const char* what() const noexcept override { return message_.c_str(); }

 private:
  std::string message_;
};

template <typename T>
void ExpectEqual(const T& actual, const T& expected, const std::string& label)
{
  if (!(actual == expected))
  {
    std::ostringstream oss;
    oss << label << " mismatch";
    throw TestFailure(oss.str());
  }
}

void ExpectTrue(bool cond, const std::string& label)
{
  if (!cond)
  {
    throw TestFailure(label);
  }
}

std::vector<uint64_t> Range(uint64_t begin_us, uint64_t end_us, uint64_t step_us)
{
  std::vector<uint64_t> values;
  for (uint64_t ts = begin_us; ts <= end_us; ts += step_us)
  {
    values.push_back(ts);
  }
  return values;
}

void FeedImuTriplets(SequenceHarness& harness, const std::vector<uint64_t>& timestamps)
{
  for (uint64_t ts : timestamps)
  {
    harness.PushGyro(ts);
    harness.PushAccl(ts);
    harness.PushQuat(ts);
    harness.Drain();
  }
}

void TestNormalTenToOne()
{
  SequenceHarness harness;
  uint32_t image_seq = 0;
  for (uint64_t ts = 1000; ts <= 30000; ts += 1000)
  {
    harness.PushGyro(ts);
    harness.PushAccl(ts);
    harness.PushQuat(ts);
    if (ts % 10000 == 0)
    {
      harness.PushImage(ts, image_seq++);
    }
    harness.Drain();
  }

  ExpectEqual(harness.PublishedImages(), std::vector<uint64_t>({10000, 20000, 30000}),
              "normal publish sequence");
  ExpectTrue(harness.CadenceRejects().empty(), "normal case should not reject");
  ExpectEqual(harness.PendingImageCount(), static_cast<size_t>(0), "normal pending image");
}

void TestEarlyImageWaitsForImuCatchup()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, Range(1000, 10000, 1000));
  harness.PushImage(10000, 0);
  harness.Drain();

  FeedImuTriplets(harness, {11000, 12000});
  harness.PushImage(20000, 1);
  harness.Drain();
  ExpectEqual(harness.PendingImageCount(), static_cast<size_t>(1),
              "early image should stay pending");
  ExpectTrue(harness.CadenceRejects().empty(), "early image should not reject early");

  FeedImuTriplets(harness, Range(13000, 20000, 1000));
  ExpectEqual(harness.PublishedImages(), std::vector<uint64_t>({10000, 20000}),
              "catch-up publish sequence");
  ExpectEqual(harness.PendingImageCount(), static_cast<size_t>(0), "catch-up pending image");
}

void TestMissingQuatWaitsInsteadOfDropping()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, Range(1000, 10000, 1000));
  harness.PushImage(10000, 0);
  harness.Drain();

  for (uint64_t ts = 11000; ts <= 20000; ts += 1000)
  {
    harness.PushGyro(ts);
    harness.PushAccl(ts);
    if (ts < 20000)
    {
      harness.PushQuat(ts);
    }
    harness.Drain();
  }
  harness.PushImage(20000, 1);
  harness.Drain();

  ExpectEqual(harness.PendingImageCount(), static_cast<size_t>(1),
              "missing quat should keep image pending");
  ExpectEqual(harness.PendingGyroCount(), static_cast<size_t>(1),
              "missing quat should block matching gyro");
  ExpectEqual(harness.PendingQuatCount(), static_cast<size_t>(0),
              "missing quat queue should really be empty");

  harness.PushQuat(20000);
  harness.Drain();
  ExpectEqual(harness.PublishedImages(), std::vector<uint64_t>({10000, 20000}),
              "late quat publish sequence");
}

void TestForwardSearchUsesLaterAcclAndQuat()
{
  SequenceHarness harness;

  harness.PushGyro(10000);
  harness.PushAccl(9900);
  harness.PushAccl(10100);
  harness.PushQuat(9950);
  harness.PushQuat(10020);
  harness.Drain();

  ExpectEqual(harness.PublishedImus().size(), static_cast<size_t>(0),
              "raw assembly test should not publish image");
  ExpectEqual(harness.PendingGyroCount(), static_cast<size_t>(0), "gyro should assemble");
  ExpectEqual(harness.PendingAcclCount(), static_cast<size_t>(0), "used accls should drain");
  ExpectEqual(harness.PendingQuatCount(), static_cast<size_t>(0), "used quats should drain");

  harness.PushImage(10000, 0);
  harness.Drain();
  ExpectEqual(harness.PublishedImages(), std::vector<uint64_t>({10000}),
              "forward search publish");
  ExpectEqual(harness.PublishedImus().front().accl_timestamp_us, static_cast<uint64_t>(10100),
              "forward search accl timestamp");
  ExpectEqual(harness.PublishedImus().front().quat_timestamp_us, static_cast<uint64_t>(10020),
              "forward search quat timestamp");
}

void TestPositiveOffsetPicksLaterImu()
{
  SequenceHarness harness;
  harness.SetOffsetUs(800);

  FeedImuTriplets(harness, Range(1000, 11000, 1000));
  harness.PushImage(10000, 0);
  harness.Drain();

  ExpectEqual(harness.PublishedImages(), std::vector<uint64_t>({10000}),
              "positive offset publish");
  ExpectEqual(harness.PublishedImus().front().timestamp_us, static_cast<uint64_t>(11000),
              "positive offset target imu");
}

void TestNegativeOffsetPicksEarlierImu()
{
  SequenceHarness harness;
  harness.SetOffsetUs(-800);

  FeedImuTriplets(harness, Range(1000, 10000, 1000));
  harness.PushImage(10000, 0);
  harness.Drain();

  ExpectEqual(harness.PublishedImages(), std::vector<uint64_t>({10000}),
              "negative offset publish");
  ExpectEqual(harness.PublishedImus().front().timestamp_us, static_cast<uint64_t>(9000),
              "negative offset target imu");
}

void TestImageSequenceGapRejects()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, Range(1000, 10000, 1000));
  harness.PushImage(10000, 0);
  harness.Drain();

  FeedImuTriplets(harness, Range(11000, 20000, 1000));
  harness.PushImage(20000, 2);
  harness.Drain();

  ExpectEqual(harness.PublishedImages(), std::vector<uint64_t>({10000}),
              "gap reject publish sequence");
  ExpectEqual(harness.CadenceRejects(), std::vector<uint64_t>({20000}),
              "gap reject timestamps");
}

void TestImagePeriodJumpRejects()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, Range(1000, 20000, 1000));
  harness.PushImage(10000, 0);
  harness.Drain();
  harness.PushImage(20000, 1);
  harness.Drain();

  FeedImuTriplets(harness, Range(21000, 36000, 1000));
  harness.PushImage(36000, 2);
  harness.Drain();

  ExpectEqual(harness.PublishedImages(), std::vector<uint64_t>({10000, 20000}),
              "period jump publish sequence");
  ExpectEqual(harness.CadenceRejects(), std::vector<uint64_t>({36000}),
              "period jump reject timestamps");
}

void TestDuplicateTimestampRejects()
{
  SequenceHarness harness;

  FeedImuTriplets(harness, Range(1000, 20000, 1000));
  harness.PushImage(10000, 0);
  harness.Drain();
  harness.PushImage(20000, 1);
  harness.Drain();
  harness.PushImage(20000, 2);
  harness.Drain();

  ExpectEqual(harness.PublishedImages(), std::vector<uint64_t>({10000, 20000}),
              "duplicate timestamp publish sequence");
  ExpectEqual(harness.CadenceRejects(), std::vector<uint64_t>({20000}),
              "duplicate timestamp reject timestamps");
}

void TestRecoveringNeedsThreeGoodFramesToRelock()
{
  SequenceHarness harness;
  harness.SetRelockConfirmFrames(3);

  FeedImuTriplets(harness, Range(1000, 10000, 1000));
  harness.PushImage(10000, 0);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::LOCKING, "first frame should start locking");
  ExpectEqual(harness.LockConfirmCount(), static_cast<uint32_t>(1),
              "first frame lock count");

  FeedImuTriplets(harness, Range(11000, 20000, 1000));
  harness.PushImage(20000, 1);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::LOCKING, "second frame still locking");
  ExpectEqual(harness.LockConfirmCount(), static_cast<uint32_t>(2),
              "second frame lock count");

  FeedImuTriplets(harness, Range(21000, 30000, 1000));
  harness.PushImage(30000, 3);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::RECOVERING,
              "bad frame should enter recovering");
  ExpectEqual(harness.LockConfirmCount(), static_cast<uint32_t>(0),
              "recovering should reset lock count");

  FeedImuTriplets(harness, Range(31000, 40000, 1000));
  harness.PushImage(40000, 4);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::LOCKING, "relock frame 1");
  ExpectEqual(harness.LockConfirmCount(), static_cast<uint32_t>(1), "relock count 1");

  FeedImuTriplets(harness, Range(41000, 50000, 1000));
  harness.PushImage(50000, 5);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::LOCKING, "relock frame 2");
  ExpectEqual(harness.LockConfirmCount(), static_cast<uint32_t>(2), "relock count 2");

  FeedImuTriplets(harness, Range(51000, 60000, 1000));
  harness.PushImage(60000, 6);
  harness.Drain();
  ExpectEqual(harness.CurrentState(), SyncState::SYNCED, "relock frame 3 should sync");
  ExpectEqual(harness.LockConfirmCount(), static_cast<uint32_t>(3), "relock count 3");
}

using TestCase = void (*)();

struct NamedCase
{
  const char* name;
  TestCase fn;
};

}  // namespace

int main()
{
  const NamedCase cases[] = {
      {"正确/标准10比1", TestNormalTenToOne},
      {"正确/图像早到后续补齐", TestEarlyImageWaitsForImuCatchup},
      {"正确/缺quat时等待", TestMissingQuatWaitsInsteadOfDropping},
      {"正确/gyro前向找后继accl与quat", TestForwardSearchUsesLaterAcclAndQuat},
      {"正确/正offset选后继imu", TestPositiveOffsetPicksLaterImu},
      {"正确/负offset选前驱imu", TestNegativeOffsetPicksEarlierImu},
      {"错误/图像序列号跳变", TestImageSequenceGapRejects},
      {"错误/图像周期突变", TestImagePeriodJumpRejects},
      {"错误/重复时间戳", TestDuplicateTimestampRejects},
      {"错误后恢复/三帧重新锁定", TestRecoveringNeedsThreeGoodFramesToRelock},
  };

  bool all_passed = true;
  for (const NamedCase& test_case : cases)
  {
    try
    {
      test_case.fn();
      std::cout << "[PASS] " << test_case.name << '\n';
    }
    catch (const std::exception& ex)
    {
      all_passed = false;
      std::cerr << "[FAIL] " << test_case.name << ": " << ex.what() << '\n';
    }
  }

  return all_passed ? 0 : 1;
}
