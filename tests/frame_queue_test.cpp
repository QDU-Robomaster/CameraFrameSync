#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "../CameraFrameSyncFrameQueue.hpp"

namespace
{
struct OwnerState
{
  uint32_t owners{0U};
};

class MockFrame
{
 public:
  MockFrame() = default;

  explicit MockFrame(OwnerState& state) : state_(&state) { ++state_->owners; }

  MockFrame(const MockFrame& other) : state_(other.state_)
  {
    if (state_ != nullptr)
    {
      ++state_->owners;
    }
  }

  MockFrame& operator=(const MockFrame& other)
  {
    if (state_ == other.state_)
    {
      return *this;
    }
    Reset();
    state_ = other.state_;
    if (state_ != nullptr)
    {
      ++state_->owners;
    }
    return *this;
  }

  MockFrame(MockFrame&& other) noexcept : state_(std::exchange(other.state_, nullptr)) {}

  MockFrame& operator=(MockFrame&& other) noexcept
  {
    if (this == &other)
    {
      return *this;
    }
    Reset();
    state_ = std::exchange(other.state_, nullptr);
    return *this;
  }

  ~MockFrame() { Reset(); }

  [[nodiscard]] bool Valid() const { return state_ != nullptr; }

  void Reset()
  {
    if (state_ == nullptr)
    {
      return;
    }
    --state_->owners;
    state_ = nullptr;
  }

 private:
  OwnerState* state_{nullptr};
};

struct FrameEvent
{
  uint64_t timestamp_us{};
  uint64_t sequence{};
  MockFrame frame{};
};

void Expect(bool condition, const std::string& message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

void TestMovesOneOwnerThroughQueue()
{
  OwnerState state{};
  MockFrame input(state);
  CameraFrameSyncDetail::FrameQueue<MockFrame, 2> queue;

  Expect(queue.Push(std::move(input)), "push must accept the first owner");
  Expect(!input.Valid(), "push must move from the caller");
  Expect(state.owners == 1U, "moving through the queue must not duplicate ownership");

  MockFrame pending{};
  Expect(queue.Pop(pending), "pop must return the queued owner");
  Expect(queue.Empty(), "pop must remove the queue entry");
  Expect(pending.Valid() && state.owners == 1U,
         "pending must be the only remaining owner");

  pending.Reset();
  Expect(state.owners == 0U, "terminal reset must release the final owner");
}

void TestDuplicateTimestampsKeepDistinctFrames()
{
  OwnerState first_state{};
  OwnerState second_state{};
  CameraFrameSyncDetail::FrameQueue<FrameEvent, 2> queue;

  Expect(queue.Push({1000U, 17U, MockFrame(first_state)}),
         "first duplicate timestamp must be accepted");
  Expect(queue.Push({1000U, 18U, MockFrame(second_state)}),
         "second duplicate timestamp must be accepted");

  FrameEvent first{};
  FrameEvent second{};
  Expect(queue.Pop(first) && queue.Pop(second), "both duplicate timestamps must pop");
  Expect(first.sequence == 17U && second.sequence == 18U,
         "FIFO must preserve distinct synchronization sequences");
  Expect(first.frame.Valid() && second.frame.Valid(),
         "each duplicate timestamp must retain its own frame owner");
}

void TestClearReleasesEveryQueuedOwner()
{
  OwnerState first_state{};
  OwnerState second_state{};
  CameraFrameSyncDetail::FrameQueue<MockFrame, 2> queue;

  Expect(queue.Push(MockFrame(first_state)), "first owner must be queued");
  Expect(queue.Push(MockFrame(second_state)), "second owner must be queued");
  Expect(first_state.owners == 1U && second_state.owners == 1U,
         "queue must own both frames before reset");

  queue.Clear();
  Expect(queue.Empty(), "clear must empty the queue");
  Expect(first_state.owners == 0U && second_state.owners == 0U,
         "clear must destruct every queued owner");

  queue.Clear();
  Expect(first_state.owners == 0U && second_state.owners == 0U,
         "repeated clear must be idempotent");
}

void TestFullQueueRejectsAndReleasesInput()
{
  OwnerState first_state{};
  OwnerState second_state{};
  OwnerState rejected_state{};
  CameraFrameSyncDetail::FrameQueue<MockFrame, 2> queue;

  Expect(queue.Push(MockFrame(first_state)), "first owner must be queued");
  Expect(queue.Push(MockFrame(second_state)), "second owner must be queued");
  Expect(!queue.Push(MockFrame(rejected_state)), "full queue must reject a third owner");
  Expect(rejected_state.owners == 0U,
         "the rejected function argument must release its owner immediately");
  Expect(first_state.owners == 1U && second_state.owners == 1U,
         "full rejection must not disturb existing entries");
}
}  // namespace

int main()
{
  try
  {
    TestMovesOneOwnerThroughQueue();
    TestDuplicateTimestampsKeepDistinctFrames();
    TestClearReleasesEveryQueuedOwner();
    TestFullQueueRejectsAndReleasesInput();
    std::cout << "[PASS] frame queue ownership\n";
    return 0;
  }
  catch (const std::exception& ex)
  {
    std::cerr << "[FAIL] " << ex.what() << '\n';
    return 1;
  }
}
