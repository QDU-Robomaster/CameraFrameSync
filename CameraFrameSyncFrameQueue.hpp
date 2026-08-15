#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <utility>

namespace CameraFrameSyncDetail
{
/**
 * @brief Fixed-capacity FIFO that moves and destroys non-trivial frame owners
 * correctly.
 *
 * LibXR's byte-oriented queues are kept for trivially copyable IMU samples.
 * Shared frame handles use this queue so their copy/move/destructor semantics
 * are never bypassed.
 */
template <typename T, std::size_t Capacity>
class FrameQueue
{
 public:
  static_assert(Capacity > 0U, "FrameQueue requires at least one slot");

  [[nodiscard]] bool Push(T value)
  {
    if (size_ == Capacity)
    {
      return false;
    }

    slots_[tail_].emplace(std::move(value));
    tail_ = (tail_ + 1U) % Capacity;
    ++size_;
    return true;
  }

  [[nodiscard]] bool Pop(T& out)
  {
    if (size_ == 0U)
    {
      return false;
    }

    out = std::move(*slots_[head_]);
    slots_[head_].reset();
    head_ = (head_ + 1U) % Capacity;
    --size_;
    return true;
  }

  [[nodiscard]] T* Front() { return size_ == 0U ? nullptr : &*slots_[head_]; }

  [[nodiscard]] const T* Front() const { return size_ == 0U ? nullptr : &*slots_[head_]; }

  [[nodiscard]] bool PopFront()
  {
    if (size_ == 0U)
    {
      return false;
    }

    slots_[head_].reset();
    head_ = (head_ + 1U) % Capacity;
    --size_;
    return true;
  }

  void Clear()
  {
    while (size_ != 0U)
    {
      slots_[head_].reset();
      head_ = (head_ + 1U) % Capacity;
      --size_;
    }
    tail_ = head_;
  }

  [[nodiscard]] bool Empty() const { return size_ == 0U; }
  [[nodiscard]] std::size_t Size() const { return size_; }

 private:
  std::array<std::optional<T>, Capacity> slots_{};
  std::size_t head_{0U};
  std::size_t tail_{0U};
  std::size_t size_{0U};
};
}  // namespace CameraFrameSyncDetail
