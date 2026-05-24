#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

#include "libxr.hpp"
#include "linux_shared_topic.hpp"

/**
 * @brief 一组已同步的图像共享槽位和 IMU 数据。
 *
 * image 持有 LinuxSharedTopic 的共享槽位；使用方处理完 out 后释放或覆盖 SyncedFrame，
 * 槽位才会回到共享图像池。
 */
template <typename ImageTopicT, typename ImuStampedT>
struct CameraFrameSyncSyncedFrame
{
  using ImageData = typename ImageTopicT::Data;

  ImageData image{};  ///< 共享图像槽位。
  ImuStampedT imu{};  ///< 与图像 timestamp_us 相同的同步 IMU。

  /**
   * @brief 返回当前图像帧指针。
   */
  const auto* GetImageFrame() const { return image.GetData(); }
};

/**
 * @brief CameraFrameSync 的阻塞式消费者。
 *
 * 该类订阅共享图像 topic 和同步 IMU topic。Wait() 只返回 timestamp_us 完全相同
 * 的一组数据，保证后续 Detector/Tracker 不需要再做姿态匹配。
 */
template <typename ImageTopicT, typename ImuStampedT>
class CameraFrameSyncSubscriber
{
 public:
  using SyncedFrame = CameraFrameSyncSyncedFrame<ImageTopicT, ImuStampedT>;
  using ImageData = typename ImageTopicT::Data;
  using ImuMessage = typename LibXR::Topic::template Message<ImuStampedT>;

  static constexpr std::size_t imu_queue_capacity = 256;

  /**
   * @brief 图像订阅使用丢旧语义，避免启动同步或消费滞后时反压相机。
   *
   * 图像是同步参考但不是积压队列；如果当前图像等不到同 timestamp 的 IMU，
   * 后续更近的图像应覆盖旧描述符，不能让共享图像 publish 返回 FULL。
   */
  static constexpr LibXR::LinuxSharedSubscriberMode image_subscriber_mode =
      LibXR::LinuxSharedSubscriberMode::BROADCAST_DROP_OLD;

  /**
   * @brief 从 CameraFrameSync 实例推导 topic 名称。
   */
  template <typename SyncT>
  explicit CameraFrameSyncSubscriber(const SyncT& sync)
      : CameraFrameSyncSubscriber(sync.ImageTopicName(), sync.ImuTopicName(),
                                  sync.HostTopicDomainName(),
                                  sync.GetSyncMode() == SyncT::SyncMode::LATEST_IMU)
  {
  }

  /**
   * @brief 在默认 host domain 中订阅图像和 IMU。
   */
  CameraFrameSyncSubscriber(std::string_view image_topic_name,
                            std::string_view imu_topic_name)
      : CameraFrameSyncSubscriber(image_topic_name, imu_topic_name, "host",
                                  false)
  {
  }

  /**
   * @brief 订阅指定图像 topic、IMU topic 和 Host topic domain。
   */
  CameraFrameSyncSubscriber(std::string_view image_topic_name,
                            std::string_view imu_topic_name,
                            std::string_view host_topic_domain_name,
                            bool latest_image_mode)
      : image_topic_name_(image_topic_name),
        imu_topic_name_(imu_topic_name),
        host_topic_domain_name_(host_topic_domain_name),
        latest_image_mode_(latest_image_mode),
        host_topic_domain_(host_topic_domain_name_.c_str()),
        image_sub_(image_topic_name_.c_str(), image_subscriber_mode),
        imu_sub_(LibXR::Topic(LibXR::Topic::WaitTopic(
                     imu_topic_name_.c_str(), UINT32_MAX, &host_topic_domain_)),
                 latest_imu_),
        imu_queue_sub_(LibXR::Topic(LibXR::Topic::WaitTopic(
                           imu_topic_name_.c_str(), UINT32_MAX,
                           &host_topic_domain_)),
                       imu_queue_)
  {
  }

  /**
   * @brief 图像共享 topic 是否可用。
   */
  bool Valid() const { return image_sub_.Valid(); }

  /**
   * @brief 阻塞等待一组 timestamp_us 完全一致的图像和 IMU。
   *
   * @param out 输出同步结果；成功返回后 out.image 持有共享图像槽位。
   * @param timeout_ms 总超时时间，UINT32_MAX 表示无限等待。
   * @return OK 表示成功；其他错误来自底层 topic 等待或超时。
   *
   * 等待顺序是先同步 IMU、后图像。同步 IMU 是 CameraFrameSync 已经完成配对的
   * 事件；这样 RAW_PROBE 启动阶段还没有同步结果时，消费者不会拿住无效图像槽位。
   */
  LibXR::ErrorCode Wait(SyncedFrame& out, uint32_t timeout_ms)
  {
    const uint64_t deadline_ms = MakeDeadline(timeout_ms);

    if (latest_image_mode_)
    {
      while (true)
      {
        ImuStampedT imu{};
        const auto imu_ans =
            WaitForLatestQueuedImu(deadline_ms, timeout_ms, imu);
        if (imu_ans == LibXR::ErrorCode::OK)
        {
          ImageData image;
          const auto image_ans = WaitForMatchingImage(
              static_cast<uint64_t>(imu.timestamp_us), deadline_ms, timeout_ms,
              image);
          if (image_ans == LibXR::ErrorCode::OK)
          {
            out.imu = imu;
            out.image = std::move(image);
            return LibXR::ErrorCode::OK;
          }
          if (image_ans == LibXR::ErrorCode::EMPTY)
          {
            continue;
          }
          return image_ans;
        }
        return imu_ans;
      }
    }

    while (true)
    {
      ImuStampedT imu{};
      const auto imu_ans = WaitForNextImu(deadline_ms, timeout_ms, imu);
      if (imu_ans != LibXR::ErrorCode::OK)
      {
        return imu_ans;
      }

      ImageData image;
      const auto image_ans = latest_image_mode_
                                 ? WaitForLatestImage(deadline_ms, timeout_ms, image)
                                 : WaitForMatchingImage(
                                       static_cast<uint64_t>(imu.timestamp_us),
                                       deadline_ms, timeout_ms, image);
      if (image_ans == LibXR::ErrorCode::OK)
      {
        out.imu = imu;
        out.image = std::move(image);
        return LibXR::ErrorCode::OK;
      }
      if (image_ans == LibXR::ErrorCode::EMPTY)
      {
        // 图像已经越过该 IMU timestamp，说明这组数据因丢旧策略被跳过。
        continue;
      }
      return image_ans;
    }
  }

 private:
  /**
   * @brief 计算本次 Wait 的绝对超时时间。
   */
  static uint64_t MakeDeadline(uint32_t timeout_ms)
  {
    if (timeout_ms == UINT32_MAX)
    {
      return std::numeric_limits<uint64_t>::max();
    }
    return static_cast<uint64_t>(LibXR::Thread::GetTime()) + timeout_ms;
  }

  /**
   * @brief 计算剩余等待时间。
   */
  static uint32_t RemainingMs(uint64_t deadline_ms, uint32_t timeout_ms)
  {
    if (timeout_ms == UINT32_MAX)
    {
      return UINT32_MAX;
    }

    const uint64_t now_ms = static_cast<uint64_t>(LibXR::Thread::GetTime());
    return now_ms >= deadline_ms ? 0U : static_cast<uint32_t>(deadline_ms - now_ms);
  }

  /**
   * @brief 等待下一条 timestamp 单调前进的同步 IMU。
   */
  LibXR::ErrorCode WaitForNextImu(uint64_t deadline_ms, uint32_t timeout_ms,
                                  ImuStampedT& imu)
  {
    while (true)
    {
      const auto wait_ans =
          imu_sub_.Wait(RemainingMs(deadline_ms, timeout_ms));
      if (wait_ans != LibXR::ErrorCode::OK)
      {
        return wait_ans;
      }

      const uint64_t imu_timestamp_us =
          static_cast<uint64_t>(latest_imu_.timestamp_us);
      if (last_imu_valid_ && imu_timestamp_us <= last_imu_timestamp_us_)
      {
        // SyncSubscriber 可能留下旧信号量计数；消息内容是最新值，旧计数直接吃掉。
        continue;
      }

      last_imu_valid_ = true;
      last_imu_timestamp_us_ = imu_timestamp_us;
      imu = latest_imu_;
      return LibXR::ErrorCode::OK;
    }
  }

  /**
   * @brief 从共享图像流中找到指定 timestamp 的图像。
   *
   * 早于目标 IMU 的图像直接释放；晚于目标 IMU 的图像保留在 pending_image_，等待
   * 后续 IMU 追上，避免因为一次错过就丢掉可能可配对的最新图像。
   */
  LibXR::ErrorCode WaitForMatchingImage(uint64_t imu_timestamp_us,
                                        uint64_t deadline_ms,
                                        uint32_t timeout_ms,
                                        ImageData& image)
  {
    while (true)
    {
      if (pending_image_.Empty())
      {
        const auto wait_ans = image_sub_.Wait(
            pending_image_, RemainingMs(deadline_ms, timeout_ms));
        if (wait_ans != LibXR::ErrorCode::OK)
        {
          return wait_ans;
        }
      }

      const auto* frame = pending_image_.GetData();
      if (frame == nullptr)
      {
        pending_image_.Reset();
        continue;
      }

      const uint64_t image_timestamp_us =
          static_cast<uint64_t>(frame->timestamp_us);
      if (image_timestamp_us < imu_timestamp_us)
      {
        pending_image_.Reset();
        continue;
      }
      if (image_timestamp_us == imu_timestamp_us)
      {
        image = std::move(pending_image_);
        return LibXR::ErrorCode::OK;
      }

      return LibXR::ErrorCode::EMPTY;
    }
  }

  LibXR::ErrorCode WaitForLatestImage(uint64_t deadline_ms,
                                      uint32_t timeout_ms,
                                      ImageData& image)
  {
    while (true)
    {
      if (pending_image_.Empty())
      {
        const auto wait_ans = image_sub_.Wait(
            pending_image_, RemainingMs(deadline_ms, timeout_ms));
        if (wait_ans != LibXR::ErrorCode::OK)
        {
          return wait_ans;
        }
      }

      const auto* frame = pending_image_.GetData();
      if (frame == nullptr)
      {
        pending_image_.Reset();
        continue;
      }

      image = std::move(pending_image_);
      return LibXR::ErrorCode::OK;
    }
  }

  LibXR::ErrorCode WaitForLatestQueuedImu(uint64_t deadline_ms,
                                          uint32_t timeout_ms,
                                          ImuStampedT& imu)
  {
    while (true)
    {
      if (!TryDrainLatestQueuedImu(imu))
      {
        const auto wait_ans =
            imu_sub_.Wait(RemainingMs(deadline_ms, timeout_ms));
        if (wait_ans != LibXR::ErrorCode::OK)
        {
          return wait_ans;
        }
        continue;
      }
      return LibXR::ErrorCode::OK;
    }
  }

  bool TryDrainLatestQueuedImu(ImuStampedT& imu)
  {
    bool updated = false;
    ImuMessage msg{};
    while (imu_queue_.Pop(msg) == LibXR::ErrorCode::OK)
    {
      const uint64_t timestamp_us =
          static_cast<uint64_t>(msg.data.timestamp_us);
      if (last_imu_valid_ && timestamp_us <= last_imu_timestamp_us_)
      {
        continue;
      }
      latest_queued_imu_ = msg.data;
      latest_queued_timestamp_us_ = timestamp_us;
      have_latest_queued_imu_ = true;
      updated = true;
    }

    if (updated || have_latest_queued_imu_)
    {
      imu = latest_queued_imu_;
      last_imu_valid_ = true;
      last_imu_timestamp_us_ = latest_queued_timestamp_us_;
      return true;
    }
    return false;
  }

 private:
  std::string image_topic_name_{};
  std::string imu_topic_name_{};
  std::string host_topic_domain_name_{};
  bool latest_image_mode_{false};
  LibXR::Topic::Domain host_topic_domain_;
  typename ImageTopicT::Subscriber image_sub_;
  ImuStampedT latest_imu_{};
  LibXR::Topic::SyncSubscriber<ImuStampedT> imu_sub_;
  LibXR::LockFreeQueue<ImuMessage> imu_queue_{imu_queue_capacity};
  LibXR::Topic::QueuedSubscriber imu_queue_sub_;
  ImuStampedT latest_queued_imu_{};
  uint64_t latest_queued_timestamp_us_{0};
  bool have_latest_queued_imu_{false};
  bool last_imu_valid_{false};
  uint64_t last_imu_timestamp_us_{0};
  ImageData pending_image_{};
};
