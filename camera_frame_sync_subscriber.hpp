#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

#include "libxr.hpp"

/**
 * @brief 一组已同步的图像共享槽位和 IMU 数据。
 *
 * image 持有 LinuxSharedTopic 的共享槽位租约；使用方处理完 out 后释放或覆盖
 * SyncedFrame，槽位才会回到共享图像池。
 */
template <typename ImageTopicT, typename ImuStampedT>
struct CameraFrameSyncSyncedFrame
{
  using ImageData = typename ImageTopicT::Data;

  ImageData image{};  ///< 共享图像槽位租约。
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

  /**
   * @brief 从 CameraFrameSync 实例推导 topic 名称。
   */
  template <typename SyncT>
  explicit CameraFrameSyncSubscriber(const SyncT& sync)
      : CameraFrameSyncSubscriber(sync.ImageTopicName(), sync.ImuTopicName(),
                                  sync.HostTopicDomainName())
  {
  }

  /**
   * @brief 在默认 host domain 中订阅图像和 IMU。
   */
  CameraFrameSyncSubscriber(std::string_view image_topic_name,
                            std::string_view imu_topic_name)
      : CameraFrameSyncSubscriber(image_topic_name, imu_topic_name, "host")
  {
  }

  /**
   * @brief 订阅指定图像 topic、IMU topic 和 Host topic domain。
   */
  CameraFrameSyncSubscriber(std::string_view image_topic_name,
                            std::string_view imu_topic_name,
                            std::string_view host_topic_domain_name)
      : image_topic_name_(image_topic_name),
        imu_topic_name_(imu_topic_name),
        host_topic_domain_name_(host_topic_domain_name),
        host_topic_domain_(host_topic_domain_name_.c_str()),
        image_sub_(image_topic_name_.c_str()),
        imu_sub_(LibXR::Topic(LibXR::Topic::WaitTopic(
                     imu_topic_name_.c_str(), UINT32_MAX, &host_topic_domain_)),
                 latest_imu_)
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
   * 如果先拿到的图像没有对应 IMU，会继续等待；如果 IMU 已经越过该图像时间戳，
   * 说明该图像无法配对，当前图像会被释放并等待下一张。
   */
  LibXR::ErrorCode Wait(SyncedFrame& out, uint32_t timeout_ms)
  {
    const uint64_t deadline_ms = MakeDeadline(timeout_ms);

    while (true)
    {
      ImageData image_data;
      const auto wait_ans =
          image_sub_.Wait(image_data, RemainingMs(deadline_ms, timeout_ms));
      if (wait_ans != LibXR::ErrorCode::OK)
      {
        return wait_ans;
      }

      const auto* image = image_data.GetData();
      if (image == nullptr)
      {
        image_data.Reset();
        continue;
      }

      const auto imu_ans =
          WaitForMatchingImu(static_cast<uint64_t>(image->timestamp_us),
                             deadline_ms, timeout_ms, out.imu);
      if (imu_ans == LibXR::ErrorCode::OK)
      {
        out.image = std::move(image_data);
        return LibXR::ErrorCode::OK;
      }
      if (imu_ans == LibXR::ErrorCode::EMPTY)
      {
        image_data.Reset();
        continue;
      }
      return imu_ans;
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
   * @brief 阻塞等待指定图像时间戳对应的同步 IMU。
   */
  LibXR::ErrorCode WaitForMatchingImu(uint64_t image_timestamp_us,
                                      uint64_t deadline_ms,
                                      uint32_t timeout_ms,
                                      ImuStampedT& matched_imu)
  {
    while (true)
    {
      if (latest_imu_valid_)
      {
        const uint64_t imu_timestamp_us =
            static_cast<uint64_t>(latest_imu_.timestamp_us);
        if (imu_timestamp_us == image_timestamp_us)
        {
          matched_imu = latest_imu_;
          latest_imu_valid_ = false;
          return LibXR::ErrorCode::OK;
        }
        if (imu_timestamp_us > image_timestamp_us)
        {
          return LibXR::ErrorCode::EMPTY;
        }
      }

      const auto wait_ans =
          imu_sub_.Wait(RemainingMs(deadline_ms, timeout_ms));
      if (wait_ans != LibXR::ErrorCode::OK)
      {
        return wait_ans;
      }
      latest_imu_valid_ = true;
    }
  }

 private:
  std::string image_topic_name_{};
  std::string imu_topic_name_{};
  std::string host_topic_domain_name_{};
  LibXR::Topic::Domain host_topic_domain_;
  typename ImageTopicT::Subscriber image_sub_;
  ImuStampedT latest_imu_{};
  LibXR::Topic::SyncSubscriber<ImuStampedT> imu_sub_;
  bool latest_imu_valid_{false};
};
