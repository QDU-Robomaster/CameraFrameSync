#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

#include "libxr.hpp"

template <typename ImageTopicT, typename ImuStampedT>
struct CameraFrameSyncSyncedFrame
{
  using ImageData = typename ImageTopicT::Data;

  ImageData image{};
  ImuStampedT imu{};

  const auto* GetImageFrame() const { return image.GetData(); }
};

template <typename ImageTopicT, typename ImuStampedT>
class CameraFrameSyncSubscriber
{
 public:
  using SyncedFrame = CameraFrameSyncSyncedFrame<ImageTopicT, ImuStampedT>;
  using ImageData = typename ImageTopicT::Data;

  template <typename SyncT>
  explicit CameraFrameSyncSubscriber(const SyncT& sync)
      : CameraFrameSyncSubscriber(sync.ImageTopicName(), sync.ImuTopicName(),
                                  sync.HostTopicDomainName())
  {
  }

  CameraFrameSyncSubscriber(std::string_view image_topic_name,
                            std::string_view imu_topic_name)
      : CameraFrameSyncSubscriber(image_topic_name, imu_topic_name, "host")
  {
  }

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

  bool Valid() const { return image_sub_.Valid(); }

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
  static uint64_t MakeDeadline(uint32_t timeout_ms)
  {
    if (timeout_ms == UINT32_MAX)
    {
      return std::numeric_limits<uint64_t>::max();
    }
    return static_cast<uint64_t>(LibXR::Thread::GetTime()) + timeout_ms;
  }

  static uint32_t RemainingMs(uint64_t deadline_ms, uint32_t timeout_ms)
  {
    if (timeout_ms == UINT32_MAX)
    {
      return UINT32_MAX;
    }

    const uint64_t now_ms = static_cast<uint64_t>(LibXR::Thread::GetTime());
    return now_ms >= deadline_ms ? 0U : static_cast<uint32_t>(deadline_ms - now_ms);
  }

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
