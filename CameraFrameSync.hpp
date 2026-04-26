#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机共享图像传输与 image+imu 同步订阅器 / Shared image transport plus image+imu sync subscriber
constructor_args:
  camera: @camera
template_args:
  - Info:
      width: 1280
      height: 720
      step: 3840
      encoding: CameraTypes::Encoding::BGR8
      camera_matrix: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
      distortion_model: CameraTypes::DistortionModel::PLUMB_BOB
      distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
      rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      projection_matrix: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
required_hardware: []
depends:
  - qdu-future/CameraBase
=== END MANIFEST === */
// clang-format on

#include <cstdint>
#include <stdexcept>
#include <utility>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "linux_shared_topic.hpp"
#include "logger.hpp"

template <CameraTypes::CameraInfo CameraInfoV>
class CameraFrameSync : public CameraBase<CameraInfoV>::ImageLeaseSink
{
 public:
  using Self = CameraFrameSync<CameraInfoV>;
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using ImageFrame = typename Base::ImageFrame;
  using ImuStamped = typename Base::ImuStamped;
  using ImageTopic = LibXR::LinuxSharedTopic<ImageFrame>;
  using ImageData = typename ImageTopic::Data;

  static inline constexpr CameraInfo camera_info = CameraInfoV;
  static constexpr LibXR::LinuxSharedTopicConfig image_topic_config{
      .slot_num = 8,
      .subscriber_num = 8,
      .queue_num = 2,
  };

  struct SyncedFrame
  {
    ImageData image{};
    ImuStamped imu{};

    const ImageFrame* GetImageFrame() const { return image.GetData(); }
  };

  class Subscriber
  {
   public:
    explicit Subscriber(const Self& sync)
        : image_sub_(sync.ImageTopicName()), imu_sub_(sync.ImuTopicName())
    {
      imu_sub_.StartWaiting();
    }

    Subscriber(const char* image_topic_name, const char* imu_topic_name)
        : image_sub_(image_topic_name), imu_sub_(imu_topic_name)
    {
      imu_sub_.StartWaiting();
    }

    bool Valid() const { return image_sub_.Valid(); }

    LibXR::ErrorCode Wait(SyncedFrame& out, uint32_t timeout_ms)
    {
      while (true)
      {
        ImageData image_data;
        const auto wait_ans = image_sub_.Wait(image_data, timeout_ms);
        if (wait_ans != LibXR::ErrorCode::OK)
        {
          return wait_ans;
        }

        const ImageFrame* image = image_data.GetData();
        if (image == nullptr)
        {
          LogNullImage();
          image_data.Reset();
          continue;
        }

        if (!imu_sub_.Available())
        {
          LogImuNotReady();
          image_data.Reset();
          continue;
        }

        const ImuStamped imu = imu_sub_.GetData();
        imu_sub_.StartWaiting();
        if (imu.timestamp_us != image->timestamp_us)
        {
          LogImuTimestampMismatch(image->timestamp_us, imu.timestamp_us);
          image_data.Reset();
          continue;
        }

        out.image = std::move(image_data);
        out.imu = imu;
        return LibXR::ErrorCode::OK;
      }
    }

   private:
    void LogNullImage()
    {
      null_image_count_++;
      if (null_image_count_ == 1 || null_image_count_ % 200 == 0)
      {
        XR_LOG_ERROR("CameraFrameSync::Subscriber: received null image lease (count=%llu)",
                     static_cast<unsigned long long>(null_image_count_));
      }
    }

    void LogImuNotReady()
    {
      imu_not_ready_count_++;
      if (imu_not_ready_count_ == 1 || imu_not_ready_count_ % 200 == 0)
      {
        XR_LOG_ERROR(
            "CameraFrameSync::Subscriber: image arrived before imu was ready (count=%llu)",
            static_cast<unsigned long long>(imu_not_ready_count_));
      }
    }

    void LogImuTimestampMismatch(uint64_t image_timestamp_us, uint64_t imu_timestamp_us)
    {
      imu_timestamp_mismatch_count_++;
      if (imu_timestamp_mismatch_count_ == 1 ||
          imu_timestamp_mismatch_count_ % 200 == 0)
      {
        XR_LOG_ERROR(
            "CameraFrameSync::Subscriber: image/imu timestamp mismatch image=%llu imu=%llu (count=%llu)",
            static_cast<unsigned long long>(image_timestamp_us),
            static_cast<unsigned long long>(imu_timestamp_us),
            static_cast<unsigned long long>(imu_timestamp_mismatch_count_));
      }
    }

   private:
    typename ImageTopic::Subscriber image_sub_;
    LibXR::Topic::ASyncSubscriber<ImuStamped> imu_sub_;
    uint64_t null_image_count_{0};
    uint64_t imu_not_ready_count_{0};
    uint64_t imu_timestamp_mismatch_count_{0};
  };

  CameraFrameSync(LibXR::HardwareContainer&, LibXR::ApplicationManager&, Base& camera)
      : camera_(camera),
        image_topic_name_(camera.ImageTopicName()),
        imu_topic_name_(camera.ImuTopicName()),
        image_topic_(image_topic_name_, image_topic_config)
  {
    if (!image_topic_.Valid())
    {
      XR_LOG_ERROR("CameraFrameSync: failed to create image topic: %s", image_topic_name_);
      throw std::runtime_error("CameraFrameSync: image topic creation failed");
    }
    if (!camera_.RegisterImageSink(*this))
    {
      writable_image_lease_.Reset();
      throw std::runtime_error("CameraFrameSync: image sink registration failed");
    }

    XR_LOG_PASS("CameraFrameSync enabled: image=%s imu=%s slot_num=%u queue_num=%u",
                image_topic_name_, imu_topic_name_,
                static_cast<unsigned>(image_topic_config.slot_num),
                static_cast<unsigned>(image_topic_config.queue_num));
  }

  const char* ImageTopicName() const { return image_topic_name_; }

  const char* ImuTopicName() const { return imu_topic_name_; }

 private:
  ImageFrame* EnsureWritableImageLease()
  {
    if (writable_image_lease_.GetData() != nullptr)
    {
      return writable_image_lease_.GetData();
    }

    const auto create_ans = image_topic_.CreateData(writable_image_lease_);
    if (create_ans != LibXR::ErrorCode::OK)
    {
      XR_LOG_ERROR("CameraFrameSync: AcquireWritableImage CreateData failed err=%d",
                   static_cast<int>(create_ans));
      return nullptr;
    }
    if (writable_image_lease_.GetData() == nullptr)
    {
      XR_LOG_ERROR("CameraFrameSync: acquired writable image is null");
      writable_image_lease_.Reset();
      return nullptr;
    }
    return writable_image_lease_.GetData();
  }

  ImageFrame* AcquireWritableImage() override { return EnsureWritableImageLease(); }

  ImageFrame* CommitAndAcquireNextWritableImage(
      ImageFrame& committed_image) override
  {
    ImageFrame* current_writable_image = writable_image_lease_.GetData();
    if (current_writable_image == nullptr)
    {
      XR_LOG_ERROR("CameraFrameSync: current writable image is null");
      return nullptr;
    }
    if (&committed_image != current_writable_image)
    {
      XR_LOG_ERROR("CameraFrameSync: committed image does not match current writable slot");
      return nullptr;
    }

    ImageData next_writable_image_lease;
    const auto create_ans = image_topic_.CreateData(next_writable_image_lease);
    if (create_ans != LibXR::ErrorCode::OK)
    {
      dropped_image_count_++;
      if (dropped_image_count_ == 1 || dropped_image_count_ % 200 == 0)
      {
        XR_LOG_WARN(
            "CameraFrameSync: no spare image slot, drop current image and reuse current writable slot (dropped=%llu, err=%d)",
            static_cast<unsigned long long>(dropped_image_count_),
            static_cast<int>(create_ans));
      }
      return current_writable_image;
    }

    const auto publish_ans = image_topic_.Publish(writable_image_lease_);
    if (publish_ans != LibXR::ErrorCode::OK)
    {
      publish_fail_count_++;
      if (publish_fail_count_ == 1 || publish_fail_count_ % 200 == 0)
      {
        XR_LOG_WARN("CameraFrameSync: image publish failed (count=%llu, err=%d)",
                    static_cast<unsigned long long>(publish_fail_count_),
                    static_cast<int>(publish_ans));
      }
    }

    writable_image_lease_ = std::move(next_writable_image_lease);
    return writable_image_lease_.GetData();
  }

 private:
  Base& camera_;
  const char* image_topic_name_;
  const char* imu_topic_name_;
  ImageTopic image_topic_;
  ImageData writable_image_lease_{};
  uint64_t dropped_image_count_{0};
  uint64_t publish_fail_count_{0};
};
