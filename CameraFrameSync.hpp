#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Host-side camera-frame synchronizer
constructor_args:
  - image_topic_name: image_frame
  - pose_topic_name: camera_pose
  - motion_topic_name: camera_motion
  - output_topic_name: camera_frame_sync
  - image_wait_timeout_ms: 100
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

#include <atomic>
#include <cstdint>
#include <array>
#include <stdexcept>
#include <type_traits>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "linux_shared_topic.hpp"
#include "logger.hpp"
#include "thread.hpp"
#include "transform.hpp"

/**
 * @brief 以图像帧为锚，对齐姿态与运动信息，并重新发布为单一共享帧。
 *
 * 当前只实现 mode1：
 * 1. 生产端保证 `pose / motion` 先发，`image` 后发；
 * 2. 消费端在等待下一帧图像前，先把异步姿态与运动订阅重新置为 waiting；
 * 3. 只有三路时间戳完全一致时，才会产出一帧同步结果。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CameraFrameSync : public LibXR::Application
{
 public:
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using SharedImageFrame = typename Base::SharedImageFrame;
  using ImageTopic = LibXR::LinuxSharedTopic<SharedImageFrame>;

  static inline constexpr CameraInfo camera_info = CameraInfoV;
  static constexpr LibXR::LinuxSharedTopicConfig output_topic_config{
      .slot_num = 4,
      .subscriber_num = 8,
      .queue_num = 4,
  };

  struct PoseStamped
  {
    LibXR::MicrosecondTimestamp timestamp{};
    LibXR::Quaternion<float> rotation{};
    LibXR::Position<float> translation{};
  };

  struct MotionStamped
  {
    LibXR::MicrosecondTimestamp timestamp{};
    LibXR::Position<float> angular_velocity{};
    LibXR::Position<float> linear_acceleration{};
  };

  struct PackedPose
  {
    std::array<float, 4> rotation_wxyz{};
    std::array<float, 3> translation_xyz{};
  };

  struct PackedMotion
  {
    std::array<float, 3> angular_velocity_xyz{};
    std::array<float, 3> linear_acceleration_xyz{};
  };

  struct SyncedFrame
  {
    SharedImageFrame image{};
    PackedPose pose{};
    PackedMotion motion{};
  };

  static_assert(std::is_trivially_copyable_v<PackedPose>,
                "PackedPose must be trivially copyable");
  static_assert(std::is_trivially_copyable_v<PackedMotion>,
                "PackedMotion must be trivially copyable");
  static_assert(std::is_trivially_copyable_v<SyncedFrame>,
                "SyncedFrame must be trivially copyable");

  enum class Mode : uint8_t
  {
    IMAGE_ANCHORED_ASYNC_STATE = 0,
  };

  struct Config
  {
    const char* image_topic_name = "image_frame";
    const char* pose_topic_name = "camera_pose";
    const char* motion_topic_name = "camera_motion";
    const char* output_topic_name = "camera_frame_sync";
    uint32_t image_wait_timeout_ms = 100;
    Mode mode = Mode::IMAGE_ANCHORED_ASYNC_STATE;
  };

  explicit CameraFrameSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                           const char* image_topic_name = "image_frame",
                           const char* pose_topic_name = "camera_pose",
                           const char* motion_topic_name = "camera_motion",
                           const char* output_topic_name = "camera_frame_sync",
                           uint32_t image_wait_timeout_ms = 100)
      : CameraFrameSync(hw, app,
                        Config{
                            .image_topic_name = image_topic_name,
                            .pose_topic_name = pose_topic_name,
                            .motion_topic_name = motion_topic_name,
                            .output_topic_name = output_topic_name,
                            .image_wait_timeout_ms = image_wait_timeout_ms,
                        })
  {
  }

  explicit CameraFrameSync(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                           Config cfg)
      : cfg_(cfg), output_topic_(cfg_.output_topic_name, output_topic_config)
  {
    (void)hw;

    if (!output_topic_.Valid())
    {
      XR_LOG_ERROR("CameraFrameSync failed to create output topic: %s",
                   cfg_.output_topic_name);
      throw std::runtime_error("CameraFrameSync: output topic creation failed");
    }

    running_.store(true, std::memory_order_release);
    worker_thread_.Create(this, ThreadFun, "CameraFrameSync",
                          static_cast<size_t>(1024 * 128),
                          LibXR::Thread::Priority::REALTIME);
    app.Register(*this);

    XR_LOG_PASS(
        "CameraFrameSync enabled: image=%s pose=%s motion=%s output=%s mode=%u",
        cfg_.image_topic_name, cfg_.pose_topic_name, cfg_.motion_topic_name,
        cfg_.output_topic_name, static_cast<unsigned>(cfg_.mode));
  }

  ~CameraFrameSync() override { running_.store(false, std::memory_order_release); }

  void OnMonitor() override {}

 private:
  static void ArmNextFrame(LibXR::Topic::ASyncSubscriber<PoseStamped>& pose_subscriber,
                           LibXR::Topic::ASyncSubscriber<MotionStamped>& motion_subscriber)
  {
    pose_subscriber.StartWaiting();
    motion_subscriber.StartWaiting();
  }

  static PackedPose PackPose(const PoseStamped& pose)
  {
    PackedPose packed;
    packed.rotation_wxyz = {
        pose.rotation.w(),
        pose.rotation.x(),
        pose.rotation.y(),
        pose.rotation.z(),
    };
    packed.translation_xyz = {
        pose.translation[0],
        pose.translation[1],
        pose.translation[2],
    };
    return packed;
  }

  static PackedMotion PackMotion(const MotionStamped& motion)
  {
    PackedMotion packed;
    packed.angular_velocity_xyz = {
        motion.angular_velocity[0],
        motion.angular_velocity[1],
        motion.angular_velocity[2],
    };
    packed.linear_acceleration_xyz = {
        motion.linear_acceleration[0],
        motion.linear_acceleration[1],
        motion.linear_acceleration[2],
    };
    return packed;
  }

  static void ThreadFun(CameraFrameSync* self)
  {
    if (self->cfg_.mode != Mode::IMAGE_ANCHORED_ASYNC_STATE)
    {
      XR_LOG_ERROR("CameraFrameSync mode=%u is not implemented",
                   static_cast<unsigned>(self->cfg_.mode));
      return;
    }

    auto pose_subscriber =
        LibXR::Topic::ASyncSubscriber<PoseStamped>(self->cfg_.pose_topic_name);
    auto motion_subscriber =
        LibXR::Topic::ASyncSubscriber<MotionStamped>(self->cfg_.motion_topic_name);

    ArmNextFrame(pose_subscriber, motion_subscriber);
    XR_LOG_PASS("CameraFrameSync mode1 armed: wait image after pose/motion");

    while (self->running_.load(std::memory_order_acquire))
    {
      typename ImageTopic::Subscriber image_subscriber(
          self->cfg_.image_topic_name,
          LibXR::LinuxSharedSubscriberMode::BROADCAST_DROP_OLD);
      if (!image_subscriber.Valid())
      {
        XR_LOG_ERROR("CameraFrameSync failed to attach image topic: %s",
                     self->cfg_.image_topic_name);
        LibXR::Thread::Sleep(200);
        continue;
      }

      typename ImageTopic::Data image_data;
      while (self->running_.load(std::memory_order_acquire))
      {
        const auto wait_ans =
            image_subscriber.Wait(image_data, self->cfg_.image_wait_timeout_ms);
        if (wait_ans == LibXR::ErrorCode::TIMEOUT)
        {
          continue;
        }
        if (wait_ans != LibXR::ErrorCode::OK)
        {
          XR_LOG_ERROR("CameraFrameSync image wait failed: err=%d",
                       static_cast<int>(wait_ans));
          image_data.Reset();
          break;
        }

        const SharedImageFrame* image = image_data.GetData();
        if (image == nullptr)
        {
          XR_LOG_ERROR("CameraFrameSync received null image payload");
          ArmNextFrame(pose_subscriber, motion_subscriber);
          image_data.Reset();
          continue;
        }

        const uint64_t image_timestamp_us = image->timestamp_us;
        if (!pose_subscriber.Available())
        {
          XR_LOG_ERROR(
              "CameraFrameSync dropped image ts=%llu because pose is not ready",
              static_cast<unsigned long long>(image_timestamp_us));
          ArmNextFrame(pose_subscriber, motion_subscriber);
          image_data.Reset();
          continue;
        }
        if (!motion_subscriber.Available())
        {
          XR_LOG_ERROR(
              "CameraFrameSync dropped image ts=%llu because motion is not ready",
              static_cast<unsigned long long>(image_timestamp_us));
          ArmNextFrame(pose_subscriber, motion_subscriber);
          image_data.Reset();
          continue;
        }

        const PoseStamped pose = pose_subscriber.GetData();
        const MotionStamped motion = motion_subscriber.GetData();
        if (static_cast<uint64_t>(pose.timestamp) != image_timestamp_us)
        {
          XR_LOG_ERROR(
              "CameraFrameSync dropped image ts=%llu because pose ts=%llu mismatched",
              static_cast<unsigned long long>(image_timestamp_us),
              static_cast<unsigned long long>(pose.timestamp));
          ArmNextFrame(pose_subscriber, motion_subscriber);
          image_data.Reset();
          continue;
        }
        if (static_cast<uint64_t>(motion.timestamp) != image_timestamp_us)
        {
          XR_LOG_ERROR(
              "CameraFrameSync dropped image ts=%llu because motion ts=%llu mismatched",
              static_cast<unsigned long long>(image_timestamp_us),
              static_cast<unsigned long long>(motion.timestamp));
          ArmNextFrame(pose_subscriber, motion_subscriber);
          image_data.Reset();
          continue;
        }

        typename LibXR::LinuxSharedTopic<SyncedFrame>::Data synced_data;
        const auto create_ans = self->output_topic_.CreateData(synced_data);
        if (create_ans != LibXR::ErrorCode::OK)
        {
          XR_LOG_ERROR("CameraFrameSync output slot unavailable: err=%d",
                       static_cast<int>(create_ans));
          ArmNextFrame(pose_subscriber, motion_subscriber);
          image_data.Reset();
          continue;
        }

        SyncedFrame* synced = synced_data.GetData();
        if (synced == nullptr)
        {
          XR_LOG_ERROR("CameraFrameSync output payload is null");
          ArmNextFrame(pose_subscriber, motion_subscriber);
          image_data.Reset();
          synced_data.Reset();
          continue;
        }

        synced->image = *image;
        synced->pose = PackPose(pose);
        synced->motion = PackMotion(motion);

        const auto publish_ans = self->output_topic_.Publish(synced_data);
        if (publish_ans != LibXR::ErrorCode::OK)
        {
          XR_LOG_ERROR("CameraFrameSync output publish failed: err=%d",
                       static_cast<int>(publish_ans));
          ArmNextFrame(pose_subscriber, motion_subscriber);
          image_data.Reset();
          continue;
        }

        ArmNextFrame(pose_subscriber, motion_subscriber);
        image_data.Reset();
      }
    }
  }

  Config cfg_{};
  LibXR::LinuxSharedTopic<SyncedFrame> output_topic_;
  std::atomic<bool> running_{false};
  LibXR::Thread worker_thread_{};
};
