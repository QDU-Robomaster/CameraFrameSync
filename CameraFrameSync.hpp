#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Camera sink that publishes one shared synced frame
constructor_args:
  - camera: '@Camera_0'
  - output_topic_name: camera_frame_sync
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
#include <type_traits>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "linux_shared_topic.hpp"
#include "logger.hpp"

template <CameraTypes::CameraInfo CameraInfoV>
using CameraSinkBase = typename CameraBase<CameraInfoV>::Sink;

/**
 * @brief 相机帧直写 sink。
 *
 * 该模块不再订阅图像 topic，而是直接注册到 `CameraBase`：
 * 1. 相机线程先调用 `AcquireFrame(...)` 预借最终 shared payload；
 * 2. 相机把像素直接写进借出的图像缓冲区；
 * 3. 写完后调用 `CommitFrame(...)` 发布整帧。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CameraFrameSync : public LibXR::Application, public CameraSinkBase<CameraInfoV>
{
 public:
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using Pose = typename Base::Pose;
  using Motion = typename Base::Motion;
  using FrameContext = typename Base::FrameContext;
  using FrameLease = typename Base::FrameLease;
  using SharedImageFrame = typename Base::SharedImageFrame;

  struct SyncedFrame
  {
    SharedImageFrame image;
    Pose pose;
    Motion motion;
  };

  using OutputTopic = LibXR::LinuxSharedTopic<SyncedFrame>;
  using OutputData = typename OutputTopic::Data;

  static inline constexpr CameraInfo camera_info = CameraInfoV;
  // 这一份 shared topic 只承担“前半段最终帧”输出，槽位数量可以保持紧凑。
  static constexpr LibXR::LinuxSharedTopicConfig output_topic_config{
      .slot_num = 4,
      .subscriber_num = 8,
      .queue_num = 4,
  };

  static_assert(std::is_trivially_copyable_v<SyncedFrame>,
                "SyncedFrame must be trivially copyable");

  explicit CameraFrameSync([[maybe_unused]] LibXR::HardwareContainer& hw,
                           LibXR::ApplicationManager& app, Base& camera,
                           const char* output_topic_name = "camera_frame_sync")
      : camera_(camera), output_topic_(output_topic_name, output_topic_config)
  {
    if (!output_topic_.Valid())
    {
      XR_LOG_ERROR("CameraFrameSync failed to create output topic: %s", output_topic_name);
      throw std::runtime_error("CameraFrameSync: output topic creation failed");
    }
    if (!camera_.RegisterSink(*this))
    {
      throw std::runtime_error("CameraFrameSync: camera sink registration failed");
    }

    app.Register(*this);
    XR_LOG_PASS("CameraFrameSync enabled: output=%s", output_topic_name);
  }

  ~CameraFrameSync() override
  {
    camera_.UnregisterSink(*this);
    ResetPendingFrame();
  }

  void OnMonitor() override {}

  bool AcquireFrame(const FrameContext& context, FrameLease& lease) override
  {
    ClearLease(lease);

    if (pending_frame_active_)
    {
      XR_LOG_ERROR(
          "CameraFrameSync: previous frame was not released before acquiring a new one");
      ResetPendingFrame();
    }

    const auto create_ans = output_topic_.CreateData(pending_frame_data_);
    if (create_ans != LibXR::ErrorCode::OK)
    {
      XR_LOG_ERROR("CameraFrameSync output slot unavailable: err=%d",
                   static_cast<int>(create_ans));
      return false;
    }

    SyncedFrame* frame = pending_frame_data_.GetData();
    if (frame == nullptr)
    {
      XR_LOG_ERROR("CameraFrameSync output payload is null");
      ResetPendingFrame();
      return false;
    }

    frame->image.timestamp_us = context.timestamp_us;
    frame->image.sequence = context.sequence;
    frame->pose = context.pose;
    frame->motion = context.motion;

    // 相机线程会直接把像素写进这块最终 shared payload。
    lease.image_data = frame->image.data.data();
    lease.image_step = camera_info.step;
    lease.private_data = frame;
    pending_frame_active_ = true;
    return true;
  }

  void CommitFrame(FrameLease& lease) override
  {
    if (!pending_frame_active_)
    {
      XR_LOG_ERROR("CameraFrameSync: CommitFrame called without an active frame");
      ClearLease(lease);
      return;
    }

    const auto publish_ans = output_topic_.Publish(pending_frame_data_);
    if (publish_ans != LibXR::ErrorCode::OK)
    {
      XR_LOG_ERROR("CameraFrameSync output publish failed: err=%d",
                   static_cast<int>(publish_ans));
      ResetPendingFrame();
      ClearLease(lease);
      return;
    }

    pending_frame_active_ = false;
    ClearLease(lease);
  }

  void AbortFrame(FrameLease& lease) override
  {
    ResetPendingFrame();
    ClearLease(lease);
  }

 private:
  static void ClearLease(FrameLease& lease) { lease = {}; }

  void ResetPendingFrame()
  {
    if (pending_frame_active_)
    {
      pending_frame_data_.Reset();
      pending_frame_active_ = false;
    }
  }

  Base& camera_;
  OutputTopic output_topic_;
  // 当前这一个 Data 对象表示“已借出但尚未发布”的共享槽位。
  OutputData pending_frame_data_{};
  bool pending_frame_active_ = false;
};
