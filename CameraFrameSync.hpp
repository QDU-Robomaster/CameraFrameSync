#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: camera_frame_sync shared-topic bridge for CameraBase
constructor_args:
  camera: @camera
  output_topic_name: "camera_frame_sync"
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

/**
 * @brief `camera_frame_sync(shared)` 的运行时桥接模块。
 *
 * `CameraBase` 只负责维护唯一的帧类型和当前可写帧指针；
 * 这个模块负责：
 * 1. 创建共享 topic；
 * 2. 申请第一块可写槽位并注册给 `CameraBase`；
 * 3. 每次相机提交一帧时，先预取下一块槽位，再发布当前槽位；
 * 4. 如果已经只剩最后一个可写槽位，就直接丢掉当前帧并继续复用这块槽位。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CameraFrameSync
{
 public:
  using Self = CameraFrameSync<CameraInfoV>;
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using Frame = typename Base::Frame;
  using OutputTopic = LibXR::LinuxSharedTopic<Frame>;
  using OutputData = typename OutputTopic::Data;

  static inline constexpr CameraInfo camera_info = CameraInfoV;
  static constexpr LibXR::LinuxSharedTopicConfig output_topic_config{
      .slot_num = 8,
      .subscriber_num = 8,
      .queue_num = 2,
  };

  CameraFrameSync(LibXR::HardwareContainer&, LibXR::ApplicationManager&, Base& camera,
                  const char* output_topic_name = "camera_frame_sync")
      : camera_(camera),
        output_topic_name_(SafeCStr(output_topic_name)),
        output_topic_(output_topic_name_, output_topic_config)
  {
    if (!output_topic_.Valid())
    {
      XR_LOG_ERROR("CameraFrameSync: failed to create topic: %s", output_topic_name_);
      throw std::runtime_error("CameraFrameSync: topic creation failed");
    }
    if (!AcquireInitialWritableSlot())
    {
      throw std::runtime_error("CameraFrameSync: initial slot acquisition failed");
    }
    if (!camera_.RegisterSync(this, current_data_.GetData(), CommitFrameAdapter))
    {
      current_data_.Reset();
      throw std::runtime_error("CameraFrameSync: camera registration failed");
    }

    XR_LOG_PASS("CameraFrameSync enabled: output=%s slot_num=%u queue_num=%u",
                output_topic_name_,
                static_cast<unsigned>(output_topic_config.slot_num),
                static_cast<unsigned>(output_topic_config.queue_num));
  }

 private:
  static const char* SafeCStr(const char* text) { return text != nullptr ? text : ""; }

  bool AcquireInitialWritableSlot()
  {
    const auto create_ans = output_topic_.CreateData(current_data_);
    if (create_ans != LibXR::ErrorCode::OK)
    {
      XR_LOG_ERROR("CameraFrameSync: initial CreateData failed err=%d",
                   static_cast<int>(create_ans));
      return false;
    }
    if (current_data_.GetData() == nullptr)
    {
      XR_LOG_ERROR("CameraFrameSync: initial writable frame is null");
      current_data_.Reset();
      return false;
    }
    return true;
  }

  static Frame* CommitFrameAdapter(void* sync_context)
  {
    return static_cast<Self*>(sync_context)->CommitAndLeaseNext();
  }

  Frame* CommitAndLeaseNext()
  {
    Frame* current_frame = current_data_.GetData();
    if (current_frame == nullptr)
    {
      XR_LOG_ERROR("CameraFrameSync: current writable frame is null");
      return nullptr;
    }

    OutputData next_data;
    const auto create_ans = output_topic_.CreateData(next_data);
    if (create_ans != LibXR::ErrorCode::OK)
    {
      dropped_frame_count_++;
      if (dropped_frame_count_ == 1 || dropped_frame_count_ % 200 == 0)
      {
        XR_LOG_WARN(
            "CameraFrameSync: no spare slot, drop current frame and reuse last slot (dropped=%llu, err=%d)",
            static_cast<unsigned long long>(dropped_frame_count_),
            static_cast<int>(create_ans));
      }
      return current_frame;
    }

    const auto publish_ans = output_topic_.Publish(current_data_);
    if (publish_ans != LibXR::ErrorCode::OK)
    {
      publish_fail_count_++;
      if (publish_fail_count_ == 1 || publish_fail_count_ % 200 == 0)
      {
        XR_LOG_WARN("CameraFrameSync: publish failed (count=%llu, err=%d)",
                    static_cast<unsigned long long>(publish_fail_count_),
                    static_cast<int>(publish_ans));
      }
    }

    current_data_ = std::move(next_data);
    return current_data_.GetData();
  }

 private:
  Base& camera_;
  const char* output_topic_name_;
  OutputTopic output_topic_;
  OutputData current_data_{};
  uint64_t dropped_frame_count_{0};
  uint64_t publish_fail_count_{0};
};
