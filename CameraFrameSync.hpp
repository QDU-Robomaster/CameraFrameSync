#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Shared synced-frame payload definition for camera_frame_sync
constructor_args: []
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

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "CameraBase.hpp"
#include "libxr.hpp"
#include "linux_shared_topic.hpp"

/**
 * @brief `camera_frame_sync(shared)` 的最终共享载荷定义。
 *
 * 这里不再承担运行时中转职责，只保留：
 * 1. 单帧 shared payload 的 ABI；
 * 2. 话题类型别名；
 * 3. 默认的单槽、非阻塞配置。
 */
template <CameraTypes::CameraInfo CameraInfoV>
struct CameraFrameSync
{
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;

  static inline constexpr CameraInfo camera_info = CameraInfoV;
  static constexpr std::size_t frame_data_alignment = 64;
  static constexpr std::size_t frame_bytes =
      static_cast<std::size_t>(camera_info.step) * static_cast<std::size_t>(camera_info.height);

  struct Pose
  {
    std::array<float, 4> rotation_wxyz;
    std::array<float, 3> translation_xyz;
  };

  struct Motion
  {
    std::array<float, 3> angular_velocity_xyz;
    std::array<float, 3> linear_acceleration_xyz;
  };

  struct alignas(frame_data_alignment) ImageFrame
  {
    uint64_t timestamp_us;
    uint64_t sequence;
    alignas(frame_data_alignment) std::array<uint8_t, frame_bytes> data;
  };

  struct SyncedFrame
  {
    ImageFrame image;
    Pose pose;
    Motion motion;
  };

  using OutputTopic = LibXR::LinuxSharedTopic<SyncedFrame>;
  using OutputData = typename OutputTopic::Data;

  // 默认预留 8 个 in-flight 槽位，给后续多线程流水线留余量；
  // 生产者路径仍不等待空槽，拿不到可用槽位时直接丢帧。
  static constexpr LibXR::LinuxSharedTopicConfig output_topic_config{
      .slot_num = 8,
      .subscriber_num = 8,
      .queue_num = 2,
  };

  static_assert(frame_bytes > 0, "CameraFrameSync requires non-zero frame bytes");
  static_assert(std::is_trivially_copyable_v<Pose>, "Pose must be trivially copyable");
  static_assert(std::is_trivially_copyable_v<Motion>, "Motion must be trivially copyable");
  static_assert(std::is_trivially_copyable_v<ImageFrame>,
                "ImageFrame must be trivially copyable");
  static_assert(std::is_standard_layout_v<ImageFrame>,
                "ImageFrame must be standard layout");
  static_assert(std::is_trivially_copyable_v<SyncedFrame>,
                "SyncedFrame must be trivially copyable");
  static_assert(alignof(ImageFrame) >= frame_data_alignment,
                "ImageFrame alignment is too small");
  static_assert(offsetof(ImageFrame, data) % frame_data_alignment == 0,
                "ImageFrame data must be aligned");
};
