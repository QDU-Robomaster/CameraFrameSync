#pragma once

#include "libxr.hpp"
#include "logger.hpp"

#include <array>
#include <ctime>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>

/**
 * @brief CameraFrameSync 同步结果内录。
 *
 * CameraBase 负责保存图像原始字节；这里只保存同步成功后的图像时间戳、
 * IMU 时间戳映射和最终 IMU 数据。
 */
class CameraFrameSyncRecording
{
 public:
  template <typename RuntimeParam, typename Camera>
  void Open(const RuntimeParam& runtime, const Camera& camera)
  {
    if (!runtime.record_enable)
    {
      return;
    }

    if (camera.RecordingEnabled() && !camera.RecordingFileStemView().empty())
    {
      file_stem_ = std::string(camera.RecordingFileStemView());
    }
    else
    {
      const std::string dir = MakeDefaultDir(camera.NameView());
      file_stem_ = std::filesystem::path(dir).filename().string();
    }

    if (!runtime.record_dir.empty())
    {
      output_dir_ = std::string(runtime.record_dir);
    }
    else if (camera.RecordingEnabled() && !camera.RecordingOutputDirView().empty())
    {
      output_dir_ = std::string(camera.RecordingOutputDirView());
    }
    else
    {
      output_dir_ =
          (std::filesystem::path("runs") / "camera_record" / file_stem_).string();
    }

    std::error_code ec;
    std::filesystem::create_directories(output_dir_, ec);
    if (ec)
    {
      XR_LOG_ERROR("CameraFrameSync: create recording dir failed %s: %s",
                   output_dir_.c_str(), ec.message().c_str());
      ASSERT(false);
      return;
    }

    const auto dir = std::filesystem::path(output_dir_);
    detail_csv_.open(dir / (file_stem_ + "_sync.csv"),
                     std::ios::out | std::ios::trunc);
    capture_imu_csv_.open(dir / (file_stem_ + "_imu.csv"),
                          std::ios::out | std::ios::trunc);
    if (!detail_csv_.is_open() || !capture_imu_csv_.is_open())
    {
      XR_LOG_ERROR("CameraFrameSync: open sync recording failed dir=%s stem=%s",
                   output_dir_.c_str(), file_stem_.c_str());
      ASSERT(false);
      return;
    }

    detail_csv_ << "row,image_timestamp_us,sync_imu_timestamp_us,"
                   "final_imu_timestamp_us,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mode\n";
    enabled_ = true;
    XR_LOG_PASS("CameraFrameSync: sync recording enabled dir=%s stem=%s",
                output_dir_.c_str(), file_stem_.c_str());
  }

  void Close()
  {
    if (detail_csv_.is_open())
    {
      detail_csv_.flush();
      detail_csv_.close();
    }
    if (capture_imu_csv_.is_open())
    {
      capture_imu_csv_.flush();
      capture_imu_csv_.close();
    }
    enabled_ = false;
  }

  void Record(uint64_t image_timestamp_us, uint64_t sync_imu_timestamp_us,
              uint64_t final_imu_timestamp_us,
              const std::array<float, 4>& rotation_wxyz,
              const std::array<float, 3>& angular_velocity_xyz,
              const std::array<float, 3>& linear_acceleration_xyz,
              const char* mode_name)
  {
    if (!enabled_)
    {
      return;
    }

    detail_csv_ << row_index_ << ","
                << static_cast<unsigned long long>(image_timestamp_us) << ","
                << static_cast<unsigned long long>(sync_imu_timestamp_us) << ","
                << static_cast<unsigned long long>(final_imu_timestamp_us) << ","
                << rotation_wxyz[0] << "," << rotation_wxyz[1] << ","
                << rotation_wxyz[2] << "," << rotation_wxyz[3] << ","
                << angular_velocity_xyz[0] << "," << angular_velocity_xyz[1]
                << "," << angular_velocity_xyz[2] << ","
                << linear_acceleration_xyz[0] << "," << linear_acceleration_xyz[1]
                << "," << linear_acceleration_xyz[2] << "," << mode_name << "\n";

    capture_imu_csv_ << static_cast<unsigned long long>(image_timestamp_us) << ","
                     << rotation_wxyz[0] << "," << rotation_wxyz[1] << ","
                     << rotation_wxyz[2] << "," << rotation_wxyz[3] << ","
                     << angular_velocity_xyz[0] << "," << angular_velocity_xyz[1]
                     << "," << angular_velocity_xyz[2] << ","
                     << linear_acceleration_xyz[0] << ","
                     << linear_acceleration_xyz[1] << ","
                     << linear_acceleration_xyz[2] << "\n";

    if (!detail_csv_.good() || !capture_imu_csv_.good())
    {
      XR_LOG_ERROR("CameraFrameSync: sync recording write failed row=%u",
                   static_cast<unsigned>(row_index_));
      ASSERT(false);
      return;
    }

    ++row_index_;
    detail_csv_.flush();
    capture_imu_csv_.flush();
  }

 private:
  static std::string MakeTimestamp()
  {
    std::time_t now = std::time(nullptr);
    std::tm local{};
#if defined(_WIN32)
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif

    std::ostringstream stamp;
    stamp << std::put_time(&local, "%Y%m%d_%H%M%S");
    return stamp.str();
  }

  static std::string SanitizeName(std::string_view camera_name)
  {
    if (camera_name.empty())
    {
      return "camera";
    }

    std::string safe;
    safe.reserve(camera_name.size());
    for (char ch : camera_name)
    {
      const bool ok = (ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z') ||
                      (ch >= 'a' && ch <= 'z') || ch == '_' || ch == '-';
      safe.push_back(ok ? ch : '_');
    }
    return safe;
  }

  static std::string MakeDefaultDir(std::string_view camera_name)
  {
    return (std::filesystem::path("runs") / "camera_record" /
            (MakeTimestamp() + "_" + SanitizeName(camera_name)))
        .string();
  }

  bool enabled_{false};
  uint64_t row_index_{0};
  std::string output_dir_{};
  std::string file_stem_{};
  std::ofstream detail_csv_{};
  std::ofstream capture_imu_csv_{};
};
