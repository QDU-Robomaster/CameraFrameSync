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
  /**
   * @brief 打开同步记录 CSV。
   *
   * 若 CameraBase 已启用图像内录，则复用 CameraBase 的 stem 和临时目录，使
   * `_frames.*`、`_sync.csv`、`_imu.csv` 落在同一个可恢复记录包中。
   *
   * @tparam RuntimeParam CameraFrameSync 运行时参数类型。
   * @tparam Camera CameraBase 派生相机类型。
   * @param runtime CameraFrameSync 运行时参数。
   * @param camera 上游相机对象。
   */
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

  /**
   * @brief flush 并关闭同步记录文件。
   */
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

  /**
   * @brief 记录一条已成功发布的图像/IMU 同步关系。
   *
   * `_sync.csv` 保存完整调试信息，`_imu.csv` 保存 CaptureFileCamera 可直接
   * 回放的每帧 IMU 数据。
   *
   * @param image_timestamp_us 图像传感器侧时间戳。
   * @param sync_imu_timestamp_us 同步基准 IMU 时间戳。
   * @param final_imu_timestamp_us 加 offset 后实际发布的 IMU 时间戳。
   * @param rotation_wxyz 姿态四元数，顺序为 wxyz。
   * @param angular_velocity_xyz 三轴角速度。
   * @param linear_acceleration_xyz 三轴线加速度。
   * @param mode_name 当前同步模式名称。
   */
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
  /**
   * @brief 生成本地时间戳字符串，格式为 `YYYYMMDD_HHMMSS`。
   */
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

  /**
   * @brief 把相机名整理成可用于文件名的安全字符串。
   */
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

  /**
   * @brief 没有 CameraBase 图像内录时生成独立同步记录目录。
   */
  static std::string MakeDefaultDir(std::string_view camera_name)
  {
    return (std::filesystem::path("runs") / "camera_record" /
            (MakeTimestamp() + "_" + SanitizeName(camera_name)))
        .string();
  }

  bool enabled_{false};  ///< 是否已经成功打开记录文件。
  uint64_t row_index_{0};  ///< 下一条 `_sync.csv` 记录的行号。
  std::string output_dir_{};  ///< 当前同步记录输出目录。
  std::string file_stem_{};  ///< 记录包文件名前缀。
  std::ofstream detail_csv_{};  ///< 完整同步映射 CSV 输出流。
  std::ofstream capture_imu_csv_{};  ///< CaptureFileCamera 回放 IMU CSV 输出流。
};
