# CameraFrameSync

`CameraFrameSync` 是相机帧和 IMU 的同步桥：

- 从 `CameraBase<Info>` 拿可写图像槽位，把已提交图像发布到 `LinuxSharedTopic`
- 按配置的前缀订阅原始 `gyro / accl / quat`
- 发布与图像 `timestamp_us` 相同的同步后 `ImuStamped`

模块不创建线程。原始 IMU 只在 Topic 回调里入队；`CameraSync` 回执只记录当前 probe 的命中结果。状态机只在图像提交时推进。

## Topic

输入：

- 图像：`CameraBase<Info>::RegisterImageSink(...)`
- 原始陀螺：`<raw_imu_topic_prefix>_gyro`，payload 为 `Eigen::Matrix<float, 3, 1>`，单位 rad/s
- 原始加速度：`<raw_imu_topic_prefix>_accl`，payload 为 `Eigen::Matrix<float, 3, 1>`，单位 m/s^2
- 原始姿态：`<raw_imu_topic_prefix>_quat`，payload 为 `LibXR::Quaternion<float>`
- 同步回执：`camera_sync_result`，payload 为 `CameraSync::SyncEvent`

输出：

- 图像共享 topic：`camera.ImageTopicName()`
- 同步 IMU topic：`camera.ImuTopicName()`
- 同步命令：`camera_sync_command`，payload 为 `CameraSync::SyncCommand`

实机和 Webots 都应通过 SharedTopic 转发 `camera_sync_command / camera_sync_result`。Host 侧默认使用 `host` topic domain。`raw_imu_topic_prefix` 为空时沿用 `camera.Name()`，实机如果下位机把云台 IMU 暴露为 `gimbal_gyro / gimbal_accl / gimbal_quat`，应显式配成 `gimbal`。

## 时间戳契约

只使用传感器侧时间戳：

- 图像时间来自 `ImageFrame::timestamp_us`
- 原始 IMU 时间来自 Topic envelope timestamp
- `CameraSync::SyncEvent` 的同步点时间来自 result topic 的 envelope timestamp

相机时间和 IMU 时间不做绝对值比较。相机侧只看相邻图像的时间差，IMU 侧只看相邻 IMU 的时间差和 `CameraSync` 回传的 IMU 时间戳。

## 原始 IMU 组装

IMU 组装以 gyro 为主轴：

1. gyro/accl/quat 三路回调分别把样本放进固定长度队列
2. 图像提交时，把队列推进到能确定为止
3. 取 gyro 队首时间戳
4. 丢掉所有早于该 gyro 的 accl / quat
5. 如果 accl / quat 缺失，先停住，不丢 gyro
6. 如果 accl / quat 已经晚于该 gyro，说明这条 gyro 缺帧，丢 gyro 并回到观察态
7. 三路 timestamp 完全一致时，组装一条完整 IMU 历史

这里不再做半周期近邻匹配。当前协议要求同一帧 IMU 的 `gyro / accl / quat` envelope timestamp 完全一致。

## 同步模式

`LATEST_IMU`：

- 不发 `CameraSync::SyncCommand`
- 每张图像直接绑定当前最新完整 IMU
- 如果 `offset_us` 指向的最终 IMU 还没到，当前图像留在唯一 pending 槽位里等待

`RAW_PROBE`：

- 默认实机模式
- 通过 `CameraSync::SyncCommand` 触发一次节拍扰动
- 通过 `CameraSync::SyncEvent` 的 envelope timestamp 得到同步点 IMU 时间
- 锁定后按 IMU 周期递推后续图像对应的同步 IMU

## RAW_PROBE 状态机

状态只有三种：

- `OBSERVING`：持续观察图像周期和完整 IMU 周期
- `PROBE_SENT`：已发送一次 `CameraSync::SyncCommand`，等待 probe 图像和同 seq 回执
- `SYNCED`：已锁定，后续按 IMU 时间轴递推

流程：

1. `OBSERVING` 中观察到稳定图像周期 `T` 和稳定 IMU 周期 `t`
2. 计算 `N = round(T / t)`，同步 IMU 周期为 `N * t`
3. 发送 `CameraSync::SyncCommand{div=sync_probe_div, active_level, seq}`
4. MCU 在下一段反向半周期执行命令，并在同步点发布 `CameraSync::SyncEvent{seq}`
5. Host 看到期望 probe 图像 gap，同时收到同 seq result 后，用 result 的 envelope timestamp 找完整 IMU
6. 找到后发布该图像对应的同步 IMU，并进入 `SYNCED`
7. `SYNCED` 中每张正常图像用 `last_sync_imu_ts + sync_period` 找下一条同步 IMU

`sync_probe_div = 3` 时，期望 probe 图像 gap 为 `2T`。通用公式是 `T * (div + 1) / 2`。

## offset_us

`offset_us` 只在 IMU 时间域内使用：

1. 先确定同步点 IMU：`sync_imu_ts`
2. 再计算最终样本：`final_imu_ts = sync_imu_ts + offset_us`
3. 如果最终样本还没到，当前图像等待
4. 发布时 `ImuStamped.timestamp_us` 写图像时间戳，IMU 内容来自 `final_imu_ts`

等待期间不会重新选择同步点 IMU。

## 消费者接口

后续模块通过 `CameraFrameSync::Subscriber` 消费同步结果：

- `Wait(out, timeout_ms)` 是阻塞接口，`UINT32_MAX` 表示无限等待
- 成功返回时，`out.image` 持有共享图像槽位，`out.imu` 是 timestamp 完全相同的同步 IMU
- 如果某张图像已经被 IMU 时间轴越过，Subscriber 会释放这张图并继续等下一张
- 回调里不做 Detector/Tracker 工作，重计算应放在消费者线程里

## 配置

默认配置走实机 `RAW_PROBE`：

```yaml
constructor_args:
  camera: '@camera'
```

内录或已经同步的数据源使用 `LATEST_IMU`：

```yaml
constructor_args:
  camera: '@camera'
  runtime:
    expr: "CameraFrameSync<Info>::RuntimeParam{.mode = CameraFrameSync<Info>::SyncMode::LATEST_IMU, .offset_us = 0}"
```

Webots / 实机可显式配置同步 topic：

```yaml
runtime:
  expr: "CameraFrameSync<Info>::RuntimeParam{.mode = CameraFrameSync<Info>::SyncMode::RAW_PROBE, .offset_us = 0, .host_topic_domain_name = \"host\", .sync_command_topic_name = \"camera_sync_command\", .sync_result_topic_name = \"camera_sync_result\", .raw_imu_topic_prefix = \"gimbal\", .sync_probe_div = 3, .sync_active_level = 1}"
```

## 日志

模块只在状态边界打日志：

- 启动时打印图像、IMU、原始 topic 和模式
- 发送 `CameraSync::SyncCommand`
- 进入 `SYNCED`
- 从 `PROBE_SENT / SYNCED` 回到 `OBSERVING`
- ingress 溢出导致运行时状态重置

正常稳态不应持续出现 `-> OBSERVING`。
