# CameraFrameSync

`CameraFrameSync` 负责两件事：

1. 承接 `CameraBase<Info>` 的图像 lease，并把图像发布到 `LinuxSharedTopic`
2. 订阅原始 `gyro / accl / quat / image_event`，在模块内部完成图像与 IMU 对齐，再发布同步后的 `imu`

`CameraFrameSync<Info>` 也是当前这套强类型图像 lease 接口的具体实现者。

## 输入与输出

输入：

- 图像写入接口：
  - `CameraBase<Info>::RegisterImageSink(...)`
- 原始小话题：
  - `<camera_name>_gyro`
  - `<camera_name>_accl`
  - `<camera_name>_quat`
  - `<camera_name>_image_event`
- 同步配置话题：
  - `<camera_name>_sync_config`

输出：

- 图像共享话题：
  - `camera.ImageTopicName()`
- 同步后 IMU 话题：
  - `camera.ImuTopicName()`

## 同步流程

1. `gyro / accl / quat` 回调只负责入队
2. `image_event` 回调作为同步触发点，串行排空各 ingress 队列
3. 以 `gyro` 为主时间轴，向后找最近的 `accl / quat`，组装出 IMU 历史
4. 对每个 `image_event`，根据 `offset_us` 在 IMU 历史里找目标样本
5. 若节拍、序号、图像事件间隔超时、IMU 到达超时或队列状态异常，则进入 `RECOVERING`

这里的“原始 IMU 对齐”发生在模块内部；下游 `Subscriber` 等的是已经发布出来的同步后 `imu`。

## `Subscriber` 语义

`Subscriber::Wait()` 仍然是严格时间戳匹配：

- 等到一帧图像
- 再等待一条 `timestamp_us` 完全相同的同步后 `imu`
- 只有两者对上，才返回 `SyncedFrame`

也就是说，`Subscriber` 不负责“原始 IMU 重同步”；那一步已经在 `CameraFrameSync` 内部完成。

## 默认策略

- 图像共享话题：
  - `slot_num = 8`
  - `queue_num = 2`
- ingress 队列长度：
  - `imu = 1024`
  - `image_event = 256`
- 历史缓存长度：
  - `pending/history = 2048`
- 图像发布路径是非阻塞的：
  - 如果拿不到新可写槽位，就继续复用当前槽位，并丢掉这次新图像

## 模板参数与依赖

- 模板参数：
  - `Info`：编译期 `CameraTypes::CameraInfo`
- 依赖：
  - `qdu-future/CameraBase`
