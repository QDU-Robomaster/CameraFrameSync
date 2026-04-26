# CameraFrameSync

`CameraFrameSync` 是当前链路里负责共享图像传输和图像/IMU 同步订阅的桥接模块。

`CameraFrameSync<Info>` 也是 `CameraBase<Info>` 当前这套强类型图像 lease
接口的具体实现者。

## 运行时职责

1. 从 `LinuxSharedTopic` 租一块可写 `ImageFrame` 槽位
2. 把自己注册给 `CameraBase<Info>`，作为生产者侧图像 lease sink
3. 当相机调用 `CommitImage()` 时，发布当前共享槽位
4. 再租下一块可写槽位给生产者继续写
5. 让下游订阅者等待图像和 IMU 时间戳对齐后再取出同步帧

## 输出话题

- 图像共享话题：
  - 与 `camera.ImageTopicName()` 同名
- IMU 异步话题：
  - 与 `camera.ImuTopicName()` 同名

## 同步载荷

- `SyncedFrame.image`
  - `CameraBase<Info>::ImageFrame` 的共享 lease
- `SyncedFrame.imu`
  - 拷贝后的 `CameraBase<Info>::ImuStamped`

## 默认策略

- `slot_num = 8`
- `queue_num = 2`
- 生产者路径是非阻塞的
- 如果没有多余可写槽位，就复用当前可写槽位，并丢掉这次新帧

## 模板参数

- `Info`：编译期 `CameraTypes::CameraInfo`

## 依赖

- `qdu-future/CameraBase`
