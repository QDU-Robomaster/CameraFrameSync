# CameraFrameSync

相机帧同步：按触发边沿给每张图配上 IMU，发布同步帧 / Camera frame sync that pairs every image with the IMU at its trigger edge and publishes synced frames

## 1. 模块作用 / Purpose

相机由 MCU（CameraSync 模块）按固定周期外触发，MCU 为每个触发边沿发一条带 IMU 时间戳的事件。CameraFrameSync 在主机侧把每张图对应到它的触发边沿，取曝光时刻的 IMU，组成 `SyncedFrame{sequence, image, imu}` 发布到 `<相机名>_synced`，检测器从这里开始处理。

The camera is triggered by the MCU (the CameraSync Module) at a fixed period, and the MCU sends one event with an IMU timestamp for every trigger edge. On the host, CameraFrameSync matches each image to its trigger edge, takes the IMU at the exposure time and publishes `SyncedFrame{sequence, image, imu}` on `<camera>_synced`, where the detector starts.

## 2. 结构 / Structure

所有回调只把输入放进一条按到达顺序处理的队列；配对、链路控制和发布都在模块自己的工作线程上进行。工作线程使用三个部件，各自可以单独测试：

Every callback only puts its input into one queue handled in arrival order; matching, link control and publishing all run on the Module's own worker thread, which uses three parts that can be tested on their own:

| 部件 / Part | 职责 / Role |
| --- | --- |
| `ImuHistory` | 角速度、加速度、姿态三路各自的时间环形缓冲，按任意时刻插值 / Time rings of the three IMU streams, interpolated at any time |
| `TriggerLink` | 与 CameraSync 的 STOP → 稳定 → 切档 → START 链路，等 ACK 时重发 / The STOP → settle → switch → START link to CameraSync, resending while waiting for ACKs |
| `FrameMatcher` | 按相机帧计数把图像对应到边沿，发现矛盾即要求重新同步 / Matches images to edges by frame counter and requests a resync on a contradiction |

## 3. 配对 / Matching

启动时 CameraFrameSync 让 MCU 停止触发，等 10 ms 让在途图像到齐，再以 `trigger_period_us` 重新开始。START 生效后的第一张图的帧计数记为 c0，对应边沿 1；计数为 c 的图对应边沿 c − c0 + 1。图像的 IMU 取在边沿时刻加 `offset_us` 处（触发延迟加半个曝光），由 `ImuHistory` 插值得到；边沿或 IMU 尚未到达时图像最多等待 100 ms。

At start-up CameraFrameSync stops the MCU trigger, waits 10 ms for images in flight and restarts it at `trigger_period_us`. The first image after START has frame counter c0 and belongs to edge 1; an image with counter c belongs to edge c − c0 + 1. Its IMU is interpolated by `ImuHistory` at the edge time plus `offset_us` (trigger delay plus half the exposure); an image waits at most 100 ms for its edge or IMU.

主机侧丢掉的图只让帧计数跳号，配对仍然准确；丢失的边沿消息只让对应的那张图被丢弃。以下情况说明计数与边沿已经错位，模块重新执行 STOP/START：

A frame dropped on the host only skips a counter value and matching stays exact; a lost edge message only drops its own image. The following show that counters and edges no longer line up, and the Module runs STOP/START again:

- 两张图的设备时间间隔与对应两个边沿的间隔相差超过四分之一周期（至少 1.5 ms），即相机漏了触发；
- 帧计数没有递增，即相机重新开始计数；
- 图像领先最新边沿 3 个以上；
- 收到不属于当前 START 的边沿，例如 MCU 重启后的默认触发。

- two images whose device-time interval differs from that of their edges by more than a quarter period (at least 1.5 ms): the camera missed a trigger;
- a frame counter that did not advance: the camera restarted its counter;
- an image more than 3 edges ahead of the newest edge;
- an edge that does not belong to the current START, such as the MCU's default trigger after a reboot.

触发周期须大于曝光时间与读出时间的较大者，否则相机会跳过触发，表现为反复重新同步。

The trigger period must exceed the camera's minimum frame period (the larger of readout and exposure), otherwise the camera skips triggers and the Module keeps resyncing.

`RequestView(view)` 可在任意线程调用：停触发、等稳定、调用相机的 `SwitchView`、再启动触发，之后的帧带新视角的几何。

`RequestView(view)` may be called from any thread: it stops the trigger, waits, calls the camera's `SwitchView` and restarts the trigger; later frames carry the new view's geometry.

## 4. LATEST_IMU

`SyncMode::LATEST_IMU` 只用于没有触发线的台架调试：相机自由运行，每张图到达时取 `ImuHistory` 中最新的 IMU。相机时钟与 MCU 时钟不同源，同步误差约为一个 IMU 周期加上传输延迟，结果不能用来评估跟踪。回放不经过本模块，由 CaptureFileCamera 直接发布同步帧。

`SyncMode::LATEST_IMU` is only for bench debugging without a trigger line: the camera free-runs and each image gets the newest IMU in `ImuHistory`. The camera and MCU clocks are independent, so the sync error is about one IMU period plus the transport delay, and results are not fit for judging tracking. Replay does not use this Module; CaptureFileCamera publishes synced frames itself.

## 5. Topic

| Topic | 载荷 / Payload | 方向 / Direction |
| --- | --- | --- |
| `<相机名>_image` | `const SharedFrame*` | 订阅 / Subscribed |
| `gyro_topic`、`accl_topic` | `Eigen::Matrix<float, 3, 1>` | 订阅，MCU 发布的名字 / Subscribed, MCU names |
| `quat_topic` | `LibXR::Quaternion<float>` | 订阅 / Subscribed |
| `camera_sync_result` | `CameraSyncDetail::SyncEvent` | 订阅（TRIGGER）/ Subscribed (TRIGGER) |
| `camera_sync_command` | `CameraSyncDetail::SyncCommand` | 发布（TRIGGER）/ Published (TRIGGER) |
| `<相机名>_synced` | `const AutoAim::SyncedFrame*` | 发布 / Published |

IMU 与 CameraSync 的 Topic 位于 `mcu_domain`（车上为 SharedTopic 使用的 `host`，Webots 中为默认 domain）。相机图像 Topic 须在本模块之前创建，否则启动即致命退出；IMU 与 CameraSync 的 Topic 不存在时由本模块按类型创建，因为 SharedTopic 只转发已存在的带类型 Topic，车上配置中本模块排在 SharedTopic 之前。

The IMU and CameraSync Topics live in `mcu_domain` (`host`, used by SharedTopic, on the robot; the default domain in Webots). The camera image Topic is created before this Module, otherwise start-up is fatal; absent IMU and CameraSync Topics are created here with their types, because SharedTopic only forwards existing typed Topics, and the robot configuration lists this Module before SharedTopic.

`imu_axes` 说明 MCU IMU 的坐标轴：`ImuAxes::BODY` 表示已是机体系（x 右、y 前、z 上）；`ImuAxes::X_FORWARD_Y_LEFT_Z_UP` 表示 x 前、y 左、z 上，进入同步前转成机体系，向量取 `(-y, x, z)`，四元数取 `(w, -y, x, z)`（哨兵 C 板的安装方向）。

`imu_axes` gives the axes of the MCU IMU: `ImuAxes::BODY` means it is already in the body frame (x right, y forward, z up); `ImuAxes::X_FORWARD_Y_LEFT_Z_UP` means x forward, y left, z up and is converted into the body frame before syncing, vectors as `(-y, x, z)` and quaternions as `(w, -y, x, z)` (the sentry C board's mounting).

## 6. 配置示例 / Configuration Example

```yaml
modules:
  - module: QDU-Robomaster/CameraFrameSync
    id: frame_sync
    args:
      - camera: camera
      - settings:
          mode: SyncMode::TRIGGER
          trigger_period_us: 10000
          offset_us: AutoAimRunConfig::HikSyncOffsetUs
          mcu_domain: "host"
          gyro_topic: "gimbal_gyro"
          accl_topic: "gimbal_accl"
          quat_topic: "gimbal_quat"
          imu_axes: ImuAxes::BODY
```

## 7. 测试 / Tests

- `tests/parts_test.cpp`：`ImuHistory` 的插值、四元数半球、尚未到达与过旧；`TriggerLink` 的重发、ACK、稳定、切档与序号循环；`FrameMatcher` 的计数对应、主机丢帧、丢失边沿与各种矛盾。
- `tests/camera_frame_sync_test.cpp`：假 MCU（1 kHz IMU）驱动真实的 CameraSync，CameraSync 的触发电平驱动假相机出图。检查每帧 IMU 都取在自己的边沿加偏移处、相机漏一次触发后只重新同步一次并恢复、切档后的几何，LATEST_IMU，以及 x 前、y 左、z 上的 IMU 转成机体系。

- `tests/parts_test.cpp`: `ImuHistory` interpolation, quaternion hemispheres, not-yet and too-old lookups; `TriggerLink` resends, ACKs, settling, view switch and sequence wrap; `FrameMatcher` counter matching, host drops, lost edges and the contradictions.
- `tests/camera_frame_sync_test.cpp`: a fake MCU (1 kHz IMU) drives the real CameraSync, whose trigger level drives a fake camera. It checks that every frame's IMU is taken at its own edge plus offset, that one missed trigger causes exactly one resync and recovery, the geometry after a view switch, LATEST_IMU, and the conversion of x-forward, y-left, z-up IMU data into the body frame.

## 8. 依赖 / Dependencies

CameraBase、AutoAimTypes、CameraSync（协议头文件）、LibXR。

CameraBase, AutoAimTypes, CameraSync (protocol header), LibXR.
