# CameraFrameSync

`CameraFrameSync` 在单进程内完成两件事：

- 从 `CameraBase<FrameLayoutV>` 的普通 Topic 接收并持有 `SharedFrame`
- 把图像与 MCU 每个真实相机触发边沿对应的 IMU 时间戳配对，发布 `SyncedFrame`

模块不创建工作线程或信号量。图像、原始 IMU 和 `CameraSync` 事件回调通过同一把
状态锁串行推进；`SyncCommand`、`SyncedFrame` 的 Topic 发布以及
`CameraBase::SwitchProfile()` 都在锁外执行，允许同步订阅者重入模块接口。

## Topic

输入：

- 图像：`camera.ImageTopicNameView()`，载荷为仅在同步回调期间有效的
  `const SharedFrame*`
- 角速度：`<camera_name>_gyro`，类型为 `Eigen::Matrix<float, 3, 1>`，单位 rad/s
- 加速度：`<camera_name>_accl`，类型为 `Eigen::Matrix<float, 3, 1>`，单位 m/s^2
- 姿态：`<camera_name>_quat`（`raw_quat_topic_name` 非空时改用该名字），类型为
  `LibXR::Quaternion<float>`
- 同步事件：`sync_result_topic_name`（默认 `camera_sync_result`），类型为
  `CameraSync::SyncEvent`

`<camera_name>` 是相机的实例名（`camera.NameView()`）。

输出：

- 同步结果：`synced_frame_topic_name`，为空时为 `<图像 topic>_synced`（可用
  `SyncedFrameTopicName()` 查询）；载荷为仅在同步回调期间有效的 `const SyncedFrame*`
- 同步命令：`sync_command_topic_name`（默认 `camera_sync_command`），类型为
  `CameraSync::SyncCommand`

命令、事件和原始 IMU Topic 使用 `host_topic_domain_name`。默认 domain 是
`shared_memory`；产品配置可以显式改为 `host` 等 SharedTopic 转发 domain，进程内仿真可用
默认域 `libxr_def_domain`。图像和 `SyncedFrame` 是当前进程内的普通 Topic，不能跨进程
传递裸指针。

`SyncedFrame` 包含单调递增的 `sequence`、图像句柄 `image`（`GetImageFrame()` 取只读帧）
和 `imu`（`ImuStamped`，平移恒为 0）。

## 原始 IMU 坐标

`raw_imu_frame` 声明原始 IMU 的坐标约定：

- `BODY_X_RIGHT_Y_FORWARD_Z_UP`（默认）：原样使用。
- `X_FORWARD_Y_LEFT_Z_UP_TO_BODY`：把 x 前 / y 左 / z 上的数据转换到本体系
  （x 右 / y 前 / z 上），向量取 `(-y, x, z)`，四元数取 `(w, -y, x, z)`。

## 所有权

Topic 中的指针只借用到当前同步回调返回。需要在回调后继续使用图像时，订阅者必须在
回调内复制整个 `SyncedFrame` 或其中的 `SharedFrame`，再把副本移动到自己的稳定工作槽位；
不得保存裸指针，也不得用异步 `QueuedSubscriber` 保存该指针。

每份 `SharedFrame` 都共同持有 CameraBase 两槽对象池中的同一槽位。最后一份句柄析构后
槽位自动归还，不复制像素，也不按帧分配堆内存。CameraFrameSync 内部的 image、trigger
和 outbound FIFO 只是等待匹配及锁外发布的固定容量状态，不是另一套跨模块传输协议。

## 时间域

相机时间和 MCU/IMU 时间属于两个独立设备时钟域：

- `ImageFrame::timestamp_us` 保留相机采样时间
- 原始 IMU 使用 Topic envelope timestamp
- `FRAME_TRIGGER` 的 Topic envelope timestamp 是 MCU 产生真实 GPIO 边沿时的 IMU 时间
- `SyncedFrame::imu.timestamp_us` 在 `TRIGGER` 模式下等于匹配边沿的时间戳

模块不会比较相机时间和 MCU 时间的绝对值。相机时间只用于计算相邻已匹配图像的 gap；
下游排序使用 `SyncedFrame::imu.timestamp_us`。同一时钟 epoch 内的权威时间戳严格递增，
迟到输出不会进入下游。检测到 gyro 时间戳回退时，会清除旧匹配、原始 IMU 和输出时间基线：
TRIGGER 链路用新命令序号重新确认 STOP，忽略旧序号 ACK，再按新确认时间等待 settle。
中断事务已消耗的重试次数不清零，恢复命令共用剩余额度；超限记录错误并进入 FAILED。
尚未返回的相机切换不会重入，也不会越过新的 STOP/settle 边界发出 START。
新 epoch 可以发布比上一 epoch 小的时间戳，下游需要重建跟踪时间基线。
普通 STOP/START 只清除匹配基线，不清除同一 MCU 时钟内的输出顺序约束。

触发周期只由相机 profile 的 `trigger_period_us` 决定。

## TRIGGER 模式

`TRIGGER` 是实机默认模式。构造时即对当前 profile 发起一次同档重同步：

```text
WAIT_STOP_ACK
  -> SETTLING
  -> WAIT_START_ACK
  -> RUNNING
```

真实切档时，`SETTLING` 后增加一次状态：

```text
WAIT_STOP_ACK
  -> SETTLING
  -> SWITCHING_CAMERA
  -> WAIT_START_ACK
  -> RUNNING
```

流程如下：

1. Host 发送 `STOP_TRIGGER{seq, active_level, trigger_period_us=0}`。
2. MCU 停触发并回 ACK。Host 校验 operation、seq、active level、reserved 和
   `effective_period_us=0`。
3. 由后续 gyro 时间推进 `camera_settle_us`（默认 10 ms），起点为 STOP ACK 的时间戳。
4. 真实切档时在状态锁外调用 `CameraBase::SwitchProfile()`，并校验返回的档位与几何；
   同档重同步跳过此步。
5. Host 发送 `START_TRIGGER{seq, active_level, trigger_period_us}`。
6. START ACK 必须返回相同 period 且 `trigger_sequence=0`，随后直接进入 `RUNNING`。
7. START 后第一条真实边沿必须是 `trigger_sequence=1`；后续每个真实边沿严格加一、时间戳
   严格递增，event 的 `seq` 必须等于本次生效的 START seq。

这里没有周期观察、probe gap、divider 或 RESET 命令。`CameraFrameSyncMode::RAW_PROBE`
只是 `TRIGGER` 的源码兼容别名。

## 图像与边沿匹配

进入 `RUNNING` 后，每个合法 `FRAME_TRIGGER` 都进入固定容量（256）trigger FIFO。第一张
图像与本次 START 后队列中的第一条真实边沿配对。后续图像按以下规则选择目标边沿：

```text
camera_gap = current_camera_ts - previous_camera_ts
stride = round(camera_gap / active_profile.trigger_period_us)
target_trigger_sequence = previous_trigger_sequence + stride
```

残差必须不超过：

```text
max(1500 us, trigger_period_us / 4)
```

`stride` 的上限为 128。合法 stride 为 2 或更大时，模块跳过 trigger FIFO 中较旧的真实
边沿，直接消费目标 sequence；这表示相机少交付了图像，不表示 MCU 补发或猜测了边沿。
目标边沿尚未到达时图像继续等待。无法解释的 camera gap、重复/回退相机时间、边沿
sequence 不连续、边沿时间回退或 trigger FIFO 溢出都会清除本地匹配并对当前 profile
重新执行 STOP/settle/START，不调用 `SwitchProfile()`。

匹配到边沿后，模块在 IMU history（最多 1024 条）中寻找对应样本。`offset_us=0` 要求精确
命中该边沿时间；非零 offset 在 IMU 时间域中选择 `trigger_timestamp + offset_us` 附近
500 us 内最近的样本。若未来样本尚未到达则继续等待，找不到合法样本则重新同步。无论选用
了哪条 IMU 内容，发布的 `SyncedFrame::imu.timestamp_us` 始终保留原始 `FRAME_TRIGGER`
时间。`SetOffsetUs()` 可在运行时修改 offset。

## 固定 profile

相机通过 `Profiles()` 声明一到两个固定 profile（构造时检查 ID 唯一、周期非零、几何
合法）。每项包含 `ProfileId`、逐帧 geometry 和直接传给 MCU 的 `trigger_period_us`。

`RequestProfile()` 只负责接纳异步 STOP/settle/SwitchProfile/START 事务：

- 相机不支持的 profile 返回 `NOT_SUPPORT`
- 非 `RUNNING` 状态（包括控制阶段和 `FAILED`）返回 `STATE_ERR`
- `RUNNING` 中请求当前 profile 直接返回 `OK`，不发送命令，也不调用 `SwitchProfile()`
- 单 profile 相机仍在启动和异常恢复时执行 STOP/START，但永远不切相机
- `SwitchProfile()` 失败或返回不一致结果时进入 `FAILED`，不发送 START，不假设旧配置可用，
  也不自动回滚
- 命令 ACK 超时也会进入 `FAILED`，但不能据此断言 MCU 未执行命令或触发已经停止

非 `RUNNING` 控制阶段到达的图像立即释放。切档不等待两个图像槽全部回池；已经被下游
持有的旧 profile `SharedFrame` 可以和新 profile 图像同时存在。

## 逐帧 geometry

构造时模块复制相机的原生 `CameraCalibration`（`Calibration()` 返回）。每张图像都携带
不可变 `ImageFrame::geometry`，提交后像素和 geometry 均不得修改。CameraFrameSync 只验证
该快照能否由 `FrameLayoutV` 承载并落在原生标定范围内，不用当前 active profile 拒绝
其它合法 geometry。

因此 profile id 只存在于控制面。下游必须使用当前帧的 geometry，把像素坐标映射到
统一的原生相机或物理坐标；无需 geometry epoch、全链路 barrier 或下游 reset Topic。

## 原始 IMU 组装

`TRIGGER` 模式以 gyro 为主键组装 `gyro / accl / quat`：

1. 三路回调分别进入固定容量（1024）队列。
2. 丢弃早于当前 gyro 的 accl 和 quat。
3. 三路 timestamp 完全一致时生成一条完整 IMU history 样本。
4. 若 accl 或 quat 已晚于当前 gyro，只丢弃这条无法补齐的 gyro。
5. gyro timestamp 严格回退会清空原始 IMU 状态，并按上文建立新 epoch；重复时间戳不建立
   新 epoch，只清空 history 并重新同步。

队列溢出同样会清空原始 IMU 状态并重新同步。模块不会按历史周期合成或补齐 IMU。

## LATEST_IMU 模式

`LATEST_IMU` 用于数据源已经自行同步的兼容路径：

- 不发送 `CameraSync::SyncCommand`
- 不允许切换到非当前 profile（`RequestProfile()` 对当前 profile 返回 `OK`，其余返回
  `NOT_SUPPORT`）
- 只消费 quat 样本，以 quat envelope timestamp 作为输出权威时间，并将角速度和加速度置零
- 每张图像取最新 quat；若自上次输出后没有时间戳更新的 quat，该图像被丢弃，同一 quat
  timestamp 不会重复发布
- `offset_us` 只用于 `TRIGGER` 的边沿匹配，`LATEST_IMU` 不应用该偏移

该模式的内部控制状态为 `BYPASS`。

## ACK、重试与失败

operation 或 seq 不属于当前待处理命令的 ACK 会被忽略。operation 和 seq 已匹配、但
active level、reserved、effective period 或 START `trigger_sequence` 非法时进入
`FAILED`，不会继续等待另一条“正确”ACK。

STOP/START 初次发布后，以 raw gyro 时间每 100 ms 重发同一条完整命令，最多重试三次。
尚在 outbound FIFO、还没有真正发布的命令不开始计时，也不消耗重试次数。重试耗尽、
相机切档失败或 outbound FIFO 无法接纳控制项都会进入 `FAILED`。

## 监控

启动日志记录输入/输出 Topic、模式、active profile、触发周期和 settle 时间。

`OnMonitor()` 由 XRobot 监控循环调用，报告当前控制状态、profile、period，以及自上次调用
以来的 raw/assembled IMU、真实 trigger、输入/持有/丢弃图像、同步输出、重同步和溢出计数，
另报告当前持有的图像数和待匹配 trigger 数。控制状态只可能是：

- `BYPASS`
- `WAIT_STOP_ACK`
- `SETTLING`
- `SWITCHING_CAMERA`
- `WAIT_START_ACK`
- `RUNNING`
- `FAILED`

此外，`OnMonitor()` 输出 `pending_processing` 的累计次数、平均、最小和最大耗时，单位均为
微秒。该统计覆盖每次 `ProcessPendingLocked()` 调用，包含没有可匹配帧时的快速返回；它用于
观察同步处理开销，不等同于端到端相机同步延迟。

非法 geometry 只在首次出现时打印详细错误，避免持续刷日志。

## 依赖

- `QDU-Robomaster/CameraBase`：相机基类、帧布局、`SharedFrame` 与 `ImuStamped`。
- `QDU-Robomaster/CameraSync`：`SyncCommand` / `SyncEvent` 协议类型；MCU 侧（或仿真中
  同进程）的 CameraSync 实例负责实际触发和回执。
- `xrobot-org/DurationStatistics`：`pending_processing` 耗时统计。

无其他外部包（Eigen 来自 LibXR）。本模块为 header-only 模板。

## 构造接口

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class CameraFrameSync;

CameraFrameSync(
    Base& camera,
    RuntimeParam runtime = DefaultRuntime());
```

模板参数：

- `FrameLayoutV`：帧布局，必须与相机实例的模板参数相同。

依赖：

- `camera`：`CameraBase<FrameLayoutV>&`，即前面某个相机模块实例（HikCamera、WebotsCamera、
  CaptureFileCamera 等）。

配置 `runtime`（`RuntimeParam`，`DefaultRuntime()` 即全部默认值）：

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `mode` | `CameraFrameSyncMode::TRIGGER` | `TRIGGER` 或 `LATEST_IMU`。 |
| `offset_us` | `0` | 在 IMU 时间域中相对触发时间的取样偏移，单位 us；只用于 `TRIGGER`。 |
| `host_topic_domain_name` | `"shared_memory"` | 命令、事件和原始 IMU Topic 的 domain。 |
| `sync_command_topic_name` | `"camera_sync_command"` | 发给 CameraSync 的命令 Topic。 |
| `sync_result_topic_name` | `"camera_sync_result"` | CameraSync 回执和触发事件 Topic。 |
| `sync_active_level` | `1` | 触发有效电平，非零按 1 处理。 |
| `camera_settle_us` | `10000` | STOP ACK 后等待相机稳定的时间，单位 us。 |
| `raw_imu_frame` | `CameraFrameSyncRawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP` | 原始 IMU 坐标约定，见上文。 |
| `raw_quat_topic_name` | 空 | 非空时替代 `<camera_name>_quat`。 |
| `synced_frame_topic_name` | 空 | 为空时使用 `<图像 topic>_synced`。 |

相机名、图像 topic、domain 和两个同步 topic 名不能为空，相机 profile 不合法时构造
`REQUIRE` 失败。

公共方法：`SyncedFrameTopicName()`、`RawTopicDomainName()`、`Calibration()`、`Profiles()`、
`ActiveProfile()`、`GetSyncMode()`、`RequestProfile()`、`SetOffsetUs()`、
`FlushPendingFrames()`（立即处理待匹配数据后清空图像和 trigger 队列）、`OnMonitor()`。

## 使用

```sh
xrobot module add QDU-Robomaster/CameraFrameSync
xrobot setup
xrobot instance add QDU-Robomaster/CameraFrameSync
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把 `camera` 填为前面相机模块实例的 id。帧布局用 constexpr 定义，必须与相机输出一致：

```yaml
constexpr_includes:
  - CameraBase.hpp
constexprs:
  FrameLayout:
    type: CameraTypes::FrameLayout
    value: '{.width = 640, .height = 480, .step = 1920, .encoding = CameraTypes::Encoding::BGR8}'
modules:
  - module: QDU-Robomaster/CameraFrameSync
    id: cameraframesync_0
    template_args:
      - ProjectConstexpr::FrameLayout
    args:
      - camera: webotscamera_0
      - runtime: CameraFrameSync<ProjectConstexpr::FrameLayout>::DefaultRuntime()
```

`webotscamera_0` 是 WebotsCamera（或 HikCamera、CaptureFileCamera）实例的 id，必须在
`modules:` 中列在本实例之前，并使用同一个 `template_args`。本模块不直接使用 BSP 对象，
不需要额外的 `XR_REGISTER`。ArmorDetector、ArmorTracker 等下游模块用 `sync: cameraframesync_0`
引用本实例。

`runtime` 也可以写成 YAML map，字符串字段写成 C++ 字符串字面量，枚举写成 C++ 表达式。
仿真中同进程 CameraSync 的 TRIGGER 配置示例：

```yaml
runtime:
  mode: 'CameraFrameSyncMode::TRIGGER'
  offset_us: 0
  host_topic_domain_name: '"libxr_def_domain"'
  sync_command_topic_name: '"camera_sync_command"'
  sync_result_topic_name: '"camera_sync_result"'
  sync_active_level: 1
  camera_settle_us: 10000
  raw_imu_frame: 'CameraFrameSyncRawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP'
  raw_quat_topic_name: '{}'
  synced_frame_topic_name: '{}'
```

数据源已经自行同步时把 `mode` 改为 `'CameraFrameSyncMode::LATEST_IMU'`。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/CameraFrameSync`
（在 BSP 中）打印当前的构造函数。

## 测试

在打开 `BUILD_TESTING` 的 BSP 构建中，本模块加入 `camera_frame_sync_sequence_test`、
`camera_frame_sync_frame_queue_test`，以及按用例拆分的 `camera_frame_sync_profile_switch_*`
（启动首边沿、stride、gap 重同步、切档顺序、ACK 过滤与畸形 ACK、重试与重试耗尽、各状态下
的时间戳 epoch、切档失败、所有权、时钟域、LATEST_IMU、运行参数等），用 `ctest` 运行。

其中 timestamp-epoch 用例在相机切档期间采集一条样本，并在当前 gyro 发布返回后再投递
（非互斥 Topic 不允许向自身递归发布），验证 STOP、settle 和 START 恢复不会重入已完成的
相机切换；它不改变运行时 Topic 或同步契约。
