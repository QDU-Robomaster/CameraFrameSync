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
- 姿态：`<camera_name>_quat`，类型为 `LibXR::Quaternion<float>`
- 同步事件：`camera_sync_result`，类型为 `CameraSync::SyncEvent`

输出：

- 同步结果：`CameraFrameSync::SyncedFrameTopicName()`，载荷为仅在同步回调期间有效的
  `const SyncedFrame*`
- 同步命令：`camera_sync_command`，类型为 `CameraSync::SyncCommand`

命令、事件和原始 IMU Topic 使用 `host_topic_domain_name`。默认 domain 是
`shared_memory`；产品配置可以显式改为 `host` 等 SharedTopic 转发 domain。图像和
`SyncedFrame` 是当前进程内的普通 Topic，不能跨进程传递裸指针。

## 所有权

Topic 中的指针只借用到当前同步回调返回。需要在回调后继续使用图像时，订阅者必须在
回调内复制整个阶段包装或其中的 `SharedFrame`，再把副本移动到自己的稳定工作槽位；
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
迟到输出不会进入下游。检测到 gyro 时间戳回退时，会清除旧匹配和输出时间基线：
TRIGGER 链路用新命令序号重新确认 STOP，忽略旧序号 ACK，再按新确认时间等待 settle。
中断事务已消耗的重试次数不清零，恢复命令共用剩余额度；超限记录错误并进入 FAILED。
尚未返回的相机切换不会重入，也不会越过新的 STOP/settle 边界发出 START。
新 epoch 可以发布比上一 epoch 小的时间戳，下游需要重建跟踪时间基线。
普通 STOP/START 只清除匹配基线，不清除同一 MCU 时钟内的输出顺序约束。

旧 YAML 的 `sync_probe_div`、`target_trigger_hz` 由兼容构造入口接收，不恢复旧探针策略。
`TRIGGER` 下要求旧频率与相机初始 profile 周期一致；`LATEST_IMU` 只校验元数据有效性，
不使用它控制触发。无效旧参数记录错误并拒绝构造。

## TRIGGER 模式

`TRIGGER` 是实机默认模式。构造时即对当前 profile 发起一次同档重同步：

```text
WAIT_STOP_ACK
  -> SETTLING
  -> WAIT_START_ACK
  -> RUNNING
```

真实切档时，`SETTLING` 后增加一次可选状态：

```text
WAIT_STOP_ACK
  -> SETTLING
  -> SWITCHING_CAMERA
  -> WAIT_START_ACK
  -> RUNNING
```

流程如下：

1. Host 发送 `STOP_TRIGGER{seq, active_level, trigger_period_us=0}`。
2. MCU 在下一条 IMU 消息处停触发并回 ACK。Host 校验 operation、seq、active level、
   reserved 和 `effective_period_us=0`。
3. 由后续 gyro 时间推进默认 10 ms 的 `camera_settle_us`。
4. 真实切档时在状态锁外调用 `CameraBase::SwitchProfile()`；同档重同步跳过此步。
5. Host 发送 `START_TRIGGER{seq, active_level, trigger_period_us}`。
6. START ACK 必须返回相同 period 且 `trigger_sequence=0`，随后直接进入 `RUNNING`。
7. START 后第一条真实边沿必须是 `trigger_sequence=1`；后续每个真实边沿严格加一，
   event 的 `seq` 必须等于本次生效的 START seq。

这里没有周期观察、probe gap、divider 或 RESET 命令。`RAW_PROBE` 仅保留为
`TRIGGER` 的源码兼容枚举别名，不代表仍存在 probe 协议。

## 图像与边沿匹配

进入 `RUNNING` 后，每个合法 `FRAME_TRIGGER` 都进入固定容量 trigger FIFO。第一张图像
与本次 START 后队列中的第一条真实边沿配对。后续图像按以下规则选择目标边沿：

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

匹配到边沿后，模块在 IMU history 中寻找对应样本。`offset_us=0` 要求精确命中该边沿
时间；非零 offset 在 IMU 时间域中选择 `trigger_timestamp + offset_us` 附近 500 us 内的
样本。若未来样本尚未到达则继续等待，找不到合法样本则重新同步。无论选用了哪条 IMU
内容，发布的 `SyncedFrame::imu.timestamp_us` 始终保留原始 `FRAME_TRIGGER` 时间。

## 固定 profile

相机通过 `Profiles()` 声明一到两个固定 profile。每项包含 `ProfileId`、逐帧 geometry
和直接传给 MCU 的 `trigger_period_us`。

`RequestProfile()` 只负责接纳异步 STOP/settle/SwitchProfile/START 事务：

- `RUNNING` 中请求当前 profile 直接返回 `OK`，不发送命令，也不调用 `SwitchProfile()`
- 单 profile 相机仍在启动和异常恢复时执行 STOP/START，但永远不切相机
- `SwitchProfile()` 失败时不发送 START，不假设旧配置可用，也不自动回滚
- 相机切换失败后进入 `FAILED`，不发送 START；后续有效 profile 请求都返回 `STATE_ERR`
- 命令 ACK 超时也会进入 `FAILED`，但不能据此断言 MCU 未执行命令或触发已经停止

非 `RUNNING` 控制阶段到达的图像立即释放。切档不等待八个图像槽全部回池；已经被下游
持有的旧 profile `SharedFrame` 可以和新 profile 图像同时存在。

## 逐帧 geometry

构造时模块复制相机的原生 `CameraCalibration`。每张图像都携带不可变
`ImageFrame::geometry`，提交后像素和 geometry 均不得修改。CameraFrameSync 只验证该
快照能否由 `FrameLayoutV` 承载并落在原生标定范围内，不用当前 active profile 拒绝
其它合法 geometry。

因此 profile id 只存在于控制面。下游必须使用当前帧的 geometry，把像素坐标映射到
统一的原生相机或物理坐标；无需 geometry epoch、全链路 barrier 或下游 reset Topic。

## 原始 IMU 组装

`TRIGGER` 模式以 gyro 为主键组装 `gyro / accl / quat`：

1. 三路回调分别进入固定容量队列。
2. 丢弃早于当前 gyro 的 accl 和 quat。
3. 三路 timestamp 完全一致时生成一条完整 IMU history 样本。
4. 若 accl 或 quat 已晚于当前 gyro，只丢弃这条无法补齐的 gyro。
5. gyro timestamp 严格回退会清空原始 IMU history，并触发重同步；重复时间戳不建立新 epoch。

队列溢出同样会清空原始 IMU 状态并重新同步。模块不会按历史周期合成或补齐 IMU。

## LATEST_IMU 模式

`LATEST_IMU` 用于数据源已经自行同步的兼容路径：

- 不发送 `CameraSync::SyncCommand`
- 不允许切换到非当前 profile
- 当前实现只消费 quat 样本，以 quat envelope timestamp 作为输出权威时间，并将角速度和
  加速度置零
- 每张图像需要一个时间戳继续向前的新 quat；同一 quat timestamp 不会重复发布
- `offset_us` 只用于 `TRIGGER` 的边沿匹配，`LATEST_IMU` 不应用该偏移

该模式的内部控制状态为 `BYPASS`。

## ACK、重试与失败

operation 或 seq 不属于当前待处理命令的 ACK 会被忽略。operation 和 seq 已匹配、但
active level、reserved、effective period 或 START `trigger_sequence` 非法时进入
`FAILED`，不会继续等待另一条“正确”ACK。

STOP/START 初次发布后，以 raw gyro 时间每 100 ms 重发同一条完整命令，最多重试三次。
尚在 outbound FIFO、还没有真正发布的命令不开始计时，也不消耗重试次数。重试耗尽、
相机切档失败或 outbound FIFO 无法接纳控制项都会进入 `FAILED`。

## 配置

默认实机配置：

```yaml
template_args:
  Layout: {constexpr: MainFrameLayout}
constructor_args:
  camera: '@camera'
  runtime:
    mode: {expr: "CameraFrameSync<ProjectConstexpr::MainFrameLayout>::SyncMode::TRIGGER"}
    offset_us: 0
    host_topic_domain_name: shared_memory
    sync_command_topic_name: camera_sync_command
    sync_result_topic_name: camera_sync_result
    sync_active_level: 1
    camera_settle_us: 10000
    synced_frame_topic_name: camera_synced
```

已经自行同步的数据源：

```yaml
template_args:
  Layout: {constexpr: MainFrameLayout}
constructor_args:
  camera: '@camera'
  runtime:
    mode: {expr: "CameraFrameSync<ProjectConstexpr::MainFrameLayout>::SyncMode::LATEST_IMU"}
    host_topic_domain_name: shared_memory
    synced_frame_topic_name: camera_synced
```

## 监控

启动日志记录输入/输出 Topic、模式、active profile、触发周期和 settle 时间。周期 monitor
报告当前控制状态、profile、period、raw/assembled IMU、真实 trigger、输入/持有/丢弃图像、
同步输出、重同步和溢出计数。控制状态只可能是：

- `BYPASS`
- `WAIT_STOP_ACK`
- `SETTLING`
- `SWITCHING_CAMERA`
- `WAIT_START_ACK`
- `RUNNING`
- `FAILED`

非法 geometry 只在首次出现时打印详细错误，避免持续刷日志。

此外，`OnMonitor()` 输出 `pending_processing` 的累计次数、平均、最小和最大耗时，
单位均为微秒。该统计覆盖每次 `ProcessPendingLocked()` 调用，包含没有可匹配帧时的
快速返回；它用于观察同步处理开销，不等同于端到端相机同步延迟。
