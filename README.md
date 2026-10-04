# CameraFrameSync

相机帧与 MCU 触发时间戳同步模块：持有相机帧并与 IMU 触发时间配对，发布 SyncedFrame / Camera frame and MCU trigger timestamp synchronization Module that holds camera frames, pairs them with IMU trigger times and publishes SyncedFrame

## 1. 模块作用 / Purpose

CameraFrameSync 在单进程内完成两件事：

- 从 `CameraBase<FrameLayoutV>` 的 Topic 接收并持有 `SharedFrame`。
- 把图像与 MCU 每个真实相机触发边沿对应的 IMU 时间戳配对，发布 `SyncedFrame`。

全部处理在各 Topic 回调所在的线程内完成，模块自身没有工作线程与信号量。图像、原始 IMU 和 `CameraSync` 事件回调通过同一把状态锁串行推进。`SyncCommand`、`SyncedFrame` 的 Topic 发布以及 `CameraBase::SwitchProfile()` 在锁外执行，同步订阅者可以重入模块接口。

`SyncedFrame` 包含单调递增的 `sequence`、图像句柄 `image`（`GetImageFrame()` 取只读帧）和 `imu`（`ImuStamped`，平移恒为 0）。

CameraFrameSync completes two tasks inside one process:

- It receives and holds `SharedFrame` handles from the Topic of `CameraBase<FrameLayoutV>`.
- It pairs each image with the IMU timestamp of the corresponding real MCU camera trigger edge and publishes a `SyncedFrame`.

All processing runs on the threads of the Topic callbacks, and the Module owns no worker thread or semaphore. The image, raw IMU and `CameraSync` event callbacks advance through one shared state lock. Publishing of `SyncCommand` and `SyncedFrame` Topics and `CameraBase::SwitchProfile()` run outside the lock, so synchronous subscribers may re-enter the Module interface.

A `SyncedFrame` holds a monotonically increasing `sequence`, the image handle `image` (`GetImageFrame()` returns the read-only frame) and `imu` (`ImuStamped`, translation always 0).

## 2. 时间域与原始 IMU 坐标 / Time Domain and Raw IMU Frame

相机时间与 MCU / IMU 时间属于两个独立的设备时钟域：

- `ImageFrame::timestamp_us` 保存相机采样时间。
- 原始 IMU 使用 Topic 消息时间戳（envelope timestamp）。
- `FRAME_TRIGGER` 的 Topic 消息时间戳是 MCU 产生真实 GPIO 边沿时的 IMU 时间。
- `TRIGGER` 模式下，`SyncedFrame::imu.timestamp_us` 等于匹配边沿的时间戳。

相机时间只用于计算相邻已匹配图像的间隔（gap），下游排序使用 `SyncedFrame::imu.timestamp_us`。同一时钟纪元（epoch）内的权威时间戳严格递增，迟到输出被丢弃。gyro 时间戳回退时开始新的纪元：旧匹配、原始 IMU 和输出时间基线被清除，`TRIGGER` 链路以新的命令序号重新确认 STOP，忽略旧序号的 ACK，再从新的确认时间起等待稳定时间（settle）。被中断事务已消耗的重试次数保留，恢复命令共用剩余额度，超限时记录错误并进入 `FAILED`。尚未返回的相机切换在新的 STOP / 稳定等待边界之后才允许发出 START。新纪元发布的时间戳可以小于上一纪元，下游据此重建跟踪时间基线。普通 STOP / START 只清除匹配基线，同一 MCU 时钟内的输出顺序约束继续保留。

触发周期由相机 profile 的 `trigger_period_us` 决定。

`raw_imu_frame` 声明原始 IMU 的坐标约定：

- `BODY_X_RIGHT_Y_FORWARD_Z_UP`（默认）：数据原样使用。
- `X_FORWARD_Y_LEFT_Z_UP_TO_BODY`：把 x 前 / y 左 / z 上的数据转换到本体系（x 右 / y 前 / z 上），向量取 `(-y, x, z)`，四元数取 `(w, -y, x, z)`。

The camera time and the MCU / IMU time belong to two independent device clock domains:

- `ImageFrame::timestamp_us` holds the camera sampling time.
- The raw IMU uses the Topic envelope timestamp.
- The Topic envelope timestamp of `FRAME_TRIGGER` is the IMU time at which the MCU produced the real GPIO edge.
- In `TRIGGER` mode, `SyncedFrame::imu.timestamp_us` equals the timestamp of the matched edge.

The camera time is used only to compute the gap between adjacent matched images, and downstream ordering uses `SyncedFrame::imu.timestamp_us`. Authoritative timestamps increase strictly within one clock epoch, and late outputs are discarded. A gyro timestamp regression starts a new epoch: the old matching, raw IMU and output time baselines are cleared, the `TRIGGER` link reconfirms STOP with a new command sequence number, ignores ACKs of the old sequence number, and waits for settle from the newly confirmed time. The retries already consumed by the interrupted transaction are kept, and the recovery commands share the remaining budget; when it is exhausted an error is logged and the state becomes `FAILED`. A camera switch that has not returned yet allows START only after the new STOP / settle boundary. A new epoch may publish timestamps smaller than those of the previous epoch, and downstream Modules rebuild their tracking time baseline accordingly. An ordinary STOP / START clears only the matching baseline, and the output-order constraint within the same MCU clock remains.

The trigger period is given by `trigger_period_us` of the camera profile.

`raw_imu_frame` declares the coordinate convention of the raw IMU:

- `BODY_X_RIGHT_Y_FORWARD_Z_UP` (default): data is used as is.
- `X_FORWARD_Y_LEFT_Z_UP_TO_BODY`: converts x-forward / y-left / z-up data into the body frame (x right / y forward / z up); vectors become `(-y, x, z)` and quaternions `(w, -y, x, z)`.

## 3. TRIGGER 模式 / TRIGGER Mode

`TRIGGER` 是实机默认模式。构造时即对当前 profile 发起一次同档重同步：

```text
WAIT_STOP_ACK
  -> SETTLING
  -> WAIT_START_ACK
  -> RUNNING
```

真实切档时，`SETTLING` 之后增加一个状态：

```text
WAIT_STOP_ACK
  -> SETTLING
  -> SWITCHING_CAMERA
  -> WAIT_START_ACK
  -> RUNNING
```

流程：

1. Host 发送 `STOP_TRIGGER{seq, active_level, trigger_period_us=0}`。
2. MCU 停止触发并回 ACK。Host 校验 operation、seq、active level、reserved 与 `effective_period_us=0`。
3. 由后续 gyro 时间推进 `camera_settle_us`（默认 10 ms），起点为 STOP ACK 的时间戳。
4. 真实切档时在状态锁外调用 `CameraBase::SwitchProfile()`，并校验返回的档位与几何；同档重同步跳过此步。
5. Host 发送 `START_TRIGGER{seq, active_level, trigger_period_us}`。
6. START ACK 返回相同的 period 且 `trigger_sequence=0`，随后进入 `RUNNING`。
7. START 之后第一条真实边沿的 `trigger_sequence` 为 1；后续每个真实边沿加一、时间戳严格递增，event 的 `seq` 等于本次生效的 START seq。

流程仅含 STOP / 稳定等待 / START 三步，由 `STOP_TRIGGER` 与 `START_TRIGGER` 两种命令实现。`CameraFrameSyncMode::RAW_PROBE` 是 `TRIGGER` 的源码别名。

图像与边沿匹配：进入 `RUNNING` 后，每个合法 `FRAME_TRIGGER` 进入容量为 256 的触发边沿 FIFO。第一张图像与本次 START 之后队列中的第一条真实边沿配对。后续图像按下式选择目标边沿：

```text
camera_gap = current_camera_ts - previous_camera_ts
stride = round(camera_gap / active_profile.trigger_period_us)
target_trigger_sequence = previous_trigger_sequence + stride
```

残差上限为：

```text
max(1500 us, trigger_period_us / 4)
```

`stride` 上限为 128。合法 `stride` 为 2 或更大时，模块跳过触发边沿 FIFO 中较旧的真实边沿，直接消费目标 sequence，对应相机少交付了图像的情形。目标边沿尚未到达时图像继续等待。出现以下情形时，模块清除本地匹配并对当前 profile 重新执行 STOP / 稳定等待 / START，期间保持当前相机 profile：无法解释的 camera gap、重复或回退的相机时间、边沿 sequence 不连续、边沿时间回退、触发边沿 FIFO 溢出。

匹配到边沿后，模块在最多 1024 条的 IMU history 中寻找对应样本。`offset_us=0` 要求精确命中该边沿时间；非零 offset 在 IMU 时间域中选择 `trigger_timestamp + offset_us` 附近 500 us 内最近的样本。未来样本尚未到达时继续等待，找不到合法样本时重新同步。无论选用哪条 IMU 内容，发布的 `SyncedFrame::imu.timestamp_us` 保留原始 `FRAME_TRIGGER` 时间。`SetOffsetUs()` 可在运行时修改 offset。

原始 IMU 组装：`TRIGGER` 模式以 gyro 为主键组装 `gyro / accl / quat`：

1. 三路回调分别进入容量为 1024 的队列。
2. 早于当前 gyro 的 accl 和 quat 被丢弃。
3. 三路 timestamp 一致时生成一条完整的 IMU history 样本。
4. accl 或 quat 已晚于当前 gyro 时，丢弃该条无法补齐的 gyro。
5. gyro timestamp 严格回退时清空原始 IMU 状态并按第 2 节开始新的纪元；时间戳重复时清空 history 并重新同步。

队列溢出时同样清空原始 IMU 状态并重新同步。

ACK、重试与失败：operation 或 seq 不属于当前待处理命令的 ACK 被忽略。operation 和 seq 已匹配、但 active level、reserved、effective period 或 START `trigger_sequence` 非法时，状态进入 `FAILED`。STOP / START 初次发布后，以 raw gyro 时间每 100 ms 重发同一条完整命令，最多重试三次。仍在待发送 FIFO（outbound FIFO）中、尚未真正发布的命令在发布之后才开始计时与计数。重试耗尽、相机切档失败或待发送 FIFO 无法接纳控制项时，状态进入 `FAILED`。

`TRIGGER` is the default mode on the machine. On construction it starts one same-profile resynchronization of the current profile, through the state sequence of the first code block above.

On a real profile switch, one more state follows `SETTLING`, as in the second code block above.

Procedure:

1. The host sends `STOP_TRIGGER{seq, active_level, trigger_period_us=0}`.
2. The MCU stops triggering and replies with an ACK. The host validates operation, seq, active level, reserved and `effective_period_us=0`.
3. Later gyro times advance `camera_settle_us` (default 10 ms), starting at the timestamp of the STOP ACK.
4. On a real profile switch, `CameraBase::SwitchProfile()` is called outside the state lock and the returned profile and geometry are validated; a same-profile resynchronization skips this step.
5. The host sends `START_TRIGGER{seq, active_level, trigger_period_us}`.
6. The START ACK returns the same period and `trigger_sequence=0`, and the state becomes `RUNNING`.
7. The first real edge after START has `trigger_sequence` 1; every following real edge increases it by one with strictly increasing timestamps, and the `seq` of the event equals the START seq in effect.

The procedure consists of the three steps STOP / settle / START, implemented by the two commands `STOP_TRIGGER` and `START_TRIGGER`. `CameraFrameSyncMode::RAW_PROBE` is a source-level alias of `TRIGGER`.

Image and edge matching: once `RUNNING`, every valid `FRAME_TRIGGER` enters the trigger FIFO of capacity 256. The first image is paired with the first real edge in the queue after this START. Later images select the target edge by the formulas in the first code block above.

The residual limit is the expression in the second code block above.

`stride` is at most 128. When a valid `stride` is 2 or larger, the Module skips older real edges in the trigger FIFO and consumes the target sequence directly, which corresponds to the camera having delivered fewer images. While the target edge has not arrived, the image keeps waiting. In the following cases the Module clears the local matching and runs STOP / settle / START again on the current profile, keeping the current camera profile meanwhile: an unexplainable camera gap, a repeated or regressing camera time, a discontinuous edge sequence, a regressing edge time, a trigger FIFO overflow.

After an edge is matched, the Module looks up the corresponding sample in the IMU history of at most 1024 entries. `offset_us=0` requires an exact hit on the edge time; a non-zero offset selects, in the IMU time domain, the nearest sample within 500 us around `trigger_timestamp + offset_us`. While the future sample has not arrived the Module keeps waiting, and when no valid sample exists it resynchronizes. Whichever IMU content is selected, the published `SyncedFrame::imu.timestamp_us` keeps the original `FRAME_TRIGGER` time. `SetOffsetUs()` changes the offset at run time.

Raw IMU assembly: `TRIGGER` mode assembles `gyro / accl / quat` keyed on gyro:

1. The three callbacks feed queues of capacity 1024 each.
2. accl and quat entries older than the current gyro are discarded.
3. When the three timestamps are equal, one complete IMU history sample is produced.
4. When accl or quat is already later than the current gyro, the gyro entry that cannot be completed is discarded.
5. A strict gyro timestamp regression clears the raw IMU state and establishes a new epoch as described in section 2; a repeated timestamp clears the history and resynchronizes.

A queue overflow likewise clears the raw IMU state and resynchronizes.

ACK, retry and failure: an ACK whose operation or seq does not belong to the pending command is ignored. When operation and seq match but the active level, reserved, effective period or START `trigger_sequence` is invalid, the state becomes `FAILED`. After the first publication of STOP / START, the identical complete command is resent every 100 ms of raw gyro time, at most three retries. A command still in the outbound FIFO and not yet published starts its timing and counting after it is published. When the retries are exhausted, a camera switch fails or the outbound FIFO cannot accept a control item, the state becomes `FAILED`.

## 4. 固定 profile 与逐帧 geometry / Fixed Profiles and Per-frame Geometry

相机通过 `Profiles()` 声明一到两个固定 profile（构造时检查 ID 唯一、周期非零、几何合法）。每项包含 `ProfileId`、逐帧 geometry 和直接传给 MCU 的 `trigger_period_us`。

`RequestProfile()` 接纳异步的 STOP / 稳定等待 / SwitchProfile / START 事务：

- 相机不支持的 profile 返回 `NOT_SUPPORT`。
- 非 `RUNNING` 状态（包括控制阶段和 `FAILED`）返回 `STATE_ERR`。
- `RUNNING` 中请求当前 profile 直接返回 `OK`，命令与 `SwitchProfile()` 均省略。
- 单 profile 相机在启动和异常恢复时执行 STOP / START，相机 profile 保持不变。
- `SwitchProfile()` 失败或返回不一致的结果时进入 `FAILED`，START 保持未发送，旧配置视为失效，回滚由调用方决定。
- 命令 ACK 超时同样进入 `FAILED`，MCU 是否已执行命令、触发是否已停止，需另行确认。

非 `RUNNING` 控制阶段到达的图像立即释放。切档过程中，已被下游持有的旧 profile `SharedFrame` 与新 profile 图像可以同时存在。

构造时模块复制相机的原生 `CameraCalibration`（`Calibration()` 返回）。每张图像携带不可变的 `ImageFrame::geometry`，提交后像素和 geometry 保持只读。CameraFrameSync 验证该快照能由 `FrameLayoutV` 承载并落在原生标定范围内，其它合法 geometry 同样被接受。

profile id 因此仅存在于控制面。下游按当前帧的 geometry 把像素坐标映射到统一的原生相机坐标或物理坐标。

The camera declares one or two fixed profiles through `Profiles()` (the constructor checks unique IDs, non-zero periods and valid geometry). Each entry holds a `ProfileId`, the per-frame geometry and the `trigger_period_us` passed directly to the MCU.

`RequestProfile()` admits the asynchronous STOP / settle / SwitchProfile / START transaction:

- A profile unsupported by the camera returns `NOT_SUPPORT`.
- Any state other than `RUNNING` (including the control phases and `FAILED`) returns `STATE_ERR`.
- Requesting the current profile while `RUNNING` returns `OK` directly, omitting both the commands and `SwitchProfile()`.
- A single-profile camera runs STOP / START at startup and during fault recovery, and its camera profile stays unchanged.
- When `SwitchProfile()` fails or returns an inconsistent result, the state becomes `FAILED`, START stays unsent, the old configuration is treated as invalid, and rollback is left to the caller.
- A command ACK timeout also leads to `FAILED`; whether the MCU executed the command and whether triggering has stopped has to be confirmed separately.

Images arriving in a control phase other than `RUNNING` are released immediately. During a switch, old-profile `SharedFrame` handles still held downstream can coexist with new-profile images.

On construction the Module copies the native `CameraCalibration` of the camera (returned by `Calibration()`). Every image carries an immutable `ImageFrame::geometry`, and pixels and geometry stay read-only after submission. CameraFrameSync validates that the snapshot can be carried by `FrameLayoutV` and lies within the native calibration range; any other valid geometry is accepted as well.

The profile id therefore lives in the control plane only. Downstream Modules map pixel coordinates to unified native-camera or physical coordinates with the geometry of the current frame.

## 5. LATEST_IMU 模式 / LATEST_IMU Mode

`LATEST_IMU` 对应数据源已经自行同步的路径：

- 发送 `CameraSync::SyncCommand` 的步骤省略。
- 仅当前 profile 可用（`RequestProfile()` 对当前 profile 返回 `OK`，其余返回 `NOT_SUPPORT`）。
- 只消费 quat 样本，以 quat 的 Topic 消息时间戳作为输出权威时间，角速度和加速度置零。
- 每张图像取最新 quat；自上次输出后没有时间戳更新的 quat 时，该图像被丢弃，同一 quat timestamp 只发布一次。
- `offset_us` 作用于 `TRIGGER` 的边沿匹配，`LATEST_IMU` 使用 quat 时间原值。

该模式的内部控制状态为 `BYPASS`。

`LATEST_IMU` serves the path where the data source is already synchronized by itself:

- The step of sending `CameraSync::SyncCommand` is omitted.
- Only the current profile is available (`RequestProfile()` returns `OK` for the current profile and `NOT_SUPPORT` for any other).
- Only quat samples are consumed; the quat envelope timestamp is the authoritative output time, and angular velocity and acceleration are set to zero.
- Each image takes the latest quat; when no quat with a newer timestamp has arrived since the last output, the image is discarded, and one quat timestamp is published once.
- `offset_us` applies to the edge matching of `TRIGGER`, while `LATEST_IMU` uses the quat time as is.

The internal control state of this mode is `BYPASS`.

## 6. 所有权与监控 / Ownership and Monitoring

Topic 中的指针在当前同步回调返回前有效。图像需要在回调后继续使用时，订阅者在回调内复制整个 `SyncedFrame` 或其中的 `SharedFrame`，再把副本移动到自己的稳定工作槽位。异步订阅者（如 `QueuedSubscriber`）保存复制后的副本。

每份 `SharedFrame` 共同持有 CameraBase 两槽对象池中的同一槽位。最后一份句柄析构后槽位归还对象池，像素不被复制，每帧也无堆内存分配。CameraFrameSync 内部的图像 FIFO、触发边沿 FIFO 和待发送 FIFO 是等待匹配及锁外发布用的固定容量状态。

启动日志记录输入 / 输出 Topic、模式、active profile、触发周期和稳定等待时间。`OnMonitor()` 由 XRobot 监控循环调用，报告当前控制状态、profile、period，以及自上次调用以来的 raw / assembled IMU、真实 trigger、输入 / 持有 / 丢弃图像、同步输出、重同步和溢出计数，另报告当前持有的图像数和待匹配 trigger 数。控制状态取值：

- `BYPASS`
- `WAIT_STOP_ACK`
- `SETTLING`
- `SWITCHING_CAMERA`
- `WAIT_START_ACK`
- `RUNNING`
- `FAILED`

此外，`OnMonitor()` 输出 `pending_processing` 的累计次数、平均、最小和最大耗时，单位均为 us。该统计覆盖每次 `ProcessPendingLocked()` 调用，包含没有可匹配帧时的快速返回，用于观察同步处理开销。非法 geometry 只在首次出现时打印详细错误。

The pointers in the Topics are valid until the current synchronous callback returns. A subscriber that needs the image after the callback copies the whole `SyncedFrame` or its `SharedFrame` inside the callback and moves the copy into its own stable work slot. An asynchronous subscriber (such as `QueuedSubscriber`) stores the copied value.

Every `SharedFrame` co-owns the same slot of the two-slot object pool of CameraBase. When the last handle is destroyed the slot returns to the pool; pixels are not copied and no heap memory is allocated per frame. The image, trigger and outbound FIFOs inside CameraFrameSync are fixed-capacity state used for pending matching and publication outside the lock.

The startup log records the input / output Topics, the mode, the active profile, the trigger period and the settle time. `OnMonitor()` is called by the XRobot monitor loop and reports the current control state, profile and period, the counts since the previous call of raw / assembled IMU, real triggers, input / retained / dropped images, synchronized outputs, resynchronizations and overflows, plus the number of currently held images and pending triggers. The control state takes one of:

- `BYPASS`
- `WAIT_STOP_ACK`
- `SETTLING`
- `SWITCHING_CAMERA`
- `WAIT_START_ACK`
- `RUNNING`
- `FAILED`

In addition, `OnMonitor()` outputs the cumulative count and the average, minimum and maximum duration of `pending_processing`, all in us. The statistics cover every `ProcessPendingLocked()` call, including quick returns when no frame can be matched, and show the synchronization processing overhead. An invalid geometry prints a detailed error on its first occurrence only.

## 7. 构造接口 / Constructor

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class CameraFrameSync;

CameraFrameSync(Base& camera, RuntimeParam runtime = DefaultRuntime());
```

模板参数：

- `FrameLayoutV`：帧布局，与相机实例的模板参数相同。

依赖：

- `camera`：`CameraBase<FrameLayoutV>&`，前面某个相机模块实例（HikCamera、WebotsCamera、CaptureFileCamera 等）。

配置参数（`RuntimeParam`，`DefaultRuntime()` 即全部默认值）：

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `mode` | `CameraFrameSyncMode::TRIGGER` | `TRIGGER` 或 `LATEST_IMU`。 |
| `offset_us` | `0` | 在 IMU 时间域中相对触发时间的取样偏移，单位 us，作用于 `TRIGGER`。 |
| `host_topic_domain_name` | `"shared_memory"` | 命令、事件和原始 IMU Topic 的 domain。 |
| `sync_command_topic_name` | `"camera_sync_command"` | 发给 CameraSync 的命令 Topic。 |
| `sync_result_topic_name` | `"camera_sync_result"` | CameraSync 回执和触发事件 Topic。 |
| `sync_active_level` | `1` | 触发有效电平，非零按 1 处理。 |
| `camera_settle_us` | `10000` | STOP ACK 后等待相机稳定的时间，单位 us。 |
| `raw_imu_frame` | `CameraFrameSyncRawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP` | 原始 IMU 坐标约定，见第 2 节。 |
| `raw_quat_topic_name` | 空 | 非空时替代 `<camera_name>_quat`。 |
| `synced_frame_topic_name` | 空 | 为空时使用 `<图像 topic>_synced`。 |

相机名、图像 Topic、domain 和两个同步 Topic 名为非空值，相机 profile 合法；否则构造时 `REQUIRE` 失败。

公共方法：`SyncedFrameTopicName()`、`RawTopicDomainName()`、`Calibration()`、`Profiles()`、`ActiveProfile()`、`GetSyncMode()`、`RequestProfile()`、`SetOffsetUs()`、`FlushPendingFrames()`（立即处理待匹配数据后清空图像和 trigger 队列）、`OnMonitor()`。

Template parameter:

- `FrameLayoutV`: the frame layout, identical to the template parameter of the camera instance.

Dependencies:

- `camera`: `CameraBase<FrameLayoutV>&`, an earlier camera Module instance (HikCamera, WebotsCamera, CaptureFileCamera, and so on).

Configuration parameters (`RuntimeParam`; `DefaultRuntime()` holds all defaults):

| Field | Default | Meaning |
| --- | --- | --- |
| `mode` | `CameraFrameSyncMode::TRIGGER` | `TRIGGER` or `LATEST_IMU`. |
| `offset_us` | `0` | Sampling offset relative to the trigger time in the IMU time domain, in us; applies to `TRIGGER`. |
| `host_topic_domain_name` | `"shared_memory"` | Domain of the command, event and raw IMU Topics. |
| `sync_command_topic_name` | `"camera_sync_command"` | Command Topic sent to CameraSync. |
| `sync_result_topic_name` | `"camera_sync_result"` | CameraSync acknowledgement and trigger event Topic. |
| `sync_active_level` | `1` | Trigger active level; non-zero is treated as 1. |
| `camera_settle_us` | `10000` | Time to wait for the camera to settle after the STOP ACK, in us. |
| `raw_imu_frame` | `CameraFrameSyncRawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP` | Raw IMU coordinate convention, see section 2. |
| `raw_quat_topic_name` | empty | Replaces `<camera_name>_quat` when non-empty. |
| `synced_frame_topic_name` | empty | `<image Topic>_synced` is used when empty. |

The camera name, the image Topic, the domain and the two synchronization Topic names are non-empty and the camera profiles are valid; otherwise `REQUIRE` fails in the constructor.

Public methods: `SyncedFrameTopicName()`, `RawTopicDomainName()`, `Calibration()`, `Profiles()`, `ActiveProfile()`, `GetSyncMode()`, `RequestProfile()`, `SetOffsetUs()`, `FlushPendingFrames()` (processes pending data immediately, then clears the image and trigger queues) and `OnMonitor()`.

## 8. Topic

`<camera_name>` 是相机的实例名（`camera.NameView()`）。命令、事件和原始 IMU Topic 使用 `host_topic_domain_name`。默认 domain 是 `shared_memory`；产品配置可以改为 `host` 等 SharedTopic 转发 domain，进程内仿真使用默认域 `libxr_def_domain`。图像和 `SyncedFrame` 是当前进程内的普通 Topic，裸指针在进程内传递。

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `camera.ImageTopicNameView()` | 订阅 | `const SharedFrame*` | 相机图像，载荷在同步回调期间有效 |
| `<camera_name>_gyro` | 订阅 | `Eigen::Matrix<float, 3, 1>` | 角速度，单位 rad/s |
| `<camera_name>_accl` | 订阅 | `Eigen::Matrix<float, 3, 1>` | 加速度，单位 m/s^2 |
| `<camera_name>_quat`（`raw_quat_topic_name` 非空时使用该名字） | 订阅 | `LibXR::Quaternion<float>` | 姿态四元数 |
| `sync_result_topic_name`（默认 `camera_sync_result`） | 订阅 | `CameraSync::SyncEvent` | CameraSync 的 ACK 与触发事件 |
| `synced_frame_topic_name`，为空时为 `<图像 Topic>_synced` | 发布 | `const SyncedFrame*` | 同步结果，载荷在同步回调期间有效；名字由 `SyncedFrameTopicName()` 查询 |
| `sync_command_topic_name`（默认 `camera_sync_command`） | 发布 | `CameraSync::SyncCommand` | STOP / START 触发命令 |

`<camera_name>` is the instance name of the camera (`camera.NameView()`). The command, event and raw IMU Topics use `host_topic_domain_name`. The default domain is `shared_memory`; a product configuration can set a SharedTopic forwarding domain such as `host`, and in-process simulation uses the default domain `libxr_def_domain`. The image and `SyncedFrame` are ordinary Topics inside the current process, and raw pointers are passed within the process.

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `camera.ImageTopicNameView()` | Subscribe | `const SharedFrame*` | Camera image, payload valid during the synchronous callback |
| `<camera_name>_gyro` | Subscribe | `Eigen::Matrix<float, 3, 1>` | Angular velocity in rad/s |
| `<camera_name>_accl` | Subscribe | `Eigen::Matrix<float, 3, 1>` | Acceleration in m/s^2 |
| `<camera_name>_quat` (the name from `raw_quat_topic_name` when non-empty) | Subscribe | `LibXR::Quaternion<float>` | Attitude quaternion |
| `sync_result_topic_name` (default `camera_sync_result`) | Subscribe | `CameraSync::SyncEvent` | ACKs and trigger events of CameraSync |
| `synced_frame_topic_name`, `<image Topic>_synced` when empty | Publish | `const SyncedFrame*` | Synchronized result, payload valid during the synchronous callback; the name is returned by `SyncedFrameTopicName()` |
| `sync_command_topic_name` (default `camera_sync_command`) | Publish | `CameraSync::SyncCommand` | STOP / START trigger commands |

## 9. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/CameraFrameSync --template-arg <FrameLayout>` 写入的实例：`template_args` 为帧布局 constexpr，`camera` 填为相机实例的 id，`runtime` 为 `DefaultRuntime()` 表达式，`RuntimeParam` 的默认值见第 7 节。帧布局与相机实例一致：

An instance written by `xrobot instance add QDU-Robomaster/CameraFrameSync --template-arg <FrameLayout>`: `template_args` is the frame layout constexpr, `camera` is set to the id of a camera instance, and `runtime` is the `DefaultRuntime()` expression, with the defaults of `RuntimeParam` listed in section 7. The frame layout equals that of the camera instance:

```yaml
constexpr_namespace: AutoAimRunConfig
constexpr_includes:
  - CameraBase.hpp
constexprs:
  MainFrameLayout:
    type: CameraTypes::FrameLayout
    value: '{.width = 800, .height = 600, .step = 2400, .encoding = CameraTypes::Encoding::BGR8}'
modules:
  - module: QDU-Robomaster/CameraFrameSync
    id: CameraFrameSync_0
    template_args:
      - AutoAimRunConfig::MainFrameLayout
    args:
      - camera: WebotsCamera_0
      - runtime: CameraFrameSync<AutoAimRunConfig::MainFrameLayout>::DefaultRuntime()
```

`WebotsCamera_0` 是 WebotsCamera（或 HikCamera、CaptureFileCamera）实例的 id，它列在本实例之前，使用相同的 `template_args`。`runtime` 也可以展开为 `RuntimeParam` 全部字段的 YAML 映射，字段顺序见第 7 节；数据源已经自行同步时，其中 `mode` 取 `CameraFrameSyncMode::LATEST_IMU`。ArmorDetector、ArmorTracker 等下游模块以 `sync: CameraFrameSync_0` 引用本实例。

`WebotsCamera_0` is the id of a WebotsCamera (or HikCamera, CaptureFileCamera) instance, listed before this instance and using the same `template_args`. `runtime` can also be expanded into a YAML mapping of all `RuntimeParam` fields in the order of section 7; when the data source is already synchronized by itself, its `mode` is `CameraFrameSyncMode::LATEST_IMU`. Downstream Modules such as ArmorDetector and ArmorTracker reference this instance with `sync: CameraFrameSync_0`.

## 10. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/CameraBase`：相机基类、帧布局、`SharedFrame` 与 `ImuStamped`。
- `QDU-Robomaster/CameraSync`：`SyncCommand` / `SyncEvent` 协议类型；MCU 侧（或仿真中同进程）的 CameraSync 实例负责实际触发和回执。
- `xrobot-org/DurationStatistics`：`pending_processing` 耗时统计。
- LibXR（含 Eigen）。

硬件：相机的触发输入由 MCU 侧 CameraSync 驱动；IMU 数据来自发布 `<camera_name>_gyro`、`<camera_name>_accl`、`<camera_name>_quat` 的实例。模块为 header-only 模板，所需的硬件对象通过相机实例和 Topic 获得。

Dependencies:

- `QDU-Robomaster/CameraBase`: camera base class, frame layout, `SharedFrame` and `ImuStamped`.
- `QDU-Robomaster/CameraSync`: `SyncCommand` / `SyncEvent` protocol types; the CameraSync instance on the MCU side (or in the same process in simulation) performs the actual triggering and acknowledgement.
- `xrobot-org/DurationStatistics`: duration statistics of `pending_processing`.
- LibXR (including Eigen).

Hardware: the camera trigger input is driven by CameraSync on the MCU side; the IMU data comes from the instances publishing `<camera_name>_gyro`, `<camera_name>_accl` and `<camera_name>_quat`. The Module is a header-only template; the hardware objects it needs come through the camera instance and the Topics.
