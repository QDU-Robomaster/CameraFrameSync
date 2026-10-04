# CameraFrameSync 内部机制 / Internals

本页记录 CameraFrameSync 的实现细节，用于维护模块和排查同步问题。使用模块时依赖的约定见 [README](../README.md)。

This page records the implementation details of CameraFrameSync, for maintaining the Module and investigating synchronization problems. The conventions that users of the Module rely on are in the [README](../README.md).

## 1. 线程与锁 / Threads and Locking

图像、原始 IMU 和 `CameraSync` 事件回调通过同一把状态锁串行推进，模块自身没有工作线程与信号量。`SyncCommand`、`SyncedFrame` 的 Topic 发布以及 `CameraBase::SwitchProfile()` 在锁外执行，因此同步订阅者可以重入模块接口。内部的图像 FIFO、触发边沿 FIFO 和待发送 FIFO（outbound FIFO）是等待匹配及锁外发布用的固定容量状态。

The image, raw IMU and `CameraSync` event callbacks advance through one shared state lock, and the Module owns no worker thread or semaphore. Publishing of the `SyncCommand` and `SyncedFrame` Topics and `CameraBase::SwitchProfile()` run outside the lock, so synchronous subscribers may re-enter the Module interface. The internal image FIFO, trigger FIFO and outbound FIFO are fixed-capacity state used for pending matching and publication outside the lock.

## 2. TRIGGER 状态机 / TRIGGER State Machine

构造时对当前 profile 发起一次同档重同步：

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

流程仅含 STOP / 稳定等待 / START 三步，由 `STOP_TRIGGER` 与 `START_TRIGGER` 两种命令实现。

On construction the Module starts one same-profile resynchronization of the current profile, through the state sequence of the first code block above. On a real profile switch, one more state follows `SETTLING`, as in the second code block above.

Procedure:

1. The host sends `STOP_TRIGGER{seq, active_level, trigger_period_us=0}`.
2. The MCU stops triggering and replies with an ACK. The host validates operation, seq, active level, reserved and `effective_period_us=0`.
3. Later gyro times advance `camera_settle_us` (default 10 ms), starting at the timestamp of the STOP ACK.
4. On a real profile switch, `CameraBase::SwitchProfile()` is called outside the state lock and the returned profile and geometry are validated; a same-profile resynchronization skips this step.
5. The host sends `START_TRIGGER{seq, active_level, trigger_period_us}`.
6. The START ACK returns the same period and `trigger_sequence=0`, and the state becomes `RUNNING`.
7. The first real edge after START has `trigger_sequence` 1; every following real edge increases it by one with strictly increasing timestamps, and the `seq` of the event equals the START seq in effect.

The procedure consists of the three steps STOP / settle / START, implemented by the two commands `STOP_TRIGGER` and `START_TRIGGER`.

## 3. 纪元恢复 / Epoch Recovery

gyro 时间戳回退时开始新的纪元：旧匹配、原始 IMU 和输出时间基线被清除，`TRIGGER` 链路以新的命令序号重新确认 STOP，忽略旧序号的 ACK，再从新的确认时间起等待稳定时间。被中断事务已消耗的重试次数保留，恢复命令共用剩余额度，超限时记录错误并进入 `FAILED`。尚未返回的相机切换在新的 STOP / 稳定等待边界之后才允许发出 START。普通 STOP / START 只清除匹配基线，同一 MCU 时钟内的输出顺序约束继续保留。

A gyro timestamp regression starts a new epoch: the old matching, raw IMU and output time baselines are cleared, the `TRIGGER` link reconfirms STOP with a new command sequence number, ignores ACKs of the old sequence number, and waits for the settle time from the newly confirmed time. The retries already consumed by the interrupted transaction are kept, and the recovery commands share the remaining budget; when it is exhausted an error is logged and the state becomes `FAILED`. A camera switch that has not returned yet allows START only after the new STOP / settle boundary. An ordinary STOP / START clears only the matching baseline, and the output-order constraint within the same MCU clock remains.

## 4. 图像与边沿匹配 / Image and Edge Matching

进入 `RUNNING` 后，每个合法 `FRAME_TRIGGER` 进入容量为 256 的触发边沿 FIFO。第一张图像与本次 START 之后队列中的第一条真实边沿配对。后续图像按下式选择目标边沿：

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

Once `RUNNING`, every valid `FRAME_TRIGGER` enters the trigger FIFO of capacity 256. The first image is paired with the first real edge in the queue after this START. Later images select the target edge by the formulas in the first code block above.

The residual limit is the expression in the second code block above.

`stride` is at most 128. When a valid `stride` is 2 or larger, the Module skips older real edges in the trigger FIFO and consumes the target sequence directly, which corresponds to the camera having delivered fewer images. While the target edge has not arrived, the image keeps waiting. In the following cases the Module clears the local matching and runs STOP / settle / START again on the current profile, keeping the current camera profile meanwhile: an unexplainable camera gap, a repeated or regressing camera time, a discontinuous edge sequence, a regressing edge time, a trigger FIFO overflow.

## 5. IMU 取样 / IMU Lookup

匹配到边沿后，模块在最多 1024 条的 IMU history 中寻找对应样本。`offset_us=0` 要求精确命中该边沿时间；非零 offset 在 IMU 时间域中选择 `trigger_timestamp + offset_us` 附近 500 us 内最近的样本。未来样本尚未到达时继续等待，找不到合法样本时重新同步。

After an edge is matched, the Module looks up the corresponding sample in the IMU history of at most 1024 entries. `offset_us=0` requires an exact hit on the edge time; a non-zero offset selects, in the IMU time domain, the nearest sample within 500 us around `trigger_timestamp + offset_us`. While the future sample has not arrived the Module keeps waiting, and when no valid sample exists it resynchronizes.

## 6. 原始 IMU 组装 / Raw IMU Assembly

`TRIGGER` 模式以 gyro 为主键组装 `gyro / accl / quat`：

1. 三路回调分别进入容量为 1024 的队列。
2. 早于当前 gyro 的 accl 和 quat 被丢弃。
3. 三路 timestamp 一致时生成一条完整的 IMU history 样本。
4. accl 或 quat 已晚于当前 gyro 时，丢弃该条无法补齐的 gyro。
5. gyro timestamp 严格回退时清空原始 IMU 状态并按第 3 节开始新的纪元；时间戳重复时清空 history 并重新同步。

队列溢出时同样清空原始 IMU 状态并重新同步。

`TRIGGER` mode assembles `gyro / accl / quat` keyed on gyro:

1. The three callbacks feed queues of capacity 1024 each.
2. accl and quat entries older than the current gyro are discarded.
3. When the three timestamps are equal, one complete IMU history sample is produced.
4. When accl or quat is already later than the current gyro, the gyro entry that cannot be completed is discarded.
5. A strict gyro timestamp regression clears the raw IMU state and establishes a new epoch as described in section 3; a repeated timestamp clears the history and resynchronizes.

A queue overflow likewise clears the raw IMU state and resynchronizes.

## 7. ACK、重试与失败 / ACK, Retry and Failure

operation 或 seq 不属于当前待处理命令的 ACK 被忽略。operation 和 seq 已匹配、但 active level、reserved、effective period 或 START `trigger_sequence` 非法时，状态进入 `FAILED`。STOP / START 初次发布后，以 raw gyro 时间每 100 ms 重发同一条完整命令，最多重试三次。仍在待发送 FIFO 中、尚未真正发布的命令在发布之后才开始计时与计数。重试耗尽、相机切档失败或待发送 FIFO 无法接纳控制项时，状态进入 `FAILED`。

An ACK whose operation or seq does not belong to the pending command is ignored. When operation and seq match but the active level, reserved, effective period or START `trigger_sequence` is invalid, the state becomes `FAILED`. After the first publication of STOP / START, the identical complete command is resent every 100 ms of raw gyro time, at most three retries. A command still in the outbound FIFO and not yet published starts its timing and counting after it is published. When the retries are exhausted, a camera switch fails or the outbound FIFO cannot accept a control item, the state becomes `FAILED`.

## 8. 监控 / Monitoring

`OnMonitor()` 报告当前控制状态、profile、period，以及自上次调用以来的 raw / assembled IMU、真实 trigger、输入 / 持有 / 丢弃图像、同步输出、重同步和溢出计数，另报告当前持有的图像数和待匹配 trigger 数。控制状态取值：

- `BYPASS`：`LATEST_IMU` 模式。
- `WAIT_STOP_ACK`
- `SETTLING`
- `SWITCHING_CAMERA`
- `WAIT_START_ACK`
- `RUNNING`
- `FAILED`

此外，`OnMonitor()` 输出 `pending_processing` 的累计次数、平均、最小和最大耗时，单位均为 us。该统计覆盖每次 `ProcessPendingLocked()` 调用，包含没有可匹配帧时的快速返回，用于观察同步处理开销。非法 geometry 只在首次出现时打印详细错误。

`OnMonitor()` reports the current control state, profile and period, the counts since the previous call of raw / assembled IMU, real triggers, input / retained / dropped images, synchronized outputs, resynchronizations and overflows, plus the number of currently held images and pending triggers. The control state takes one of:

- `BYPASS`: `LATEST_IMU` mode.
- `WAIT_STOP_ACK`
- `SETTLING`
- `SWITCHING_CAMERA`
- `WAIT_START_ACK`
- `RUNNING`
- `FAILED`

In addition, `OnMonitor()` outputs the cumulative count and the average, minimum and maximum duration of `pending_processing`, all in us. The statistics cover every `ProcessPendingLocked()` call, including quick returns when no frame can be matched, and show the synchronization processing overhead. An invalid geometry prints a detailed error on its first occurrence only.
