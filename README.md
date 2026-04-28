# CameraFrameSync

`CameraFrameSync` 负责两件事：

1. 承接 `CameraBase<Info>` 的图像 lease，并把图像发布到 `LinuxSharedTopic`
2. 订阅原始 `gyro / accl / quat`，用图像提交时的 `ImageFrame::timestamp_us` 完成图像与 IMU 对齐，再发布同步后的 `imu`

当前支持两种同步模式：

- `RAW_PROBE`
  - 当前默认模式
  - 通过原始传感器节拍和一次性 `sensor_sync_cmd` 探针锁定图像与 IMU
- `LATEST_IMU`
  - 兼容之前“默认认为图像与 IMU 已同步”的用法
  - 每张图像直接取当前最新的原始 IMU 作为同步 IMU
  - 仍然会在 IMU 自己时间域里应用 `offset_us`

## 输入与输出

输入：

- 图像写入接口
  - `CameraBase<Info>::RegisterImageSink(...)`
- 原始小话题
  - `<camera.Name()>_gyro`
  - `<camera.Name()>_accl`
  - `<camera.Name()>_quat`
- 一次性同步探针命令
  - `sensor_sync_cmd`

输出：

- 图像共享话题
  - `camera.ImageTopicName()`
- 同步后 IMU 话题
  - `camera.ImuTopicName()`

其中：

- `sensor_sync_cmd` 是固定名字，不跟随相机名变化
- 原始 `gyro / accl / quat` 前缀直接取 `camera.Name()`
- `camera.Name()` 必须非空；模块内部不再做隐式回退
- 可通过 `SetSyncMode(...)` 在 `RAW_PROBE / LATEST_IMU` 之间切换
- 运行时只保留一个调节点：
  - `offset_us`
  - 它表示在 IMU 自己的 `sensor_timestamp_us` 时间域里，对最终取样位置做常量平移

## 当前同步策略

先说模式边界：

- `RAW_PROBE`
  - 使用下面整套 cadence / probe / 重锁逻辑
- `LATEST_IMU`
  - 不发 `sensor_sync_cmd`
  - 不要求图像 cadence 锁定
  - 每次处理图像时直接取当前 `imu_history` 里的最新 IMU
  - 如果 `offset_us` 指向的最终 IMU 还没到，就把当前图像留在唯一的待处理槽位里等待

这版实现只使用**传感器侧时间戳**：

- 原始 `gyro / accl / quat` 使用 `sensor_timestamp_us`
- 图像使用 `ImageFrame::timestamp_us`
- 同步主逻辑里已经**没有**主机到达时间戳

约束也很明确：

- 不拿相机时间戳和 IMU 时间戳做跨域绝对值比较
- 只在各自时间域里做差分
- 跨域关系只靠：
  - 图像基线周期 `T`
  - IMU 基线周期 `t`
  - 估算出的整数步长 `N = round(T / t)`
  - 一次性 `2T` 探针引入的节拍变化

## 数据流

1. `gyro / accl / quat` 回调只入各自无锁 ingress 队列
2. 图像 sink commit 回调记录 `ImageFrame::timestamp_us`，它是唯一同步触发点
3. 每次处理图像提交时：
   - 先排空四路 ingress
   - 再按 `gyro` 主时间轴组装原始 IMU 历史
   - 最后串行处理待同步图像时间戳
   - 当前如果有一张图像还在等 offset 目标 IMU，就继续重试这一张，不会改成处理后面的图像

## 原始 IMU 组装

组装规则：

- 以 `gyro.sensor_timestamp_us` 为主轴
- `accl / quat` 在同一个 IMU 时间域里配对

具体做法：

- 启动初期还没观察到稳定 IMU 周期时
  - 为 `accl / quat` 各取第一条时间不早于 `gyro` 的样本
- 一旦已经得到稳定 IMU 周期
  - 在 `gyro` 的正负半个周期窗口内
  - 为 `accl / quat` 各选离它最近的样本
  - 如果前后距离完全相同，优先更晚样本
- 如果某一路已经越过半周期窗口还找不到可配对样本
  - 当前 `gyro` 直接丢弃
  - 避免整条组装链被卡死

## 图像侧状态机

这一节主要描述 `RAW_PROBE` 模式。

状态只有两种，再加一个 `probe_pending` 标记：

- `OBSERVING`
  - 持续观察节拍并维护原始 IMU 历史
  - 图像提交和 `gyro` 节拍稳定后，才允许发一次 `sensor_sync_cmd`
- `SYNCED`
  - 已经进入稳态跟踪

其中：

- `probe_pending = false`
  - 当前没有等待中的 `2T` 探针
- `probe_pending = true`
  - 已经发过一次 probe，正在等待那次 `2T` 图像 gap

任何一条稳定节拍断掉，都会直接 reset 回 `OBSERVING`。

## 锁定流程

这一节只适用于 `RAW_PROBE`。

1. 持续观察节拍
   - 图像提交 cadence 稳定后记录正常图像基线周期 `T`
   - 原始 IMU 历史形成稳定周期后记录 `t`
2. 根据 `T / t` 估算图像对应的 IMU 整数步长 `N`
3. 自动发出一次 `sensor_sync_cmd`
4. 下位机只把**下一次图像间隔**拉成 `2T`，随后自动恢复到 `T`
5. 当主机看到 `T -> 2T` 这次图像节拍变化时：
   - 在 IMU 历史里找一对间隔约为 `2 * N * t` 的 IMU
   - 这对 IMU 的后一条就是 probe 图像对应的同步 IMU 候选
6. 如果 probe 图像还缺少后续 IMU 历史，先继续等待
7. 命中后记下：
   - 上一条同步 IMU 时间戳
   - 同步 IMU 周期
8. 命中后立刻进入 `SYNCED`
9. 后续普通图像进入稳态跟踪

## 稳态跟踪

这一节只适用于 `RAW_PROBE`。

锁定后不再全局扫描，而是沿着上一帧关系递推：

1. 预测下一条同步 IMU 的目标时间戳
   - `expected_sync_imu_ts = last_sync_imu_ts + sync_imu_period`
2. 只有当 IMU 历史已经覆盖这个目标时，才允许处理当前图像
3. 在 IMU 历史里找离这个目标最近、且误差仍在容差内的样本
4. 找到后，先把它当作“同步 IMU”
5. 再在 IMU 自己时间域里应用 `offset_us`
6. 由 `sync_imu_ts + offset_us` 找最终要发布的 IMU 样本

如果这一步只差 `offset_us` 对应的目标 IMU 还没到：

- 当前图像会停在模块内部唯一的“待处理图像”槽位里等待
- 但已经选中的 `sync_imu` 不会在下一次重试时被重新改写
- 也就是“等最终样本”，而不是“边等边改同步基准”

最终发布时：

- 图像仍保留自己的相机侧时间戳
- 同步后 `imu.timestamp_us` 也使用这张图像的传感器侧时间戳

## 等待、拒绝与恢复

`LATEST_IMU` 模式下不会走 probe 锁定，也不会按 `T / 2T` 图像 gap 做接受判定。
它只保留两类行为：

- `WAIT`
  - 当前图像选中的“最新 IMU”还缺少 `offset_us` 对应的最终样本
- `DROP`
  - 图像时间戳回退/乱序
  - 或者已经覆盖 `offset_us` 目标，但仍找不到合法最终 IMU

`RAW_PROBE` 模式则继续使用下面这套完整重锁语义：

这版实现不再用主机到达时间做 timeout，而是全都按**序列语义**处理：

- `WAIT`
  - IMU 历史还没覆盖 probe 候选
  - 或者还没覆盖稳态预测目标
  - 或者 `offset_us` 目标样本还没到
- `REJECT`
  - probe 挂起后，下一张图像 gap 不是 `2T`
  - 稳态下普通图像 gap 不是 `T`
  - 预测目标已经被 IMU 历史覆盖，但仍找不到合法候选
- `RESET`
  - 任一路 cadence 失稳
  - ingress / history 溢出
  - 图像事件回退或乱序

reset 后会：

- 清掉 probe 挂起状态
- 清掉当前等待中的图像
- 清掉当前锁定关系
- 清掉图像基线
- 立刻回到 `OBSERVING`

## `Subscriber` 语义

`Subscriber::Wait()` 仍然保持严格匹配：

- 先等一帧图像
- 再等一条 `timestamp_us` 与该图像完全相同的同步后 `imu`
- 两者对上才返回 `SyncedFrame`

下游 `Subscriber` 不参与原始 IMU 重同步；重同步都在 `CameraFrameSync` 内部完成。

## 默认边界

- 图像共享话题
  - `slot_num = 8`
  - `queue_num = 2`
- ingress 队列长度
  - `imu_ingress_length = 128`
  - `image_ingress_length = 32`
- 历史缓存长度
  - `pending_limit = 256`
  - `history_limit = 256`
- `Subscriber` 内部等待队列长度
  - `queue_length = 32`
- 节拍稳定观察阈值
  - `cadence_stable_gaps = 2`
  - `raw_cadence_min_tolerance_us = 300`
  - `image_cadence_min_tolerance_us = 1500`

## 模板参数与依赖

- 模板参数
  - `Info`：编译期 `CameraTypes::CameraInfo`
- 依赖
  - `qdu-future/CameraBase`
