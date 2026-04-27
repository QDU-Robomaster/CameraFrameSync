# CameraFrameSync

`CameraFrameSync` 负责两件事：

1. 承接 `CameraBase<Info>` 的图像 lease，并把图像发布到 `LinuxSharedTopic`
2. 订阅原始 `gyro / accl / quat / image_event`，在模块内部完成图像与 IMU 同步，再发布同步后的 `imu`

## 输入与输出

输入：

- 图像写入接口
  - `CameraBase<Info>::RegisterImageSink(...)`
- 原始小话题
  - `<camera.Name()>_gyro`
  - `<camera.Name()>_accl`
  - `<camera.Name()>_quat`
  - `<camera.Name()>_image_event`
- 一次性同步探针命令
  - `sensor_sync_cmd`

输出：

- 图像共享话题
  - `camera.ImageTopicName()`
- 同步后 IMU 话题
  - `camera.ImuTopicName()`

其中：

- `sensor_sync_cmd` 是固定名字，不跟随相机名变化
- 原始 `gyro / accl / quat / image_event` 前缀直接取 `camera.Name()`
- `camera.Name()` 必须非空；模块内部不再做隐式回退
- 对下游稳定暴露的只有共享图像与同步后 `imu`；原始小话题只服务于模块内部同步
- 运行时只保留一个调节点：
  - `offset_us`
  - 它表示在 IMU 自己的 `sensor_timestamp_us` 时间域里，对最终取样位置做常量平移

## 当前同步策略

这个模块现在明确分成两段：

1. 先锁“图像对应的是哪一个同步 gyro 帧”
2. 再在 IMU 自己的时间域里应用 `offset_us`，找最终要发给下游的 IMU 样本

也就是说：

- **不会**再把图像域时间戳和 IMU 域时间戳直接拿来做跨域最近邻匹配
- `rx_time` 只用于主机侧锁定同步关系
- `sensor_timestamp_us` 只在 IMU 域内用于 `offset` 推导

另外，`CameraFrameSync` 本身不创建独立同步线程：

- `gyro / accl / quat` 回调只入队
- `image_event` 回调是唯一串行同步触发点
- 所有重同步、探针发送、图像匹配、IMU 发布都在这条 `image_event` 路径里推进
- 节拍稳定观察是常驻的，不是只在启动时做一次

模块内部的粗状态可以理解成：

- `OBSERVING`
  - 启动后默认就处在这里
  - 一直观察 `gyro / accl / quat / image_event` 四路到达节拍
  - 只有四路都稳定，才允许发一次 `sensor_sync_cmd`
- `LOCKING`
  - 已经看到 probe 的 `2T` 图像 gap，正在确认锁定关系
- `SYNCED`
  - 已经进入稳态跟踪

这里不再单独保留 `RECOVERING` 状态：

- 发现同步关系失效时，直接执行一次 reset
- 清掉当前锁定关系、图像基线、待发 probe
- 然后立刻回到 `OBSERVING`

## 详细流程

1. `gyro / accl / quat` 回调只负责入各自 ingress 队列
2. `image_event` 回调作为唯一同步触发点，串行排空所有 ingress
3. 以 `gyro` 为主时间轴，向后找第一条时间不早于它的 `accl / quat`，组装原始 IMU 历史
4. 持续观察 `gyro / accl / quat / image_event` 四路 `rx_time` 节拍
   - 任一路还没稳定，就继续停在 `OBSERVING`
   - 任一路在稳定后又失稳，就立刻清掉当前锁定关系
5. 对正常图像流记录基线 `T`，并据此估算 `image_rate / imu_rate` 的整数步长
6. 四路节拍都稳定后，自动发出一次性 `sensor_sync_cmd`
7. 等待 `image_event` 出现一次 `T -> 2T -> T` 的 probe 节拍变化
   - 这里的 `2T` 是主动探针，不算 image cadence 失稳
8. 对 probe 帧：
   - 利用“前一张普通图像 -> 当前 probe 图像”的**双周期变化**
   - 在主机侧 `rx_time` 上搜索最合理的同步 gyro 帧
   - 锁住 `image -> sync gyro frame` 的对应关系
9. 锁定后，后续图像先按上一帧关系预测下一条同步 IMU，再用 `host skew` 与 IMU 域周期做校验
10. 最后以该 gyro 的 `sensor_timestamp_us + offset_us` 为目标，在 IMU 历史里取最终样本
11. 恢复策略分两层：
   - 一般节拍失稳或匹配失败：
     - 直接 reset 回 `OBSERVING`
     - 清掉锁定关系与图像基线
     - 继续跑常驻节拍观察，等稳定后再发下一次 probe
   - 超时、队列溢出这类硬故障：
     - 额外清掉 cadence 观察状态
     - 从头重新观察四路节拍

## `Subscriber` 语义

`Subscriber::Wait()` 仍然是严格时间戳匹配：

- 等到一帧图像
- 再等待一条 `timestamp_us` 与该图像完全相同的同步后 `imu`
- 只有两者对上，才返回 `SyncedFrame`

下游 `Subscriber` 不参与原始 IMU 重同步；重同步都在 `CameraFrameSync` 内部完成。

## 默认边界

- 图像共享话题
  - `slot_num = 8`
  - `queue_num = 2`
- ingress 队列长度
  - `imu = 1024`
  - `image_event = 256`
- 历史缓存长度
  - `pending/history = 2048`
  - 内部使用定长 ring，满时覆盖最旧样本并触发恢复态
- 图像发布路径保持非阻塞
  - 如果拿不到新可写槽位，就继续复用当前槽位，并丢掉这次新图像

## 模板参数与依赖

- 模板参数
  - `Info`：编译期 `CameraTypes::CameraInfo`
- 依赖
  - `qdu-future/CameraBase`
