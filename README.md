# CameraFrameSync

Shared image-transport bridge plus image+imu sync subscriber.

`CameraFrameSync<Info>` is the current typed image lease sink implementation for
`CameraBase<Info>`.

## Runtime Role

1. lease one writable `ImageFrame` slot from `LinuxSharedTopic`
2. register itself to `CameraBase<Info>` as the producer-side image lease sink
3. when the camera calls `CommitImage()`, publish the current shared slot
4. lease the next writable slot for the producer
5. let downstream subscribers wait until image and imu timestamps match

## Output Topics

- image shared topic:
  - same name as `camera.ImageTopicName()`
- imu async topic:
  - same name as `camera.ImuTopicName()`

## Synced Payload

- `SyncedFrame.image`
  - shared lease of `CameraBase<Info>::ImageFrame`
- `SyncedFrame.imu`
  - copied `CameraBase<Info>::ImuStamped`

## Default Policy

- `slot_num = 8`
- `queue_num = 2`
- producer path is non-blocking
- if no spare writable slot is available, the current writable slot is reused
  and the new frame is dropped

## Template Arguments

- `Info`: compile-time `CameraTypes::CameraInfo`

## Depends

- `qdu-future/CameraBase`
