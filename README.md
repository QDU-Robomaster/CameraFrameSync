# CameraFrameSync

Shared synced-frame payload definition for `camera_frame_sync(shared)`.

`CameraFrameSync<Info>` no longer owns a runtime forwarding module.

The runtime publisher is now `WebotsCamera<Info>` itself:

1. sample pose and motion
2. create one shared payload slot directly
3. write image bytes into that final payload
4. publish without blocking

Output topic remains:

- `camera_frame_sync(shared)`

Current shared payload layout:

- `image.timestamp_us`
- `image.sequence`
- `image.data`
- `pose.rotation_wxyz`
- `pose.translation_xyz`
- `motion.angular_velocity_xyz`
- `motion.linear_acceleration_xyz`

Default shared topic policy:

- `slot_num = 8`
- `queue_num = 2`
- producer path is non-blocking; if no writable slot is available, the new frame is dropped

## Template Arguments

- `Info`: compile-time `CameraTypes::CameraInfo`

## Depends

- `qdu-future/CameraBase`
