# CameraFrameSync

Host-side camera-frame synchronizer.

`CameraFrameSync<Info>` consumes one shared image topic and two ordinary topics:

- `image_frame(shared)`
- `camera_pose`
- `camera_motion`

Then it republishes a single shared-memory payload:

- `camera_frame_sync(shared)`

Current shared payload layout:

- full copied `SharedImageFrame`
- `pose.rotation_wxyz`
- `pose.translation_xyz`
- `motion.angular_velocity_xyz`
- `motion.linear_acceleration_xyz`

## Current Mode

Only `mode1` is implemented now:

- image frame is the anchor
- producer must publish `pose / motion` first, then publish `image`
- consumer arms async pose/motion subscribers before waiting for the next image
- only exact timestamp match is accepted
- any missing or mismatched side data causes an error log and that image is dropped

There is no fallback, nearest-neighbor, or partial output path.

## Constructor Arguments

- `image_topic_name`: input shared image topic name, default `image_frame`
- `pose_topic_name`: input pose topic name, default `camera_pose`
- `motion_topic_name`: input motion topic name, default `camera_motion`
- `output_topic_name`: output shared synced-frame topic name, default `camera_frame_sync`
- `image_wait_timeout_ms`: image wait timeout, default `100`

## Template Arguments

- `Info`: compile-time `CameraTypes::CameraInfo`

## Depends

- `qdu-future/CameraBase`
