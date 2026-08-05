# Current Pipeline and Timing

This document describes the repository state based on upstream `humble` commit
`c531aef` (pulled August 4, 2026) plus the low-latency timestamp work on branch
`low-latency-capture-timestamps`.

## Supported setup

- Upstream target: Ubuntu 22.04 and ROS 2 Humble.
- Toolchain found in this workspace: ROS 2 Jazzy, FFmpeg/libavcodec 60.31.102,
  and OpenCV 4.6.0. The decoder includes the repository's existing FFmpeg 6 and
  Jazzy `cv_bridge` compatibility adjustments.
- Package version: `1.0.0` from `package.xml`.
- Verified upstream cameras: Insta360 X2 and X3.
- Camera mode: dual-lens mode with USB mode set to Android.
- X3: a microSD card may be required for the camera API.
- CameraSDK: release after April 23, 2025, using the two-argument
  `SyncLocalTimeToCamera` API.
- Audited `libCameraSDK.so`: 11,518,472 bytes, SHA-256
  `e1341d1921b6f207d506df293ab5f33039af1da6bb06843b77a13e1b13b13d01`.
- Decoder: system FFmpeg/libavcodec, with NVIDIA NVDEC attempted first and the
  FFmpeg software H.264 decoder used as fallback.

Conda was active in the audited workspace and initially caused CMake to select
Conda Python 3.13 and OpenCV 4.12. ROS Jazzy uses system Python and its
`cv_bridge` package uses system OpenCV 4.6. Deactivate Conda before building or
select the compatible dependencies explicitly:

```bash
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

The CameraSDK headers and shared library are supplied by the user under
`include/camera`, `include/stream`, and `lib/libCameraSDK.so`. They are ignored
by Git. The checked SDK API and shared library do not provide a reliable SDK
release version string, so deployments should archive the original SDK package.

## Image pipeline

```text
Insta360 CameraSDK
  -> /dual_fisheye/image/compressed (H.264)
  -> FFmpeg decoder (decodes the continuous stream)
  -> one replaceable latest-frame slot
  -> /dual_fisheye/image (10 or 30 Hz)
  -> optional equirectangular node
  -> /equirectangular/image
```

H.264 input is decoded continuously even in 10 Hz mode. Dropping arbitrary
compressed frames would break inter-frame references. Rate limiting happens
after decode: every decoded frame replaces the pending frame, and the publisher
takes only the newest pending frame at each tick. A frame is not published
twice when no new decoded frame is available.

The raw decoded-image publisher uses reliable QoS with depth one. The compressed
input subscription retains its existing depth because it carries the stateful
H.264 stream. The equirectangular C++ subscriber and publisher already use depth
one and preserve the incoming image header.

### X3 dual-lens resolution selection

CameraSDK treats preview resolution and active-sensor selection as independent
settings. The camera can retain `SENSOR_DEVICE_FRONT` or `SENSOR_DEVICE_REAR`
from an earlier mode. Before every preview, the driver calls
`SetActiveSensor(SENSOR_DEVICE_ALL)` and fails startup if panoramic mode cannot
be selected.

On the tested X3 firmware, requesting `RES_1920_960P30` after selecting both
sensors yields two separate 2880×2880 H.264 streams, one for each lens. The SDK
reports them through `stream_index` 0 and 1; the original driver only consumes
index 0, which explains the apparent single-lens result. The X3's
`RES_3840_1920P30` path instead supplies the combined dual-fisheye canvas the
driver expects.

The `video_resolution` launch argument accepts:

- `1920x960` (default): capture the combined 3840×1920 stream and downscale it
  directly during FFmpeg pixel conversion.
- `3840x1920`: publish the combined stream at its native dimensions.

Both modes use the main stream (`using_lrv=false`) and contain both fisheye
lenses. The 1920×960 option reduces ROS bandwidth, copies, and downstream model
input size, but it does not reduce USB transport or H.264 decode work because
the camera source remains 3840×1920. Unsupported strings fail explicitly rather
than silently selecting an unexpected SDK mode.

The camera node also waits up to three seconds for a compressed-image
subscriber before starting the stream. This prevents the decoder from missing
the H.264 sequence/picture parameter sets sent at startup.

## Output-rate configuration

The launch argument and decoder parameter are both named `publish_rate_hz`.
Accepted values are `10` and `30`; invalid values produce a warning and
fall back to 30 Hz.

```bash
# Low-rate, latest-frame mode
ros2 launch insta360_ros_driver bringup.launch.xml publish_rate_hz:=10

# Full camera-rate mode (default)
ros2 launch insta360_ros_driver bringup.launch.xml publish_rate_hz:=30
```

The older `skip_frame` and `i_frame_only` decoder parameters remain for
compatibility. `publish_rate_hz` is preferred for normal 10/30 Hz operation.

## Timestamp behavior

The CameraSDK supplies video and gyro timestamps in milliseconds relative to
camera startup. They are not Unix/ROS timestamps. The driver maps that camera
clock into the active ROS clock using observations made when SDK callbacks
arrive:

```text
candidate offset = ROS callback time - camera timestamp
capture stamp    = camera timestamp + minimum observed offset
```

Using the minimum observed offset removes variable delay caused by USB, SDK, and
callback buffering. It cannot identify an unknown fixed camera/transport delay,
because the SDK does not expose the camera boot instant in the host clock
domain. For applications requiring a calibrated absolute exposure time, measure
that fixed offset for the deployed camera/host combination.

The compressed-frame ROS stamp is passed into FFmpeg as packet PTS. The decoder
uses each decoded frame's `best_effort_timestamp`, so H.264 decoder delay or
frame reordering does not replace the capture stamp with publication time.
Equirectangular output copies the decoded image header unchanged.

Gyroscope batches use the same clock mapper. Each IMU message uses its own SDK
timestamp instead of assigning one callback time to the entire batch, preserving
sample spacing and image/IMU alignment.

The SDK does not document whether the video timestamp represents exposure start,
exposure midpoint, or another point in the frame interval. The driver therefore
treats it as the device frame timestamp. Precise visual-inertial calibration may
also need the SDK exposure callback and a measured temporal offset.
