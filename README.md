# insta360_ros_driver

A ROS driver for the Insta360 cameras. This driver is tested on Ubuntu 22.04 with ROS2 Humble. The driver has also been verified on the Insta360 X2 and X3 cameras.

## Current Version

This working version is based on upstream branch `humble` at commit `c531aef`
(pulled August 4, 2026). The ROS package version remains `1.0.0`.

| Component | Current setup |
| --- | --- |
| Upstream target | Ubuntu 22.04 / ROS 2 Humble |
| Locally checked toolchain | ROS 2 Jazzy, FFmpeg/libavcodec 60, OpenCV 4.6 |
| CameraSDK | User-supplied SDK released after April 23, 2025 |
| CameraSDK API | `SyncLocalTimeToCamera(uint64_t utc_time, uint32_t offset_time)` |
| Local `libCameraSDK.so` SHA-256 | `e1341d1921b6f207d506df293ab5f33039af1da6bb06843b77a13e1b13b13d01` |
| Video input | H.264 dual-fisheye stream, normally 30 FPS |
| X3 preview modes | Dual-lens 1920×960 (default), or dual-lens 3840×1920 |
| Decoded output | Latest-frame-only at selectable 10 or 30 Hz |
| Timestamp source | CameraSDK frame/gyro timestamp, mapped into ROS time |

The proprietary CameraSDK headers and `libCameraSDK.so` are not tracked by this
repository, and the supplied library does not expose a version string through
the checked headers. The checksum above identifies the library used for this
build; keep the original SDK package with deployment records as well.

See [Current pipeline and timing](docs/current_version.md) for the data path,
buffering policy, timestamp limitations, and configuration details.

The following resolutions are available, all at 30 FPS.

- 3840 x 1920
- 2560 x 1280
- 2304 x 1152
- 1920 x 960

You can change [this line](https://github.com/ai4ce/insta360_ros_driver/blob/79588d9e0e9d029c3371d4095ea718daaf1e06fb/src/main.cpp#L126) to edit the resolution.

## Installation
To use this driver, you need to first have Insta360 SDK. Please apply for the SDK from the [Insta360 website](https://www.insta360.com/sdk/home). 

For additional instructions, see this [post](https://github.com/ai4ce/insta360_ros_driver/issues/10#issuecomment-3371481987).

**Note: Please make you use the latest SDK. This package works with the SDK posted after April 23, 2025**

```
cd ~/ros2_ws/src
git clone -b humble https://github.com/ai4ce/insta360_ros_driver
cd ..
```
Then, the Insta360 libraries need to be installed as follows:
- add the <code>camera</code> and <code>stream</code> header files inside the <code>include</code> directory
- add the <code>libCameraSDK.so</code> library under the <code>lib</code> directory.

Afterwards, install the other required dependencies and build
```
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If Conda is active, deactivate it before building so ROS uses the system Python
and OpenCV. In the locally checked Jazzy workspace, the equivalent explicit
build settings are:

```bash
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

Mixing Conda OpenCV with the OpenCV version used by `cv_bridge` can compile but
fail at link time because the ABIs and dependent libraries differ.

The Insta360 X3 (and potentially other models) needs a micro-sd card inserted to use the API. Ensure this is done.

Before continuing,  **make sure the camera is set to dual-lens mode**

Additionally, **ensure the camera's USB mode is set to Android**:
1. On the camera, swipe down the screen to the main menu
2. Go to Settings -> General
3. Set USB Mode to **Android** (not Webcam or other modes)
4. This is required for the ROS driver to properly detect and communicate with the camera (see [Issue #4](https://github.com/ai4ce/insta360_ros_driver/issues/4))

The Insta360 requires sudo privilege to be accessed via USB. To compensate for this, a udev configuration can be automatically created that will only request for sudo once. The camera can thus be setup initially via:
```
cd ~/ros2_ws/src/insta360_ros_driver
./setup.sh
```
This creates a symlink  based on the vendor ID of Insta360 cameras. The symlink, in this case <code>/dev/insta</code> is used to grant permissions to the usb port used by the camera.

![setup](docs/setup.png)

**Sometimes, this does not work (e.g. you see "device /dev/insta not found" or something similar). You can try entering the commands manually, since that sometimes sees success, especially for the first time.**
```
echo SUBSYSTEM=='"usb"', ATTR{manufacturer}=='"Arashi Vision"', SYMLINK+='"insta"', MODE='"0777"' | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/insta
```

## Usage
The camera provides images as an H.264 compressed stream. The decoder node
continuously decodes that stream, retains only the newest decoded frame, and
publishes it at the configured output rate.

### Camera Bringup
The camera can be brought up with the following launch file
```
ros2 launch insta360_ros_driver bringup.launch.xml
```

The default decoded output rate is 30 Hz. Select the lower-rate mode with:

```bash
ros2 launch insta360_ros_driver bringup.launch.xml publish_rate_hz:=10
```

`publish_rate_hz` accepts `10` or `30`. This controls decoded image
publication; the H.264 stream is still decoded continuously so inter-frame
references remain valid. At each publication tick, only the newest decoded
frame is used. The driver does not replay a frame when no new frame arrived.
![bringup](docs/bringup_rqt.png)

A dual fisheye image will be published.

![dual_fisheye](docs/dual_fisheye.png)

#### Published Topics
- /dual_fisheye/image
- /dual_fisheye/image/compressed
- /equirectangular/image
- /imu/data
- /imu/data_raw

The launch file has the following optional arguments:

- `equirectangular` (default: `false`) enables equirectangular output. Configure
  it in `config/equirectangular.yaml`.
- `decoder` (default: `true`) enables H.264 decoding.
- `publish_rate_hz` (default: `30`; accepted values: `10`, `30`) sets the
  decoded-image output rate.
- `video_resolution` (default: `1920x960`; accepted values: `1920x960`,
  `3840x1920`) selects the decoded image size. The tested X3 firmware returns
  separate square per-lens streams when the SDK is asked for `1920x960`, so the
  driver always captures the known-good combined `3840x1920` stream and scales
  it in the decoder for `1920x960` output. This reduces ROS and inference image
  size, but does not reduce camera transport or H.264 decoding cost. The driver
  also explicitly selects both sensors before starting the stream.
- `imu_filter` (default: `true`) enables the Madgwick orientation filter.

![equirectangular](docs/equirectangular.png)

This uses the [imu_filter_madgwick](https://wiki.ros.org/imu_filter_madgwick) package to approximate orientation from the IMU. Note that by default, we publish `/imu/data_raw` which only contains linear acceleration and angular velocity. The madgwick filter uses this information to publish orientation to `/imu/data`. You can configure the filter in `config/imu_filter.yaml`. 

![IMU](https://github.com/user-attachments/assets/02b50cad-8415-4dde-9014-9ab3a4d415b9)

## Equirectangular Calibration
You can adjust the extrinsic parameters used to improve the equirectangular image. 
```
# Run the camera driver
ros2 run insta360_ros_driver insta360_ros_driver
# Activate image decoding
ros2 run insta360_ros_driver decoder
# Run the equirectangular node in calibration mode
ros2 run insta360_ros_driver equirectangular.py --calibrate
```
This will open an app to adjust the extrinsics. You can press 's' to get the parameters in YAML format. **Note that you need to press 'a' to update the image preview after changing the intrinsics with the GUI**
![Equirectangular Calibration](docs/calibration.png)

Pressing 's' will return the parameters via the terminal. You can copy paste this onto the configuration file as needed. By default, the launch file reads this from `config/equirectangular.yaml`

```
==================================================
CALIBRATION PARAMETERS (YAML FORMAT)
==================================================
equirectangular_node:
  ros__parameters:
    cx_offset: 0.0
    cy_offset: 0.0
    crop_size: 960
    translation: [0.0, 0.0, -0.105]
    rotation_deg: [-0.5, 0.0, 1.1]
    gpu: True
    out_width: 1920
    out_height: 960
==================================================
```

The decoder intentionally drops superseded decoded frames when the publication
rate is lower than the camera rate. If you do not care about live processing,
you can record `/dual_fisheye/image/compressed` and decode it after recording.
```
ros2 bag record /dual_fisheye/image /imu/data_raw
```

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=ai4ce/insta360_ros_driver&type=Date)](https://star-history.com/#ai4ce/insta360_ros_driver&Date)
