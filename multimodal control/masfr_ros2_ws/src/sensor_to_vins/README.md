# sensor_to_vins

A ROS 2 C++ package for forwarding camera and IMU data to VINS-compatible topics.

## Overview

`sensor_to_vins` contains two executables:

- **`img_to_vins`** – Reads a stereo camera stream, splits it into left and right images, and publishes them to `/cam0/image_raw` and `/cam1/image_raw`.
- **`imu_to_vins`** – Subscribes to PX4 sensor data and republishes it as a ROS 2 IMU message on `/imu0`.

This package is intended for visual-inertial odometry pipelines that expect standard ROS image and IMU topics.

## Requirements

- ROS 2
- `rclcpp`, `sensor_msgs`, `cv_bridge`
- OpenCV
- PX4 messages and PX4 ROS bridge for IMU input

## Installation

Build the package from your ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select sensor_to_vins
source install/setup.bash
```

## Usage

Run the stereo image bridge:

```bash
ros2 run sensor_to_vins img_to_vins
```

Run the PX4 IMU bridge:

```bash
ros2 run sensor_to_vins imu_to_vins
```

## Topics

### `img_to_vins`
Publishes:

- `/cam0/image_raw`
- `/cam1/image_raw`

### `imu_to_vins`
Subscribes to:

- `/fmu/out/sensor_combined`

Publishes:

- `/imu0`

## Configuration

- **Camera device**: `img_to_vins.cpp` opens camera index `0` and expects a 2560x720 stereo frame.
- **Image split**: left and right images are split evenly into two 1280x720 frames.
- **IMU source**: `imu_to_vins.cpp` uses PX4 sensor data from `/fmu/out/sensor_combined`.

## Testing

Run lint checks:

```bash
colcon test --packages-select sensor_to_vins
```

## Warnings

- The camera index and image resolution are hardcoded and may need adjustment for your hardware.
- `img_to_vins` publishes from a blocking loop inside the node constructor; this is suitable for a simple bridge but not for more complex camera management.
- `imu_to_vins` forwards raw PX4 IMU values and does not perform additional filtering.

## License

This project is licensed under the BSD 3-Clause License. See the [LICENSE](LICENSE) file for details.