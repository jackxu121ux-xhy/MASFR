# mocap_receiver

A ROS 2 Python package for receiving motion capture data and forwarding it to ROS 2 and PX4 topics.

## Overview

`mocap_receiver` connects to a motion capture server over TCP, receives 3D pose data, and republishes it as ROS 2 messages. It also forwards a PX4-compatible visual odometry stream and publishes a startup command used to initialize the PX4 side.

This package is useful when a motion capture system is used as the main pose source for a robot, UAV, or hybrid platform.

## Requirements

- ROS 2
- `rclpy`, `geometry_msgs`, `px4_msgs`
- A motion capture server that streams pose data over TCP
- PX4 bridge configured to accept visual odometry input

## Installation

Place the package in your ROS 2 workspace `src/` directory and build:

```bash
cd ~/ros2_ws
colcon build --packages-select mocap_receiver
source install/setup.bash
```

## Usage

Run the receiver node:

```bash
ros2 run mocap_receiver mocap_node
```

The node connects to the mocap server at the address defined in `mocap_node.py`, parses pose frames, and publishes:

- `mocap_pose`
- `/fmu/in/vehicle_visual_odometry`
- `/fmu/in/vehicle_global_position`
- `/fmu/in/vehicle_command`

## Configuration

- **Mocap server**: hardcoded in `mocap_node.py` as a TCP host and port.
- **Frame format**: the node expects a `0xFFFF` header followed by seven `double` values: `x, y, z, qx, qy, qz, qw`.
- **PX4 initialization**: the node publishes an initial `VehicleCommand` used to set a GPS origin-like reference.

## Testing

Run style checks:

```bash
colcon test --packages-select mocap_receiver
```

## Warnings

- This package depends on a running mocap server with the expected binary protocol.
- The TCP host, port, and message format are platform-specific and may need adjustment.
- The node runs continuously and should be launched only after the mocap service is available.

## License

This project is licensed under the BSD 3-Clause License. See the [LICENSE](LICENSE) file for details.