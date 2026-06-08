# px4_offboard_control

A ROS 2 Python package for PX4 offboard control and mission execution.

## Overview

`px4_offboard_control` provides two executable nodes:

- **`offboard_control`** – Basic offboard control that publishes heartbeat, trajectory setpoints, and vehicle commands. Demonstrates takeoff, waypoint following, and landing sequences.
- **`offboard_mission_H`** – Extended version with serial communication for coordinating external devices (e.g., robotic actuators) during flight.

Ideal for PX4 + ROS 2 experimentation, trajectory validation, and robot-UAV coordination.

## Requirements

- ROS 2 (with PX4 communication bridge configured)
- `px4_msgs` package
- Python 3, `rclpy`
- `pyserial` (only for `offboard_mission_H`)

## Installation

Place the package in your ROS 2 workspace `src/` directory and build:

```bash
cd ~/ros2_ws
colcon build --packages-select px4_offboard_control
source install/setup.bash
```

For serial support:
```bash
pip install pyserial
```

## Usage

Run the basic offboard control node:
```bash
ros2 run px4_offboard_control offboard_control
```

Or the mission-enabled version with serial I/O:
```bash
ros2 run px4_offboard_control offboard_mission_H
```

## Topics

### offboard_control publishes:
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

### offboard_mission_H additionally subscribes to:
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_status`

And publishes:
- `/robot_state`

## Configuration

`offboard_mission_H` is hardcoded to use serial device `/dev/ttyAMA1` at 115200 baud. Modify the node source code to use a different device or enable configuration via ROS parameters.

## Testing

Run style checks:
```bash
colcon test --packages-select px4_offboard_control
```

Tests include `flake8`, `pep257`, and copyright validation.

## Warnings

- **Safety**: This package controls real UAVs. Test in simulation or low-risk environments first.
- **Trajectory**: Fixed waypoints and control sequences are hardcoded for testing; not production-ready.
- **Hardware**: Serial communication in `offboard_mission_H` is platform-specific and may require modification.

## License

This project is licensed under the BSD 3-Clause License. See the [LICENSE](LICENSE) file for details.
