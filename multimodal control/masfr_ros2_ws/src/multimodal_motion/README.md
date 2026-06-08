# multimodal_motion

A ROS 2 Python package for coordinating multi-modal locomotion: UAV flight control and hexapod crawler motion.

## Overview

`multimodal_motion` is a coordination system for hybrid aerial-ground robots. It includes three executable nodes:

- **`manipulation`** – Mission orchestrator. Loads mixed motion trajectories from CSV (alternating flight and crawling segments), broadcasts target paths to the appropriate execution nodes, and manages state transitions.
- **`node_crawling`** – Hexapod crawler executor. Subscribes to crawling target paths, uses motion capture feedback for trajectory tracking, and sends serial commands to the ground platform.
- **`node_px4`** – UAV flight executor. Subscribes to flying target paths, publishes PX4 offboard control commands, and maintains coordination via serial I/O with the ground platform.

The package leverages a shared `Trajectory` utility class for trajectory geometry computation and error metrics.

Ideal for research on aerial-ground multi-robot systems, hybrid mobility platforms, and coordinated motion planning.

## Requirements

- ROS 2
- `rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `px4_msgs`
- `scipy`, `pandas`, `numpy` (for math and trajectory management)
- `pyserial` (for serial communication)
- Motion capture system (MOCAP) for ground robot pose feedback
- PX4 autopilot firmware and ROS 2 bridge

## Installation

Place the package in your ROS 2 workspace `src/` directory and build:

```bash
cd ~/ros2_ws
colcon build --packages-select multimodal_motion
source install/setup.bash
```

Install dependencies:
```bash
pip install scipy pandas numpy pyserial
```

## Usage

### Start the mission orchestrator:
```bash
ros2 run multimodal_motion manipulation
```
Loads CSV trajectory file and manages motion segment transitions. CSV format should include `points` (coordinates) and `labels` (motion type: "wf" for flight, "wg" for crawling).

**Note**: Hardcoded CSV path is `~/Desktop/output.csv`. Modify `manipulation.py` to change.

### Start the crawler executor:
```bash
ros2 run multimodal_motion node_crawling
```
Subscribes to `/target_crawl_path`, receives MOCAP pose at `/mocap_pose`, sends serial commands to crawler via `/dev/ttyAMA2` (115200 baud).

### Start the UAV executor:
```bash
ros2 run multimodal_motion node_px4
```
Subscribes to `/target_fly_path`, publishes PX4 offboard control, receives vehicle state from autopilot.

## Topics

### manipulation publishes:
- `/target_fly_path` (Float32MultiArray) – Target trajectory for UAV (4D: x, y, z, yaw)
- `/target_crawl_path` (Float32MultiArray) – Target trajectory for crawler (3D: x, y, z)
- `/kill_all` (Bool) – Termination signal when all motion segments complete

### manipulation subscribes to:
- `/fly_terminate` (Bool) – Signal from UAV node when flight segment ends
- `/crawl_terminate` (Bool) – Signal from crawler node when crawling segment ends

### node_crawling subscribes to:
- `/target_crawl_path` (Float32MultiArray)
- `/mocap_pose` (PoseStamped)
- `/kill_all` (Bool)

### node_crawling publishes:
- `/current_crawl_path` (Path) – Executed trajectory
- `/current_crawl_pose` (PoseStamped) – Robot pose
- `/crawl_terminate` (Bool) – Completion signal

### node_px4 subscribes to:
- `/target_fly_path` (Float32MultiArray)
- `/kill_all` (Bool)
- `/fmu/out/vehicle_local_position` (VehicleLocalPosition)
- `/fmu/out/vehicle_status` (VehicleStatus)

### node_px4 publishes:
- `/fmu/in/offboard_control_mode` (OffboardControlMode)
- `/fmu/in/trajectory_setpoint` (TrajectorySetpoint)
- `/fmu/in/vehicle_command` (VehicleCommand)
- `/fly_terminate` (Bool) – Completion signal

## Serial Protocol

Both crawler and UAV nodes communicate hardware via `/dev/ttyAMA2` (115200 baud) using binary format:

```python
struct.pack('bbbbf', cmd_type, cmd_id, param1, param2, param3)
```

## Configuration

- **CSV trajectory file**: Hardcoded in `manipulation.py` as `~/Desktop/output.csv`. Change path in `load_trajectory_csv()`.
- **Serial ports**: Crawler uses `/dev/ttyAMA2`, UAV uses `/dev/ttyAMA2` for ancillary communication. Modify node source to change.
- **Motion capture**: Assumes MOCAP publishes to `/mocap_pose`. Remap if using a different publisher.
- **Trajectory processing**: CSV labels "wf" (flying) and "wg" (ground/crawling) determine execution routing.

## Testing

Run style checks:
```bash
colcon test --packages-select multimodal_motion
```

## Warnings

- **System complexity**: This is a tightly integrated multi-robot system. Ensure all three nodes start and synchronize properly before flight.
- **CSV dependency**: Mission success depends on correct CSV format and file availability.
- **Synchronization**: Crawler and UAV execution order is dictated by `manipulation`. Ensure both platforms are ready before starting.
- **Motion capture**: Crawler trajectory tracking requires stable MOCAP feedback. Loss of MOCAP will cause trajectory following to fail.
- **Serial coordination**: Both nodes share a single serial port for hardware control. Serialization may cause delays.

## License

This project is licensed under the BSD 3-Clause License. See the [LICENSE](LICENSE) file for details.
