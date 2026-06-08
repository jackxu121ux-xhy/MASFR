# launch_file

A small ROS 2 launch package for starting the multimodal motion system.

## Overview

`launch_file` contains launch descriptions that start the coordinated multi-robot stack from the `multimodal_motion` package.

The main launch file starts the following nodes:

- `node_crawling`
- `node_px4`
- `manipulation`

It is a thin orchestration layer and does not implement control logic itself.

## Requirements

- ROS 2
- `launch`, `launch_ros`
- The `multimodal_motion` package installed in the same workspace

## Installation

Build the package from your ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select launch_file
source install/setup.bash
```

## Usage

Launch the multi-robot stack:

```bash
ros2 launch launch_file multimodal_motion.launch.py
```

## Contents

- `launch/multimodal_motion.launch.py` – Launches crawling, PX4, and manipulation nodes from `multimodal_motion`

## Configuration

- The launch sequence assumes the `multimodal_motion` package is available.
- The current launch file starts the crawling node first, then starts PX4 and manipulation using process start handlers.

## Testing

Run style checks:

```bash
colcon test --packages-select launch_file
```

## Warnings

- This package only coordinates startup order; all runtime behavior lives in `multimodal_motion`.
- If node names or executable names change, the launch file must be updated accordingly.

## License

This project is licensed under the BSD 3-Clause License. See the [LICENSE](LICENSE) file for details.