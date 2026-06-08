# Multimodal Control

This directory collects the MASFR execution software and embedded crawling firmware.

## Contents

- `masfr_ros2_ws/` - ROS 2 workspace for PX4 offboard control, crawler serial control, motion-capture feedback, VINS sensor bridging, launch orchestration, and multimodal mission coordination.
- `crawling/program_stm/` - STM32F405 firmware for the crawler control board. It receives serial packets from the ROS 2 crawling client and drives actuator, solenoid, pump, initialization, and gait logic.
- `crawling/README.md` - crawling-control index and hardware safety notes.

## Usage Notes

The ROS 2 workspace keeps the high-level execution nodes. The STM32 firmware in `crawling/program_stm/` provides the low-level actuator, solenoid, pump, and gait logic used by the crawler. The two layers communicate through the serial protocol documented in the `robot_crawl` and `program_stm` README files.

Because this path contains a space, shell examples should quote it:

```bash
cd "multimodal control/masfr_ros2_ws"
```
