# robot_crawl

ROS 2 package for crawler motion control and trajectory following.

## Overview

This package provides two main nodes:

- `crawl_from_rc`: RC-based teleoperation via serial commands.
- `crawl_path_follow`: motion-capture-based path tracking.

It also includes helper modules such as `crawl_to_H.py` and `trajectory_2d.py`.

## Works with `robotSerial.py`

`robotSerial.py` is the keyboard client for the STM32 firmware stored at `multimodal control/crawling/program_stm/` in the MASFR repository.

- It sends 8-byte packets over `/dev/ttyAMA1` at 115200 baud.
- The firmware receives them in `main.c` and executes motion logic in `robotMotion.c`.
- Supported actions include LED toggle, solenoid toggle, pump toggle, and walking commands.

Typical key mappings:

- `w/q/s/e` - forward/backward steps
- `a/d` - left/right steps
- `1` to `4` - solenoid valves
- `0` - pump toggle
- `l` - LED toggle

## Serial Protocol

Commands use a packed binary layout:

```python
struct.pack('bbbbf', cmd_type, cmd_id, param1, param2, param3)
```

Common gait command example:

- `(127, 8, 1, 0, 0.8)` - continuous forward
- `(127, 2, 1, 0, 0.5)` - turn left
- `(127, 9, 1, 0, 0)` - stop continuous motion

## Usage

```bash
ros2 run robot_crawl crawl_from_rc
ros2 run robot_crawl crawl_path_follow
```

`crawl_from_rc` listens to `/fmu/out/rc_channels`. `crawl_path_follow` uses `/mocap_pose` and publishes the current/target path topics.

## Notes

- Default serial device: `/dev/ttyAMA1`.
- `crawl_path_follow` requires motion capture input.
- Before running on hardware, confirm the serial port, baudrate, and emergency stop path.

## License

See the repository license and the vendor license files under `Drivers/`.
