# program_stm

STM32F405-based firmware project for the crawler control board.

This project provides the low-level firmware that drives the robot's actuators and gait logic. It is stored under the MASFR crawling-control materials and is intended to work together with `robotSerial.py` from the ROS 2 `robot_crawl` package, which sends commands over UART from a keyboard or ROS 2 node to the MCU.

## What it does

- Receives 8-byte serial commands on `USART1` at 115200 baud.
- Switches the LED, solenoid valves, and pump output.
- Executes crawler motion routines such as forward, backward, left, right, and continuous forward walking.
- Initializes the servo/actuator state at startup and keeps the robot in a known pose before motion commands are processed.

## How it works with `robotSerial.py`

`robotSerial.py` is the operator-side client. It packs control commands and writes them to the serial port. The STM32 firmware receives those packets in `Core/Src/main.c`, parses them in the UART receive callback, and calls the motion helpers in `Hardware/robotMotion/robotMotion.c`.

Typical command groups are:

- `0x01` - LED toggle
- `0x02` - solenoid valve toggle
- `0x03` - pump toggle
- `0x7F` - gait command

For gait commands, the packet carries:

- `cmd_id` - motion type
- `param1` - first stepping side
- `param2` - step scale / division factor

The keyboard client maps keys like `w/q/s/e`, `a/d`, `1-4`, `0`, and `l` to those packet types.

## Main motion routines

- `robotOneStepForward()` - single forward step
- `robotOneStepBackward()` - single backward step
- `robotOneStepLeft()` - single lateral step to the left
- `robotOneStepRight()` - single lateral step to the right
- `robotContinuousForward()` - repeated forward stepping until a stop flag is received

These routines coordinate the solenoid valves and servo positions in a staged sequence.

## Directory overview

- `Core/` - application source, startup code, interrupt handlers, and MCU initialization.
- `Hardware/` - custom hardware drivers and motion code.
- `Drivers/` - STM32 HAL and CMSIS vendor code.
- `control_circuit.ioc` - CubeMX project configuration.
- `MDK-ARM/` - Keil MDK project files.

## Build notes

The project was generated for STM32CubeMX and Keil MDK. A typical workflow is:

1. Open `control_circuit.ioc` in CubeMX if you need to regenerate code.
2. Open `MDK-ARM/control_circuit.uvprojx` in Keil MDK.
3. Build the target and flash the generated firmware to the board.

## File policy for open source

The repository keeps source and project configuration files, but excludes local IDE settings, debug views, and build outputs. Those files are either ignored or stored in `_local_ignored/` for local recovery.

## Safety

- Verify the correct serial device before connecting hardware.
- Confirm the board is powered and the actuator wiring matches the firmware pins.
- Test on a bench or with power cut-off ready before running motion commands.

## License

See the repository root license and any vendor license files included under `Drivers/`.
