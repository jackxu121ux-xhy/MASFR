# Crawling Control

This directory contains MASFR crawling-control materials for adhesion-sliding locomotion.

## Contents

- `program_stm/` - STM32F405 firmware for the crawler control board. It receives serial commands, drives solenoid valves and pump outputs, and executes staged gait routines.
- Parameter and notes files may be added here for gait timing tables, pressure/vacuum timing, servo command settings, and trajectory-tracking gains.

## ROS 2 Integration

The high-level ROS 2 crawler package lives in `../masfr_ros2_ws/src/robot_crawl/`. It sends packed serial commands to the STM32 firmware over the configured serial device, typically `/dev/ttyAMA1` at 115200 baud.

The multimodal mission package in `../masfr_ros2_ws/src/multimodal_motion/` can also dispatch crawling segments using motion-capture feedback and the same serial command format.

Before running on hardware, verify the serial device path, actuator wiring, pump/valve mapping, and emergency stop procedure.
