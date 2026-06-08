#!/usr/bin/env python3

import rclpy
import serial
import struct
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int16, Float32MultiArray, Bool
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from nav_msgs.msg import Path

from scipy.spatial.transform import Rotation as R


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1024
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create px4 subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        # Create robot subscribers
        self.fly_path_subscriber = self.create_subscription(Float32MultiArray, '/target_fly_path', self.fly_path_callback, 10)
        self.kill_all_subscriber = self.create_subscription(Bool, '/kill_all', self.kill_all_callback, 10)
        # send True signal to the topic to inform the manipulator node the flying state terminates 
        self.fly_final_publisher = self.create_publisher(Bool, '/fly_terminate', 10)

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.target_fly_path = []
        self.begin = False
        self.is_taken_off = False

        self.mainloop_counter = 0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer_mainloop = self.create_timer(1, self.mainloop)


        self.serRobot = serial.Serial('/dev/ttyAMA2', 115200)
        if self.serRobot.is_open == False:
            self.serRobot.open()

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
    
    def robot_state_callback(self, robot_status):
        self.robot_status = robot_status

    def fly_path_callback(self, msg):

        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size

        points = []
        for i in range(rows):
            row = msg.data[i * cols: (i + 1) * cols]
            row = np.asarray(row)
            row = row / 10
            points.append(row.tolist())
    
        self.target_fly_path = points
        print(self.target_fly_path)
        self.begin = True

    def kill_all_callback(self, msg):
        if msg:
            exit(0)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def takeoff(self):
        for i in range(50):
            print('Arming the vehicle')
            self.flight_termination(0.0)
            self.arm()
            time.sleep(0.1)
            

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def flight_termination(self, param1):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION, param1=param1)
        self.get_logger().info('kill command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def engage_position_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=3.0)
        self.get_logger().info("Switching to position mode")

    def engage_height_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=2.0)
        self.get_logger().info("Switching to height mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        #msg.param3 = params.get("param3", 0.0)
        #msg.param4 = params.get("param4", 0.0)
        #msg.param5 = params.get("param5", 0.0)
        #msg.param6 = params.get("param6", 0.0)
        #msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.confirmation = 0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

    def mainloop(self):

        if not self.begin:
            return

        if not self.is_taken_off:
            self.takeoff()
            self.is_taken_off = True
        
        if self.mainloop_counter < len(self.target_fly_path):
            self.publish_position_setpoint(self.target_fly_path[self.mainloop_counter][0], self.target_fly_path[self.mainloop_counter][1],
                                           self.target_fly_path[self.mainloop_counter][2], self.target_fly_path[self.mainloop_counter][3])
        else:
            msg = struct.pack('bbbbf', 2, 1, 0, 0, 0)
            self.serRobot.write(msg)
            msg = struct.pack('bbbbf', 2, 2, 0, 0, 0)
            self.serRobot.write(msg)
            msg = struct.pack('bbbbf', 2, 3, 0, 0, 0)
            self.serRobot.write(msg)
            msg = struct.pack('bbbbf', 2, 4, 0, 0, 0)
            self.serRobot.write(msg)

            self.land()
            self.begin = False
            self.is_taken_off = False

            terminate_signal = Bool()
            terminate_signal.data = True
            self.fly_final_publisher.publish(terminate_signal)

            self.mainloop_counter = 0
            return

        self.mainloop_counter = self.mainloop_counter + 1

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
