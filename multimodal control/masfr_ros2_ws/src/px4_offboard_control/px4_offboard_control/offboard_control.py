#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus


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
        self.vehicle_local_position_publisher_rviz = self.create_publisher(
            VehicleCommand, '/fmu/out/vehicle_local_position/rviz', qos_profile)

        # Create subscribers
        #self.vehicle_local_position_subscriber = self.create_subscription(
        #    VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        #self.vehicle_status_subscriber = self.create_subscription(
        #    VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -0.5
        self.trajSetpoint = [0, 0, 0, 0]

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        #self.timer_setpoint = self.create_timer(0.1, self.timer_setpoint_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

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
        msg.yaw = yaw  # (90 degree)
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
    #def timer_setpoint_callback(self):

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        
        #if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #print(self.vehicle_status.nav_state)
        #print(self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z)
        
        #self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z - 0.1, 3.14)
        #if self.offboard_setpoint_counter == 10:
        #    self.engage_offboard_mode()

        if self.offboard_setpoint_counter <= 30:

            print('Arming the vehicle')
            #self.flight_termination(0.0)
            self.arm()
        elif self.offboard_setpoint_counter <= 100:
            self.publish_position_setpoint(0.0, 0.0, -0.6, -1.57)
        elif self.offboard_setpoint_counter <= 257:
            t = (self.offboard_setpoint_counter - 100) / 25
            x = -0.5 + 0.5 * math.cos(t)
            y = -0.5 * math.sin(t)
            yaw = - 1.57 - t # not limited in range [-pi, pi]
            self.publish_position_setpoint(x, y, -0.6, yaw)
        elif self.offboard_setpoint_counter <= 300:
            self.publish_position_setpoint(0.0, 0.0, -0.6, -1.57)
        elif self.offboard_setpoint_counter <= 340:
            print('Engaging position mode')
            self.land()
        elif self.offboard_setpoint_counter <= 350:
            print('Disarming the vehicle')
            self.disarm()
        '''if self.offboard_setpoint_counter <= 80:
            self.publish_position_setpoint(0.0, -0.5, -0.1, 3.14)
        elif self.offboard_setpoint_counter <= 130:
            self.publish_position_setpoint(-1.0, -0.5, -0.1, -1.57)
        elif self.offboard_setpoint_counter <= 180:
            self.publish_position_setpoint(-1.0, 0.5, -0.1, 0.0)
        elif self.offboard_setpoint_counter <= 230:
            self.publish_position_setpoint(0.0, 0.5, -0.1, 1.57)
        elif self.offboard_setpoint_counter <= 280:
            self.publish_position_setpoint(-0.0, -0.0, 0.4, 3.14)'''

        '''if self.offboard_setpoint_counter == 258:
            print('Engaging position mode')
            self.land()

        if self.offboard_setpoint_counter == 280:
            print('Disarming the vehicle')
            self.disarm()'''

        if self.offboard_setpoint_counter < 351:
            self.offboard_setpoint_counter += 1
        else:
            exit(0)


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
