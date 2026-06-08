#!/usr/bin/python3
# receiver_node.py
import socket
import rclpy
import struct
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PoseStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry, VehicleGlobalPosition, VehicleCommand

class MoCapReceiver(Node):
    def __init__(self):
        super().__init__('mocap_receiver')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(PoseStamped, 'mocap_pose', 10)
        self.publisher_px4_ = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile)
        self.publisher_global_ = self.create_publisher(
            VehicleGlobalPosition, '/fmu/in/vehicle_global_position', qos_profile)
        self.publisher_command_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        #self.publisher_mavros = self.create_publisher(Mavlink, '')
        # 创建TCP socket
        self.host = '192.168.1.100'
        #rebuild the package after modifying!
        self.port = 9045
        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_client.connect((self.host, self.port))
        #self.socket_client.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64)
        self.get_logger().info(f"Connect to MoCap server: {self.port}")

        self.bHeader = bytes.fromhex('FFFF')
        
        #fake a global position for px4 to unlock
        #global_msg_px4 = VehicleGlobalPosition(lat=40, lon=116, alt=43.5)
        #global_msg_px4.timestamp_sample = self.get_clock().now().seconds_nanoseconds()
        com_set_gps_origin = VehicleCommand(command=100000, param1=40.1, param2=116.25, param3=43.5)
        #com_set_gps_origin.param1(40)
        #com_set_gps_origin.param2(116)
        #com_set_gps_origin.param3(43.5)
        self.publisher_command_.publish(com_set_gps_origin)
        while rclpy.ok():
            #self.publisher_global_.publish(global_msg_px4)
            try:
                while True:
                    header1 = self.socket_client.recv(1)
                    if header1 == b'\xff':
                        header2 = self.socket_client.recv(1)    
                        if header2 == b'\xff':
                            break

                data = self.socket_client.recv(56)

                while len(data) < 56:
                    tmpData = self.socket_client.recv(56 - len(data))
                    data = data + tmpData

                # 解析数据
                x, y, z, qx, qy, qz, qw = struct.unpack('ddddddd', data)

                # 创建PoseStamped消息
                #mavlink_msg = Mavlink()
                # mavlink_msg.payload64
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = z
                pose_msg.pose.orientation.x = qx
                pose_msg.pose.orientation.y = qy
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw

                pose_msg_px4 = VehicleOdometry()
                pose_msg_px4.pose_frame = VehicleOdometry.POSE_FRAME_NED
                pose_msg_px4.position[0] = x / 1000
                pose_msg_px4.position[1] = -y / 1000
                pose_msg_px4.position[2] = -z / 1000

                pose_msg_px4.q[0] = qw
                pose_msg_px4.q[1] = qx
                pose_msg_px4.q[2] = -qy
                pose_msg_px4.q[3] = -qz
                # 发布消息
                self.publisher_px4_.publish(pose_msg_px4)
                
                self.publisher_.publish(pose_msg)
                self.get_logger().info('Pose message published')

            except Exception as e:
                self.get_logger().error(f"Error receiving data: {e}")

def main(args=None):
    rclpy.init(args=args)
    mocap_receiver = MoCapReceiver()
    rclpy.spin(mocap_receiver)
    mocap_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()