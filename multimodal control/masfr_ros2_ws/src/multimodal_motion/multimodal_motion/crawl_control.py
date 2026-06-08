import rclpy
import struct
import serial
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, Bool
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path

from .trajectory import Trajectory
import time
import math
from scipy.spatial.transform import Rotation as R
import numpy as np

class CrawlPathFollow(Node):
    def __init__(self):
        super().__init__('node_crawling')
        self.get_logger().info("Crawling node starts!")
        self.subscriber_pos_cur_ = self.create_subscription(PoseStamped, '/mocap_pose', self.callbackPoseCur, 10)
        self.currentPose = PoseStamped()  #global pose of the robot, received from mocap or something 

        self.trajectory = Trajectory()
        self.currentPath = Path()
        
        self.crawl_path_subscriber = self.create_subscription(Float32MultiArray, '/target_crawl_path', self.target_path_callback, 10)
        self.kill_all_subscriber = self.create_subscription(Bool, '/kill_all', self.kill_all_callback, 10)
        self.current_path_publisher = self.create_publisher(Path, '/current_crawl_path', 10)
        self.current_pose_publisher = self.create_publisher(PoseStamped, '/current_crawl_pose', 10)

        # send True signal to the topic to inform the manipulator node the crawling state terminates 
        self.crawl_final_publisher = self.create_publisher(Bool, '/crawl_terminate', 10)

        self.serRobot = serial.Serial('/dev/ttyAMA2', 115200)
        if self.serRobot.is_open == False:
            self.serRobot.open()
        self.get_logger().info("Serial to robot launches!")

        self.begin = False

        self.rate = self.create_rate(0.25)
        self.rate_ms = self.create_rate(10)
        self.timerMainloop = self.create_timer(8.2, self.mainloop)

        self.channel1_on = False
        self.channel2_on = False
        self.channel3_on = False
        self.channel4_on = False

    def target_path_callback(self, msg:Float32MultiArray):
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size

        points = []
        for i in range(rows):
            row = msg.data[i * cols: (i + 1) * cols]  # to millimeter
            row = np.asarray(row)
            row = row * 100
            points.append(row.tolist())
        self.get_logger().info("Crawling target path received")
        self.trajectory.points = points
        self.begin = True
        print(self.trajectory.points)


    def kill_all_callback(self, msg):
        if msg:
            exit(0)

    def callbackPoseCur(self, msg): 
            
        self.currentPose.header.stamp = self.get_clock().now().to_msg()
        self.currentPose.header.frame_id = "map"

        self.currentPose.pose.position.x = msg.pose.position.x
        self.currentPose.pose.position.y = msg.pose.position.y
        self.currentPose.pose.position.z = msg.pose.position.z
        
        R0 = R.from_euler('xyz', [0, 0, -90], degrees=True)
        q1 = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        q2 = q1 * R0

        self.currentPose.pose.orientation.w = q2.as_quat()[3]
        self.currentPose.pose.orientation.x = q2.as_quat()[0]
        self.currentPose.pose.orientation.y = q2.as_quat()[1]
        self.currentPose.pose.orientation.z = q2.as_quat()[2]

        self.current_pose_publisher.publish(self.currentPose)

    def robotOneStepForward(self, firstSide, division):  #firstSide = 1 for left, firstSide = -1 for right, division < 1
        msg = struct.pack('bbbbf', 127, 1, firstSide, 0, division)

        if firstSide == 1:
            self.channel1_on = False
            self.channel2_on = False
            self.channel3_on = True
            self.channel4_on = True
        else:
            self.channel1_on = True
            self.channel2_on = True
            self.channel3_on = False
            self.channel4_on = False

        self.serRobot.write(msg)
    
    def robotOneStepBackward(self, firstSide, division):  #firstSide = 1 for left, firstSide = -1 for right, division < 1
        msg = struct.pack('bbbbf', 127, -1, firstSide, 0, division)
        self.serRobot.write(msg)

    def robotOneStepTurnLeft(self, division):
        msg = struct.pack('bbbbf', 127, 2, 1, 0, division)
        self.serRobot.write(msg)

        self.channel1_on = False
        self.channel2_on = True
        self.channel3_on = True
        self.channel4_on = False

    def robotOneStepTurnRight(self, division):
        msg = struct.pack('bbbbf', 127, 3, 1, 0, division)
        self.serRobot.write(msg)

        self.channel1_on = False
        self.channel2_on = True
        self.channel3_on = True
        self.channel4_on = False

    def mainloop(self):
        if not self.begin:
            return

        self.currentPath.poses.append(self.currentPose)
        self.currentPath.header.frame_id = "map"
        self.currentPath.header.stamp = self.get_clock().now().to_msg()
        self.current_path_publisher.publish(self.currentPath)

        currPos = [self.currentPose.pose.position.x, self.currentPose.pose.position.y, self.currentPose.pose.position.z,
                   self.currentPose.pose.orientation.w, self.currentPose.pose.orientation.x, self.currentPose.pose.orientation.y,
                   self.currentPose.pose.orientation.z]
        print("current pos is:")
        print(currPos)
        side = self.trajectory.get_which_side(currPos)
        print(f"side: {side}")
        theta_error = self.trajectory.get_theta_error(currPos)
        print(f"theta error: {theta_error}")
        distance_error = self.trajectory.get_distance_error(currPos)
        print(f"distance error: {distance_error}")
        
        side_tangent = self.trajectory.get_which_side_tangent(currPos)
        print(f"head side: {side_tangent}")
        #control_rotate = 0.8 if theta_error > 0.5 else 1.773 * theta_error
        
        if side_tangent * side < 0:
            control_rotate = 0
            control_linear = 0.5
        else:
            if theta_error > 0.5:
                control_rotate = 0.8
            elif theta_error < 0.08:
                control_rotate = 0.8 if distance_error > 0.35 else 2.322 * distance_error        
                #control_rotate = 0.8 if distance_error > 0.035 else 23.22 * distance_error        
            else:
                control_rotate = 1.773 * theta_error
                #control_rotate = 1.773 * theta_error
            
            control_linear = 0.5 - 0.5 * control_rotate
        print(f"control rotate: {control_rotate}, control linear: {control_linear}")    
        if side == 1:
            self.robotOneStepTurnRight(control_rotate)
            #self.rate.sleep()
            time.sleep(4)
            self.robotOneStepForward(1, control_linear)
            time.sleep(4)
        else:
            self.robotOneStepTurnLeft(control_rotate)
            time.sleep(4)
            self.robotOneStepForward(-1, control_linear)
            time.sleep(4)

        if(self.trajectory.nearest_point >= self.trajectory.n_points - 5):
            # crawling process terminates
            # switch off the valves
            if self.channel1_on:
                msg_ser = struct.pack('bbbbf', 2, 1, 0, 0, 0)
                self.serRobot.write(msg_ser)
                self.channel1_on = False
            if self.channel2_on:
                msg_ser = struct.pack('bbbbf', 2, 2, 0, 0, 0)
                self.serRobot.write(msg_ser)
                self.channel2_on = False
            if self.channel3_on:
                msg_ser = struct.pack('bbbbf', 2, 3, 0, 0, 0)
                self.serRobot.write(msg_ser)
                self.channel3_on = False
            if self.channel4_on:
                msg_ser = struct.pack('bbbbf', 2, 4, 0, 0, 0)
                self.serRobot.write(msg_ser)
                self.channel4_on = False

            # send terminate signal to manipulation node
            terminate_signal = Bool()
            terminate_signal.data = True
            self.crawl_final_publisher.publish(terminate_signal)

            # set the bit to false
            self.begin = False

            self.get_logger().info("Crawling terminates")
            
    
def main():
    rclpy.init()
    node = CrawlPathFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()