import rclpy
import struct
import serial
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path

from robot_crawl.trajectory import Trajectory
import time
import math
from scipy.spatial.transform import Rotation as R

class CrawlPathFollow(Node):
    def __init__(self):
        super().__init__('node_crawl_path_follow')
        self.get_logger().info("Follow path node starts!")
        self.subscriber_pos_cur_ = self.create_subscription(PoseStamped, '/mocap_pose', self.callbackPoseCur, 10)
        self.currentPose = PoseStamped()  #global pose of the robot, received from mocap or something

        self.initialPose = PoseStamped()  #initial pose of the robot
        self.boolInitialPoseLoaded = False
        self.currentPoseRel = PoseStamped()  
        
        #global pose of the robot, relative to initial pose
        #equals to currentPose - initialPose

        self.trajectory = Trajectory()
        self.trajectory.path_ellipse_slope(400, 400)

        self.currentPath = Path()
        self.pubTargetPath = self.create_publisher(Path, '/target_crawl_path', 10)
        self.pubCurrentPath = self.create_publisher(Path, '/current_crawl_path', 10)
        self.pubCurrentPoseRel = self.create_publisher(PoseStamped, '/currentPose', 10)

        self.sendTargetPath()

        self.serRobot = serial.Serial('/dev/ttyAMA1', 115200)
        if self.serRobot.is_open == False:
            self.serRobot.open()
        self.get_logger().info("Serial to robot launches!")

        self.final = False
        self.rate = self.create_rate(0.25)
        self.rate_ms = self.create_rate(10)
        self.timerMainloop = self.create_timer(8.2, self.mainloop)

    def callbackPoseCur(self, msg): 
        if not self.boolInitialPoseLoaded:
            self.boolInitialPoseLoaded = True
            self.initialPose = msg
            self.get_logger().info("Initial Pose Loaded")
        else:
            self.currentPose = msg
            #elf.currentPoseRel = msg
            self.currentPoseRel.header.stamp = self.get_clock().now().to_msg()
            self.currentPoseRel.header.frame_id = "map"

            self.currentPoseRel.pose.position.x = self.currentPose.pose.position.x - self.initialPose.pose.position.x + 1e-6
            self.currentPoseRel.pose.position.y = self.currentPose.pose.position.y - self.initialPose.pose.position.y + 1e-6
            self.currentPoseRel.pose.position.z = self.currentPose.pose.position.z - self.initialPose.pose.position.z
            
            R0 = R.from_euler('xyz', [0, 0, -90], degrees=True)
            q1 = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            q2 = q1 * R0            
            #self.currentPoseRel.pose.orientation.w = math.sqrt(2) * (msg.pose.orientation.w + msg.pose.orientation.z) / 2
            #self.currentPoseRel.pose.orientation.x = math.sqrt(2) * (msg.pose.orientation.x + msg.pose.orientation.y) / 2
            #self.currentPoseRel.pose.orientation.y = math.sqrt(2) * (msg.pose.orientation.y - msg.pose.orientation.x) / 2
            #self.currentPoseRel.pose.orientation.z = math.sqrt(2) * (msg.pose.orientation.z - msg.pose.orientation.w) / 2

            self.currentPoseRel.pose.orientation.w = q2.as_quat()[3]
            self.currentPoseRel.pose.orientation.x = q2.as_quat()[0]
            self.currentPoseRel.pose.orientation.y = q2.as_quat()[1]
            self.currentPoseRel.pose.orientation.z = q2.as_quat()[2]

            self.pubCurrentPoseRel.publish(self.currentPoseRel)

    def robotOneStepForward(self, firstSide, division):  #firstSide = 1 for left, firstSide = -1 for right, division < 1
        msg = struct.pack('bbbbf', 127, 1, firstSide, 0, division)
        self.serRobot.write(msg)
    
    def robotOneStepBackward(self, firstSide, division):  #firstSide = 1 for left, firstSide = -1 for right, division < 1
        msg = struct.pack('bbbbf', 127, -1, firstSide, 0, division)
        self.serRobot.write(msg)

    def robotOneStepTurnLeft(self, division):
        msg = struct.pack('bbbbf', 127, 2, 1, 0, division)
        self.serRobot.write(msg)

    def robotOneStepTurnRight(self, division):
        msg = struct.pack('bbbbf', 127, 3, 1, 0, division)
        self.serRobot.write(msg)

    def sendTargetPath(self):
        msgPath = Path()
        msgPath.header.frame_id = "map"
        msgPath.header.stamp = self.get_clock().now().to_msg()
    
        point = [0, 0, 0]
        
        for point in self.trajectory.points:
            msgPose = PoseStamped()
            msgPose.header.stamp = self.get_clock().now().to_msg()
            msgPose.header.frame_id = "map"

            msgPose.pose.position.x = point[0]
            msgPose.pose.position.y = point[1]
            msgPose.pose.position.z = point[2]
            # must be float! dummy ros2
            msgPath.poses.append(msgPose)

        self.pubTargetPath.publish(msgPath) 

    def mainloop(self):
        if not self.boolInitialPoseLoaded:
            self.rate_ms.sleep()
            return
        if self.final == True:
            return
        self.currentPath.poses.append(self.currentPoseRel)
        self.currentPath.header.frame_id = "map"
        self.currentPath.header.stamp = self.get_clock().now().to_msg()
        self.pubCurrentPath.publish(self.currentPath)
        self.sendTargetPath()

        currPos = [self.currentPoseRel.pose.position.x, self.currentPoseRel.pose.position.y, self.currentPoseRel.pose.position.z,
                   self.currentPoseRel.pose.orientation.w, self.currentPoseRel.pose.orientation.x, self.currentPoseRel.pose.orientation.y,
                   self.currentPoseRel.pose.orientation.z]
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

        #if(self.trajectory.nearest_point >= self.trajectory.n_points - 50):
        #    self.final = True
    
def main():
    rclpy.init()
    node = CrawlPathFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()