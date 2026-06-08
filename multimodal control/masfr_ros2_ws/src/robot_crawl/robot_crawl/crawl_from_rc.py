import rclpy
import struct
import serial
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import RcChannels


class CrawlFromRC(Node):
    def __init__(self):
        super().__init__('node_crawl_from_rc')
        self.get_logger().info("Robot crawls following the RC instruction!")
        
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscriber_rc = self.create_subscription(RcChannels,
                                                      '/fmu/out/rc_channels',
                                                      self.rc_callback,
                                                      qos_profile)
        self.advancing = False
        self.turning = False  
        self.ser = serial.Serial('/dev/ttyAMA1', 115200)
        
        self.channel1_on = False
        self.channel2_on = False
        self.channel3_on = False
        self.channel4_on = False
        self.pump_on = True
        
        self.last_channel7 = -1
        self.pump_switch_time = 0

        if self.ser.is_open == False:
            self.ser.open()  

    def rc_callback(self, msg):
        #print(msg.channels)
        instr_side = msg.channels[5]
        instr_forward = msg.channels[6]
        instr_turn = msg.channels[7]
        instr_kill = msg.channels[9]  
        # 1 refers to killed, crawling available
        # -1 refers 'ready to fly', can't crawl
        
        if instr_kill < 0:
            
            if self.pump_switch_time % 2 == 0:
                if self.pump_switch_time == 0:
                    if self.channel1_on:
                        msg_ser = struct.pack('bbbbf', 2, 1, 0, 0, 0)
                        self.ser.write(msg_ser)
                        self.channel1_on = False
                    if self.channel2_on:
                        msg_ser = struct.pack('bbbbf', 2, 2, 0, 0, 0)
                        self.ser.write(msg_ser)
                        self.channel2_on = False
                    if self.channel3_on:
                        msg_ser = struct.pack('bbbbf', 2, 3, 0, 0, 0)
                        self.ser.write(msg_ser)
                        self.channel3_on = False
                    if self.channel4_on:
                        msg_ser = struct.pack('bbbbf', 2, 4, 0, 0, 0)
                        self.ser.write(msg_ser)
                        self.channel4_on = False
                else:
                    if self.channel1_on:
                        msg_ser = struct.pack('bbbbf', 2, 1, 0, 0, 0)
                        self.ser.write(msg_ser)
                        self.channel1_on = False
                    if self.channel4_on:
                        msg_ser = struct.pack('bbbbf', 2, 4, 0, 0, 0)
                        self.ser.write(msg_ser)
                        self.channel4_on = False
            else:
                if not self.channel1_on:
                    msg_ser = struct.pack('bbbbf', 2, 1, 0, 0, 0)
                    self.ser.write(msg_ser)
                    self.channel1_on = True
                if not self.channel4_on:
                    msg_ser = struct.pack('bbbbf', 2, 4, 0, 0, 0)
                    self.ser.write(msg_ser)
                    self.channel4_on = True
                '''if not self.channel3_on:
                    msg_ser = struct.pack('bbbbf', 2, 3, 0, 0, 0)
                    self.ser.write(msg_ser)
                    self.channel3_on = True
                if not self.channel4_on:
                    msg_ser = struct.pack('bbbbf', 2, 4, 0, 0, 0)
                    self.ser.write(msg_ser)
                    self.channel4_on = True'''
            
            if self.last_channel7 * instr_turn < 0 and instr_turn > 0:
                self.pump_switch_time = self.pump_switch_time + 1

            self.last_channel7 = instr_turn
            return

        self.pump_switch_time = 0
        
        if instr_forward > 0:
            if self.advancing == False:
                self.advancing = True
                msg_ser = struct.pack('bbbbf', 127, 8, 1, 0, 0.8)
                self.ser.write(msg_ser)
                print("robot move forward continuously")
                self.channel1_on = False
                self.channel2_on = False
                self.channel3_on = True
                self.channel4_on = True
                
        else:
            if self.advancing == True:
                self.advancing = False
                msg_ser = struct.pack('bbbbf', 127, 9, 1, 0, 0)
                self.ser.write(msg_ser)
                print("robot stop moving")

            if instr_turn > 0 and self.turning == False:
                self.turning = True
                side = 2 if instr_side > 0 else 3
                side_str = 'left' if instr_side > 0 else 'right'
                msg_ser = struct.pack('bbbbf', 127, side, 1, 0, 0.5)
                self.ser.write(msg_ser)
                print("robot one step turning " + side_str)

                self.channel1_on = False
                self.channel2_on = True
                self.channel3_on = True
                self.channel4_on = False

            elif instr_turn < 0:
                self.turning = False

    
def main():
    rclpy.init()
    node = CrawlFromRC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()