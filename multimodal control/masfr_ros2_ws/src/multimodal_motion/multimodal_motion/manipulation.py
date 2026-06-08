import rclpy
import pandas as pd
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int16, Float32MultiArray, MultiArrayDimension, MultiArrayLayout, Bool

class Manipulation(Node):

    def __init__(self) -> None:
        super().__init__('robot_manipulation')

        # sequence中每一个元素代表完整的一段（飞或者爬）的运动轨迹
        # 是由csv文件中总体轨迹拆分而来
        self.trajectory_sequence = []
        self.label_sequence = []

        self.load_trajectory_csv("~/Desktop/output.csv")
        self.motion_segments = len(self.label_sequence)

        print(f"Loaded {self.motion_segments} motion segments")
        print(self.trajectory_sequence)
        print(self.label_sequence)

        # 此变量代表了机器人运行阶段，即再哪一段轨迹运动
        self.robot_stage = 0

        # create manipulation publishers
        self.fly_path_publisher = self.create_publisher(Float32MultiArray, '/target_fly_path', 10)
        self.crawl_path_publisher = self.create_publisher(Float32MultiArray, '/target_crawl_path', 10)
        self.kill_all_publisher = self.create_publisher(Bool, '/kill_all', 10) # stop all nodes, robot motion terminates

        # create manipulation subscribers
        self.crawl_final_subscriber = self.create_subscription(Bool, '/crawl_terminate', self.motion_segment_final_callback, 10)
        self.fly_final_subscriber = self.create_subscription(Bool, '/fly_terminate', self.motion_segment_final_callback, 10)

        # 
        #self.timerMainloop = self.create_timer(0.1, self.mainloop)
        self.send_target_path()
        

    def load_trajectory_csv(self, str_csv):
        df = pd.read_csv(str_csv)

        def parse_point(point_str):
            stripped_str = point_str.strip().replace("(", "").replace(")", "").replace("[", "").replace("]", "")
            coords = [part.strip() for part in stripped_str.split() if part.strip()]
            coords = [c + "0" if c.endswith(".") else c for c in coords]
            return list(map(float, coords))

        df['coordinates'] = df['points'].apply(parse_point)

        last_label = ''
        trajectory = []

        for index, row in df.iterrows():
            coords = row['coordinates']
            label = row['labels']
            
            if label != last_label:
                if trajectory:
                    self.trajectory_sequence.append(trajectory)
                self.label_sequence.append(label)
                trajectory = []
            else:
                trajectory.append(coords)
            
            if index == len(df) - 1:
                self.trajectory_sequence.append(trajectory)
            last_label = label

        for index, (trajectory, label) in enumerate(zip(self.trajectory_sequence, self.label_sequence)):
            if label == "wf":
                start_vec = np.asarray(self.trajectory_sequence[index - 1][-1]) - np.asarray(self.trajectory_sequence[index - 1][-2])
                end_vec = np.asarray(self.trajectory_sequence[index + 1][1]) - np.asarray(self.trajectory_sequence[index + 1][0])
                start_yaw = np.arctan2(start_vec[1], start_vec[0])
                end_yaw = np.arctan2(end_vec[1], end_vec[0])

                length = len(trajectory)
                delta_yaw = (end_yaw - start_yaw) / (length - 1)
                for i, point in enumerate(trajectory):
                    point.append(start_yaw + delta_yaw * i)

    def send_target_path(self):
        # 向运动控制节点发送目标轨迹，判断下一段轨迹是飞行还是爬行，发送到相应的publisher
        msg_target_path = Float32MultiArray()
        trajectory = self.trajectory_sequence[self.robot_stage]
        flat_trajectory = [item for sublist in trajectory for item in sublist]

        if self.label_sequence[self.robot_stage] == "wf":
            print(f"send flying target trajectory")
            msg_target_path.layout.dim = [
                MultiArrayDimension(label = 'rows', size = len(trajectory), stride = 4),
                MultiArrayDimension(label = 'cols', size = 4, stride = 1)
            ]
            msg_target_path.layout.data_offset = 0
            msg_target_path.data = flat_trajectory

            self.fly_path_publisher.publish(msg_target_path)

        elif self.label_sequence[self.robot_stage] == "wg":
            print(f"send crawling target trajectory")
            msg_target_path.layout.dim = [
                MultiArrayDimension(label = 'rows', size = len(trajectory), stride = 3),
                MultiArrayDimension(label = 'cols', size = 3, stride = 1)
            ]
            msg_target_path.layout.data_offset = 0
            msg_target_path.data = flat_trajectory

            self.crawl_path_publisher.publish(msg_target_path)

    def motion_segment_final_callback(self, msg:Bool):
        if msg.data:
            print(f"Robot's {self.robot_stage + 1}th motion segment ends")
            self.robot_stage = self.robot_stage + 1
            if self.robot_stage < self.motion_segments:
                self.send_target_path()

            else:
                print("All motion terminates, process stop")
                kill_signal = Bool()
                kill_signal.data = True
                self.kill_all_publisher.publish(kill_signal)
                exit(0)


def main(args=None) -> None:
    print('Starting manipulation node')
    rclpy.init(args=args)
    manipulation_node = Manipulation()
    rclpy.spin(manipulation_node)
    manipulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)