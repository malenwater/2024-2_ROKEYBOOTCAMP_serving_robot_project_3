#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast
from geometry_msgs.msg import Pose, PoseArray, Twist
from srv_call_test import TurtlebotArmClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import getkey
from std_msgs.msg import Header
import math
from sensor_msgs.msg import JointState
import os

class YoloDetect(Node):
    def __init__(self):
        super().__init__('yolo_detect')        
        self.subscription = self.create_subscription(
            String,
            '/yolo/detected_info',
            self.listener_callback,
            10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)
        self.twist = Twist()

        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.trajectory_msg = JointTrajectory()

        self.yolofind = False
        self.armrun = False
        self.yolo_x = 0.0
        self.yolo_y = 0.0

        # 보정값 (offset) 추가 복구
        self.right_low_x_offset = 0.0
        self.right_low_y_offset = 0.0
        self.right_high_x_offset = 0.0
        self.right_high_y_offset = 0.0
        self.left_low_x_offset = 0.0
        self.left_low_y_offset = 0.0
        self.left_high_x_offset = 0.0
        self.left_high_y_offset = 0.0

        # 파일에서 보정값 불러오기
        file_path = 'offset_values.txt'
        if os.path.exists(file_path):
            with open(file_path, "r") as file:
                for line in file:
                    parts = line.strip().split(":")
                    if len(parts) == 2:
                        var_name, value = parts
                        try:
                            value = float(value.strip())
                            setattr(self, var_name.strip(), value)
                        except ValueError:
                            pass

    def listener_callback(self, msg):
        if not self.armrun:
            try:
                data_list = ast.literal_eval(msg.data)
                if len(data_list) > 0:
                    self.yolo_x = data_list[0][1]
                    self.yolo_y = data_list[0][2]
                    print(f"Detected coordinates: {self.yolo_x}, {self.yolo_y}")
            except Exception as e:
                self.get_logger().error(f"Error processing the data: {e}")

    def append_pose_init(self, x, y, z):
        pose_array = PoseArray()
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose_array.poses.append(pose)
        return pose_array

    def arm_controll(self):
        arm_client = TurtlebotArmClient()
        print("task start!")
        print(f"Get coordinates: {self.yolo_x}, {self.yolo_y}")

        if self.yolofind:
            self.armrun = True
            arm_client.send_request(2, "open")
            time.sleep(1)
            
            yolo_robot_x = self.yolo_y + self.right_low_x_offset
            yolo_robot_y = self.yolo_x + self.right_low_y_offset
            
            pose_array = self.append_pose_init(0.14 - yolo_robot_x, -yolo_robot_y, 0.122354)
            arm_client.send_request(0, "", pose_array)
            time.sleep(1)
            
            pose_array = self.append_pose_init(0.14 - yolo_robot_x, -yolo_robot_y, 0.087354)
            arm_client.send_request(0, "", pose_array)
            
            arm_client.send_request(2, "close")
            arm_client.send_request(1, "home2")
            time.sleep(1)
            print("jobs_done")
            self.armrun = False
            self.yolofind = False

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetect()
    arm_client = TurtlebotArmClient()

    arm_client.send_request(1, "camera_home")
    time.sleep(1)

    while rclpy.ok():
        key_value = getkey.getkey()
        if key_value == '1':
            rclpy.spin_once(node)
            node.armrun = False
        elif key_value == '3':
            yolo_robot_x = node.yolo_y + node.right_low_x_offset
            yolo_robot_y = node.yolo_x + node.right_low_y_offset
            pose_array = node.append_pose_init(0.14 - yolo_robot_x, -yolo_robot_y, 0.122354)
            arm_client.send_request(0, "", pose_array)
            time.sleep(1)
        elif key_value == '4':
            pose_array = node.append_pose_init(0.14 - yolo_robot_x, -yolo_robot_y, 0.095354)
            arm_client.send_request(0, "", pose_array)
            time.sleep(1)
        elif key_value == '5':
            arm_client.send_request(2, "close")
            time.sleep(1)
        elif key_value == '6':
            arm_client.send_request(2, "open")
            time.sleep(1)
        elif key_value == '7':
            arm_client.send_request(1, "home2")
            time.sleep(1)
        elif key_value == '8':
            arm_client.send_request(1, "camera_home")
            time.sleep(1)
        elif key_value == 'q':
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

