import rclpy
from sensor_msgs.msg import LaserScan
from math import *
from rclpy.qos import *
import os


class Sub_class:
    def __init__(self):
        rclpy.init()  # 0. ros 초기화
        self.node = rclpy.create_node("wego_sub_node")  # 1. node 이름 설정
        self.sub_node = self.node.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)  # 2. node 역할 설정
        self.lidar_msg = LaserScan()
        self.lidar_msg

    def cb(self, msg):
        os.system("clear")
        degree_increment = msg.angle_increment * 180 / pi
        degree_min = msg.angle_min * 180 / pi
        degree_max = msg.angle_max * 180 / pi
        degrees = [degree_min + degree_increment * index for index, value in enumerate(msg.ranges)]
        for index, value in enumerate(msg.ranges):
            if -30 < degrees[index] < 30 and 0 < value < 0.5:
                print(f"obstacle:{degrees[index]}")


def main():
    sub_class = Sub_class()
    rclpy.spin(sub_class.node)
    sub_class.node.destroy_node()  # 4. node 삭제
    rclpy.shutdown()  # 5. ros 종료


if __name__ == "__main__":
    main()
