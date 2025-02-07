import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import *
import os


class Pub_class:
    def __init__(self):
        rclpy.init()  # 0. ros 초기화
        self.node = rclpy.create_node("wego_pub_node")  # 1. node 이름 설정
        self.pub = self.node.create_publisher(Twist, "/cmd_vel", qos_profile=1)  # 2. node 역할 설정
        self.sub = self.node.create_subscription(LaserScan, "/scan", self.lidar_cb, qos_profile=1)  # 2. node 역할 설정
        self.node.create_timer(0.1, self.cmd_cb)
        self.cmd_msg = Twist()
        self.lidar_msg = LaserScan()
        self.cmd_msg.linear.x

    def cmd_cb(self):
        # self.pub.publish(self.cmd_msg)  # 3. publish
        pass

    def lidar_cb(self, msg):
        os.system("clear")
        degree_increment = msg.angle_increment * 180 / pi
        degree_min = msg.angle_min * 180 / pi
        degree_max = msg.angle_max * 180 / pi
        degrees = [degree_min + degree_increment * index for index, value in enumerate(msg.ranges)]
        for index, value in enumerate(msg.ranges):
            if -30 < degrees[index] < 30 and 0 < value < 0.5:
                print(f"obstacle:{degrees[index]}")
                self.cmd_msg.linear.x = 0.3
            else:
                self.cmd_msg.linear.x = 0.0
        self.pub.publish(self.cmd_msg)  # 3. publish


def main():
    pub_class = Pub_class()
    rclpy.spin(pub_class.node)
    pub_class.node.destroy_node()  # 4. node 삭제
    rclpy.shutdown()  # 5. ros 종료


if __name__ == "__main__":
    main()
