import rclpy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from random import *
from math import *


class Sub_class:
    def __init__(self):
        rclpy.init()  # 0. ros 초기화
        self.node = rclpy.create_node("wego_sub_node")  # 1. node 이름 설정
        self.sub = self.node.create_subscription(Pose, "/turtle1/pose", self.cb, qos_profile=1)  # 2. node 역할 설정
        self.pub = self.node.create_publisher(Twist, "/turtle1/cmd_vel", qos_profile=1)  # 2. node 역할 설정
        self.pose_msg = Pose()
        self.cmd_msg = Twist()
        self.pose_msg.angular_velocity

    def cb(self, msg):
        # goal_ang = atan2(5.54 - msg.y, 5.54 - msg.x)
        # # goal_ang = atan2(msg.y - 5.54, msg.x - 5.54)
        # if 1 < msg.x < 10 and 1 < msg.y < 10:
        #     self.cmd_msg.linear.x = random() * 2
        #     self.cmd_msg.angular.z = random() * 4 - 2
        # else:
        #     self.cmd_msg.linear.x = 0.3
        #     # self.cmd_msg.angular.z = 1.0
        #     # self.cmd_msg.angular.z = 0.1
        # self.cmd_msg.angular.z = goal_ang - msg.theta
        if msg.x < 8:
            self.cmd_msg.linear.x = 1.0
        else:
            self.cmd_msg.linear.x = 0.0
        self.pub.publish(self.cmd_msg)
        print(msg.x)


def main():
    sub_class = Sub_class()
    rclpy.spin(sub_class.node)
    sub_class.node.destroy_node()  # 4. node 삭제
    rclpy.shutdown()  # 5. ros 종료


if __name__ == "__main__":
    main()
