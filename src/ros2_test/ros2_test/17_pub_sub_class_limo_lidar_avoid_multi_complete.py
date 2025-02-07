import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import *
from math import *
import os


class Pub_class:
    def __init__(self):
        rclpy.init()  # 0. ros init
        self.node = rclpy.create_node("wego_pub_node")  # 1. node name
        self.pub = self.node.create_publisher(Twist, "/cmd_vel", qos_profile=1)  # 2. node role
        self.sub = self.node.create_subscription(LaserScan, "/scan", self.lidar_cb, qos_profile_sensor_data)  # 2. node role
        self.node.create_timer(0.15, self.cmd_cb)
        self.cmd_msg = Twist()
        self.lidar_msg = LaserScan()
        self.cmd_msg.linear.x
        self.obs_std = 5
        self.obstacle_flag = False
        self.cal_angle = 0

    def lidar_cb(self, msg):
        self.lidar_msg = msg
        self.avoidance()

    def avoidance(self):
        degree_increment = self.lidar_msg.angle_increment * 180 / pi
        degree_min = self.lidar_msg.angle_min * 180 / pi
        degree_max = self.lidar_msg.angle_max * 180 / pi
        mid_space = 0
        obstacles = []
        degrees = [degree_min + degree_increment * index for index, value in enumerate(self.lidar_msg.ranges)]
        for index, value in enumerate(self.lidar_msg.ranges):
            if abs(degrees[index]) < 90 and 0 < value < 0.5:
                obstacles.append(index)

                if len(obstacles) >= 2:
                    if obstacles[-1] - obstacles[-2] > self.obs_std:
                        mid_space = obstacles[-1] - obstacles[-2]
                        mid_center = (obstacles[-1] + obstacles[-2]) // 2

        if len(obstacles) > 0:
            self.obstacle_flag = True
            print(obstacles)
            right_space = obstacles[0]
            left_space = len(self.lidar_msg.ranges) - obstacles[-1]
            print([left_space, mid_space, right_space])
            if max([left_space, mid_space, right_space]) == left_space:
                left_center = len(self.lidar_msg.ranges) - left_space // 2
                print("going left")
                cal_angle = degrees[left_center] * pi / 180
            elif max([left_space, mid_space, right_space]) == right_space:
                right_center = right_space // 2
                cal_angle = degrees[right_center] * pi / 180
                print("going right")
            else:
                cal_angle = degrees[mid_center] * pi / 180
                print("going mid")
            print(cal_angle * 180 / pi)
            self.cal_angle = cal_angle

        else:
            self.obstacle_flag = False

    def cmd_cb(self):
        if self.obstacle_flag == True:
            self.cmd_msg.angular.z = self.cal_angle
            self.cmd_msg.linear.x = 0.0
        else:
            self.cmd_msg.linear.x = 0.1
            self.cmd_msg.angular.z = 0.0

        self.pub.publish(self.cmd_msg)


def main():
    pub_class = Pub_class()
    rclpy.spin(pub_class.node)
    pub_class.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
