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

    def cmd_cb(self):
        # self.pub.publish(self.cmd_msg)
        pass

    def lidar_cb(self, msg):
        os.system("clear")
        degree_increment = msg.angle_increment * 180 / pi
        degree_min = msg.angle_min * 180 / pi
        degree_max = msg.angle_max * 180 / pi
        mid_space = 0
        obstacles = []
        degrees = [degree_min + degree_increment * index for index, value in enumerate(msg.ranges)]
        # ranges[ranges<0.3]=0
        # print(ranges)
        for index, value in enumerate(msg.ranges):
            # print(value)
            if abs(degrees[index]) < 90 and 0 < value < 0.5:
                obstacles.append(index)
                # print(f'obstacle: {degrees[index]}, {msg.ranges[index]}')
                if len(obstacles) >= 2:

                    if obstacles[-1] - obstacles[-2] > self.obs_std:
                        mid_space = obstacles[-1] - obstacles[-2]
                        mid_center = (obstacles[-1] + obstacles[-2]) // 2
            else:
                pass
        if len(obstacles) > 0:
            print(obstacles)
            right_space = obstacles[0]
            left_space = len(msg.ranges) - obstacles[-1]
            print([left_space, mid_space, right_space])
            if max([left_space, mid_space, right_space]) == left_space:
                left_center = len(msg.ranges) - left_space // 2
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
            self.cmd_msg.angular.z = cal_angle
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
