import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from math import *


class Pub_class:
    def __init__(self):
        rclpy.init()  # 0. ros 초기화
        self.node = rclpy.create_node("wego_pub_node")  # 1. node 이름 설정
        self.pub = self.node.create_publisher(Twist, "/turtle1/cmd_vel", qos_profile=1)  # 2. node 역할 설정
        self.sub = self.node.create_subscription(Imu, "/imu", self.imu_cb, qos_profile=1)  # 2. node 역할 설정
        self.node.create_timer(0.1, self.cb)
        self.cmd_msg = Twist()
        self.imu_msg = Imu()
        self.cmd_msg.linear.x
        self.degree = 0
        self.goal_degree = [45, 135, -135, -45]

        self.n = 0

    def cb(self):

        diff_degree = self.goal_degree[self.n] - self.degree
        if abs(diff_degree) < 1:
            self.n += 1
        self.n = self.n % len(self.goal_degree)
        print(f"degree:{self.degree}")
        print(f"goal:{self.goal_degree[self.n]}")
        print(f"diff:{diff_degree}")
        if abs(diff_degree) > 180:
            if diff_degree < 0:
                diff_degree = diff_degree + 360
            else:
                diff_degree = diff_degree - 360
        self.cmd_msg.angular.z = diff_degree * pi / 180
        self.pub.publish(self.cmd_msg)  # 3. publish

    def imu_cb(self, msg):
        self.pose_msg = msg
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
        self.degree = yaw * 180 / pi


def main():
    pub_class = Pub_class()
    rclpy.spin(pub_class.node)
    pub_class.node.destroy_node()  # 4. node 삭제
    rclpy.shutdown()  # 5. ros 종료


if __name__ == "__main__":
    main()
