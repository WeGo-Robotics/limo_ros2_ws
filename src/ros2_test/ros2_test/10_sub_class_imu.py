import rclpy
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from math import *


class Sub_class:
    def __init__(self):
        rclpy.init()  # 0. ros 초기화
        self.node = rclpy.create_node("wego_sub_node")  # 1. node 이름 설정
        self.sub_node = self.node.create_subscription(Imu, "/imu", self.cb, qos_profile=1)  # 2. node 역할 설정
        self.imu_msg = Imu()
        self.imu_msg.orientation.z

    def cb(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
        print(yaw * 180 / pi)


def main():
    sub_class = Sub_class()
    rclpy.spin(sub_class.node)
    sub_class.node.destroy_node()  # 4. node 삭제
    rclpy.shutdown()  # 5. ros 종료


if __name__ == "__main__":
    main()
