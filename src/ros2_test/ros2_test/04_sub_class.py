import rclpy
from std_msgs.msg import Int32


class Sub_class:
    def __init__(self):
        rclpy.init()  # 0. ros 초기화
        self.node = rclpy.create_node("wego_sub_node")  # 1. node 이름 설정
        self.sub_node = self.node.create_subscription(Int32, "/counter", self.cb, qos_profile=1)  # 2. node 역할 설정

    def cb(self, msg):
        print(msg)


def main():
    sub_class = Sub_class()
    rclpy.spin(sub_class.node)
    sub_class.node.destroy_node()  # 4. node 삭제
    rclpy.shutdown()  # 5. ros 종료


if __name__ == "__main__":
    main()
