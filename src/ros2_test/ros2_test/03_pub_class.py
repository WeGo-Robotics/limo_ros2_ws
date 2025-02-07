import rclpy
from std_msgs.msg import Int32


class Pub_class:
    def __init__(self):
        rclpy.init()  # 0. ros 초기화
        self.node = rclpy.create_node("wego_pub_node")  # 1. node 이름 설정
        self.pub_node = self.node.create_publisher(Int32, "/counter", qos_profile=1)  # 2. node 역할 설정
        self.node.create_timer(0.5, self.cb)
        self.int_msg = Int32()
        self.n = 0

    def cb(self):
        self.int_msg.data = self.n
        self.pub_node.publish(self.int_msg)  # 3. publish
        self.n = self.n + 1


def main():
    pub_class = Pub_class()
    rclpy.spin(pub_class.node)
    pub_class.node.destroy_node()  # 4. node 삭제
    rclpy.shutdown()  # 5. ros 종료


if __name__ == "__main__":
    main()
