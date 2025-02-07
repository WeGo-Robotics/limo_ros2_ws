import rclpy
from geometry_msgs.msg import Twist


class Pub_class:
    def __init__(self):
        rclpy.init()  # 0. ros 초기화
        self.node = rclpy.create_node("wego_pub_node")  # 1. node 이름 설정
        self.pub_node = self.node.create_publisher(Twist, "/turtle1/cmd_vel", qos_profile=1)  # 2. node 역할 설정
        self.node.create_timer(1.5, self.cb)
        self.cmd_msg = Twist()
        self.cmd_msg.linear.x

    def cb(self):
        self.cmd_msg.linear.x = -0.5
        self.pub_node.publish(self.cmd_msg)  # 3. publish


def main():
    pub_class = Pub_class()
    rclpy.spin(pub_class.node)
    pub_class.node.destroy_node()  # 4. node 삭제
    rclpy.shutdown()  # 5. ros 종료


if __name__ == "__main__":
    main()
