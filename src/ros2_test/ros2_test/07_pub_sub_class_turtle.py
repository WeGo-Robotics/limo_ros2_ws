import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Pub_class:
    def __init__(self):
        rclpy.init()  # 0. ros 초기화
        self.node = rclpy.create_node("wego_pub_node")  # 1. node 이름 설정
        self.pub = self.node.create_publisher(Twist, "/turtle1/cmd_vel", qos_profile=1)  # 2. node 역할 설정
        self.sub = self.node.create_subscription(Pose, "/turtle1/pose", self.pose_cb, qos_profile=1)  # 2. node 역할 설정
        self.node.create_timer(0.01, self.cb)
        self.cmd_msg = Twist()
        self.pose_msg = Pose()
        self.cmd_msg.linear.x

    def cb(self):
        if self.pose_msg.x < 8:
            self.cmd_msg.linear.x = 0.5
        else:
            self.cmd_msg.linear.x = 0.0
        self.pub.publish(self.cmd_msg)  # 3. publish

    def pose_cb(self, msg):
        self.pose_msg = msg
        print(msg.x)


def main():
    pub_class = Pub_class()
    rclpy.spin(pub_class.node)
    pub_class.node.destroy_node()  # 4. node 삭제
    rclpy.shutdown()  # 5. ros 종료


if __name__ == "__main__":
    main()
