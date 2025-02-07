import rclpy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge


class Sub_class:
    def __init__(self):
        rclpy.init()  # 0. ros 초기화
        self.node = rclpy.create_node("wego_sub_node")  # 1. node 이름 설정
        self.sub_node = self.node.create_subscription(CompressedImage, "/image_raw/compressed", self.cb, qos_profile=1)  # 2. node 역할 설정
        self.bridge = CvBridge()

    def cb(self, msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        cv2.imshow("cv_img", cv_img)

        cv2.waitKey(1)


def main():
    sub_class = Sub_class()
    rclpy.spin(sub_class.node)
    sub_class.node.destroy_node()  # 4. node 삭제
    rclpy.shutdown()  # 5. ros 종료


if __name__ == "__main__":
    main()
