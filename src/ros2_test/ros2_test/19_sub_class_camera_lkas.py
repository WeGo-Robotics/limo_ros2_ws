import rclpy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from rclpy.qos import *
from cv_bridge import CvBridge
import os
from math import *
from geometry_msgs.msg import Twist


class Sub_class:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("wego_sub_node")
        self.sub_node = self.node.create_subscription(CompressedImage, "/camera/color/image_raw/compressed", self.camera_cb, qos_profile_sensor_data)
        self.pub = self.node.create_publisher(Twist, "/cmd_vel", qos_profile=1)
        self.image = CompressedImage()
        # self.node.create_timer(0.5, self.cb)
        self.bridge = CvBridge()
        self.cmd_msg = Twist()

    def camera_cb(self, msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        y, x, channel = cv_img.shape
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        white_lower = np.array([0, 0, 180])
        white_upper = np.array([179, 20, 255])
        white_filter = cv2.inRange(hsv_img, white_lower, white_upper)

        yellow_lower = np.array([15, 80, 80])
        yellow_upper = np.array([45, 255, 255])
        yellow_filter = cv2.inRange(hsv_img, yellow_lower, yellow_upper)
        combine_filter = cv2.bitwise_or(white_filter, yellow_filter)

        and_img = cv2.bitwise_and(cv_img, cv_img, mask=combine_filter)
        margin_x1 = 0
        margin_x2 = 240
        margin_y = 260

        src_pt1 = (margin_x1, y)
        src_pt2 = (margin_x2, margin_y)
        src_pt3 = (x - margin_x2, margin_y)
        src_pt4 = (x - margin_x1, y)
        src_pts = np.float32([src_pt1, src_pt2, src_pt3, src_pt4])

        dst_margin_x = 120

        dst_pt1 = (dst_margin_x, y)
        dst_pt2 = (dst_margin_x, 0)
        dst_pt3 = (x - dst_margin_x, 0)
        dst_pt4 = (x - dst_margin_x, y)
        dst_pts = np.float32([dst_pt1, dst_pt2, dst_pt3, dst_pt4])

        matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        matrix_inv = cv2.getPerspectiveTransform(dst_pts, src_pts)
        warp_img = cv2.warpPerspective(and_img, matrix, (x, y))
        gray_img = cv2.cvtColor(warp_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(gray_img)
        bin_img[gray_img != 0] = 1
        print(bin_img.shape)
        center_index = x // 2

        window_num = 8
        margin = 40
        window_y_size = y // window_num  # 60
        left_indices = []
        right_indices = []

        for i in range(0, window_num):
            upper_y = y - window_y_size * (i + 1)
            lower_y = y - window_y_size * i

            left_window = bin_img[upper_y:lower_y, :center_index]
            left_histogram = np.sum(left_window, axis=0)
            left_histogram[left_histogram < 40] = 0

            right_window = bin_img[upper_y:lower_y, center_index:]
            right_histogram = np.sum(right_window, axis=0)
            right_histogram[right_histogram < 40] = 0

            try:
                left_nonzero = np.nonzero(left_histogram)[0]
                left_avg_index = (left_nonzero[0] + left_nonzero[-1]) // 2
                left_indices.append(left_avg_index)
                cv2.line(warp_img, (left_avg_index, upper_y + window_y_size // 2), (left_avg_index, upper_y + window_y_size // 2), (0, 0, 255), 10)
                cv2.rectangle(warp_img, (left_avg_index - margin, upper_y), (left_avg_index + margin, lower_y), (255, 0, 0), 3)
            except:
                pass
            try:
                right_nonzero = np.nonzero(right_histogram)[0]
                right_avg_index = (right_nonzero[0] + right_nonzero[-1]) // 2 + center_index
                right_indices.append(right_avg_index)
                cv2.line(
                    warp_img,
                    (right_avg_index, upper_y + window_y_size // 2),
                    (right_avg_index, upper_y + window_y_size // 2),
                    (255, 0, 0),
                    10,
                )
                cv2.rectangle(warp_img, (right_avg_index - margin, upper_y), (right_avg_index + margin, lower_y), (0, 0, 255), 3)
            except:
                pass

        non_lane_flag = False
        if len(left_indices) == 0 or len(right_indices) == 0:
            if len(left_indices) != 0:
                avg_indices = np.average(left_indices)
                center_index = margin
            elif len(right_indices) != 0:
                avg_indices = np.average(right_indices)
                print("right_lane_exist")
                center_index = x - margin
            else:
                avg_indices = 320
                center_index = 320
                non_lane_flag = True
            avg_indices = int(avg_indices)
        else:
            left_avg_indices = np.average(left_indices)
            right_avg_indices = np.average(right_indices)
            avg_indices = int((left_avg_indices + right_avg_indices) // 2)
        cv2.line(warp_img, (avg_indices, 0), (avg_indices, y), (0, 255, 255), 3)
        cv2.line(warp_img, (center_index, 0), (center_index, y), (0, 255, 0), 3)

        error_index = center_index - avg_indices
        if non_lane_flag == True:
            print("not find lane")
            self.lkas_steer = 0.35
        else:
            print("find lane")
            print(len(left_indices))
            self.lkas_steer = error_index * pi / x
        self.cmd_msg.linear.x = 0.1
        self.cmd_msg.angular.z = self.lkas_steer
        self.pub.publish(self.cmd_msg)
        warp_inv_img = cv2.warpPerspective(warp_img, matrix_inv, (x, y))

        cv2.circle(cv_img, src_pt1, 10, (255, 0, 0), -1)
        cv2.circle(cv_img, src_pt2, 10, (0, 255, 0), -1)
        cv2.circle(cv_img, src_pt3, 10, (0, 0, 255), -1)
        cv2.circle(cv_img, src_pt4, 10, (0, 255, 255), -1)
        cv2.imshow("cv_img", cv_img)
        cv2.imshow("and_img", and_img)
        # cv2.imshow("combine_filter",combine_filter)
        cv2.imshow("warp_img", warp_img)
        cv2.imshow("warp_inv_img", warp_inv_img)
        # cv2.imshow("gray_img",gray_img)
        # cv2.imshow("bin_img",bin_img)

        cv2.waitKey(1)


def main():
    sub_class = Sub_class()
    rclpy.spin(sub_class.node)
    sub_class.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
