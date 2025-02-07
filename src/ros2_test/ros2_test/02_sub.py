import rclpy
from std_msgs.msg import Int32

rclpy.init()  # 0. ros 초기화
node = rclpy.create_node("wego_sub_node")  # 1. node 이름 설정


def cb(msg):  # 3. callback
    print(msg)


sub_node = node.create_subscription(Int32, "/counter", callback=cb, qos_profile=1)  # 2. node 역할 설정
rclpy.spin(node)

node.destroy_node()  # 4. node 삭제
rclpy.shutdown()  # 5. ros 종료
