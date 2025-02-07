import rclpy
from std_msgs.msg import Int32

rclpy.init()  # 0. ros 초기화
node = rclpy.create_node("wego_pub_node")  # 1. node 이름 설정
pub_node = node.create_publisher(Int32, "/counter", qos_profile=1)  # 2. node 역할 설정
n = 0


def cb():
    global n
    int_msg = Int32()
    int_msg.data = n
    pub_node.publish(int_msg)  # 3. publish
    n = n + 1


node.create_timer(0.5, cb)
rclpy.spin(node)

node.destroy_node()  # 4. node 삭제
rclpy.shutdown()  # 5. ros 종료
