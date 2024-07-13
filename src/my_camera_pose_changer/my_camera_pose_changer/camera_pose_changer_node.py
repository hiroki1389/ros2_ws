import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class CameraPoseChangerNode(Node):
    def __init__(self):
        super().__init__('camera_info_changer_node')
        self.publisher = self.create_publisher(Int32, '/camera_info_changer', 10)

    def get_user_input(self):
        try:
            user_input = input("Enter a number to publish to /camera_info_changer: ")
            return int(user_input)
        except ValueError:
            print("Invalid input. Please enter a valid number.")

    def publish_number(self, number):
        msg = Int32()
        msg.data = number
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseChangerNode()

    while rclpy.ok():
        number = node.get_user_input()
        if number is None:
            continue
        node.publish_number(number)

        # キーボード入力があった時点のタイムスタンプ（ミリ秒単位）をログに表示
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S.") + f"{int(time.time() * 1000) % 1000:03d}"
        node.get_logger().info(f"Input received at {timestamp}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
