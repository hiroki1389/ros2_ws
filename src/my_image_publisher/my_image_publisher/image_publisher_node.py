import rclpy
from sensor_msgs.msg import Image, CompressedImage
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import time

SEND_FREQUENCY = 1 # 送信頻度(s)

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.publisher = self.create_publisher(CompressedImage, '/rs/color/compressed', 10)
        self.subscription = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.last_time = time.time()

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_time >= SEND_FREQUENCY:
            self.get_logger().info('Received an image.')

            # /sensor_msgs/msg/Image から OpenCV イメージに変換
            bridge = CvBridge()
            image_data = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # 画像を JPEG 圧縮
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            _, compressed_data = cv2.imencode('.jpg', image_data, encode_param)
            
            # CompressedImage メッセージの作成
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"  # 圧縮形式を設定
            compressed_msg.data = compressed_data.tostring()

            self.get_logger().info('Sending a compressed image.')

            # CompressedImage メッセージを送信
            self.publisher.publish(compressed_msg)
            
            self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()

    input("Press Enter: ")
    # キーボード入力があった時点のタイムスタンプ（ミリ秒単位）をログに表示
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S.") + f"{int(time.time() * 1000) % 1000:03d}"
    node.get_logger().info(f"Input received at {timestamp}")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()