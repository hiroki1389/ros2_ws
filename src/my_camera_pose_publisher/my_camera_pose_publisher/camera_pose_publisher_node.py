import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int32
from my_object_info_msgs.msg import CameraInfo

class CameraPosePublisherNode(Node):
    def __init__(self):
        super().__init__('camera_info_publisher_node')
        self.create_subscription(Int32, '/camera_info_changer', self.set_phase, 10)
        self.publisher = self.create_publisher(CameraInfo, '/camera_info_hiroki', 10)
        self.timer = self.create_timer(1.0/60.0, self.publish_pose)

        self.phase = -1

        #　配列順にカメラ位置を示す
        self.pos_x = [0.735, 1.433, 2.335]
        self.pos_y = [0.993, 0.854, 1.333]
        self.pos_z = [2.953, 1.044, 0.699]
        # pitch, yaw, rollの順番で姿勢は決める
        # pitchが ±pi/2のとき、rollとpitchが一意に決まらないからyawを優先して決める
        #   ⇛ ちなみにこの問題をジンバルロックという
        self.pitch = [0.00 * math.pi, 0.00 * math.pi, 0.00 * math.pi]
        self.yaw   = [0.00 * math.pi, 0.35 * math.pi, 0.50 * math.pi]
        self.roll  = [0.00 * math.pi, 0.00 * math.pi, 0.00 * math.pi]
        
        valuable_depth = 5.0 # RealSense D435は最大深度が10mなので5mくらいが妥当？

        # 視野角は公式サイトのやつじゃなくて、調整後の640*480の画像の視野角を実際に測った
        # horizontal_angle = math.radians(91.2)
        # vertical_angle = math.radians(65.5)
        horizontal_angle = math.atan(0.4) * 2
        vertical_angle   = math.atan(0.5) * 2

        self.get_logger().info(f'valuable_depth  :  {valuable_depth}')
        self.get_logger().info(f'horizontal_angle:  {horizontal_angle}')
        self.get_logger().info(f'vertical_angle :  {vertical_angle}')

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.valuable_depth = valuable_depth
        self.camera_info_msg.horizontal_angle = horizontal_angle
        self.camera_info_msg.vertical_angle = vertical_angle

    def set_phase(self, new_phase):
        self.phase = new_phase.data
        if self.phase < 0 or len(self.pos_x) <= self.phase:
            return
        
        self.get_logger().info('-------------------------------')
        self.get_logger().info(f'pos_x:  {self.pos_x[self.phase]}')
        self.get_logger().info(f'pos_y:  {self.pos_y[self.phase]}')
        self.get_logger().info(f'pos_z:  {self.pos_z[self.phase]}')
        # 基本はyaw,pitch,rollの順がわかりやすいからそうする
        self.get_logger().info(f'yaw  :  {self.yaw[self.phase]}')
        self.get_logger().info(f'pitch:  {self.pitch[self.phase]}')
        self.get_logger().info(f'roll :  {self.roll[self.phase]}')

        self.camera_info_msg.pos_x = self.pos_x[self.phase]
        self.camera_info_msg.pos_y = self.pos_y[self.phase]
        self.camera_info_msg.pos_z = self.pos_z[self.phase]
        self.camera_info_msg.yaw   = self.yaw  [self.phase]
        self.camera_info_msg.pitch = self.pitch[self.phase]
        self.camera_info_msg.roll  = self.roll [self.phase]

    def publish_pose(self):
        if self.phase < 0 or len(self.pos_x) <= self.phase:
            return
        self.publisher.publish(self.camera_info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPosePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
