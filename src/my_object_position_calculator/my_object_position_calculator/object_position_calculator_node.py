import rclpy
from rclpy.node import Node
from bboxes_ex_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from my_object_info_msgs.msg import CameraInfo
from my_object_info_msgs.msg import ObjectInfo
from my_object_info_msgs.msg import ObjectInfoArray
from cv_bridge import CvBridge
import numpy as np
import math

class ObjectPositionCalculatorNode(Node):
    def __init__(self):
        super().__init__('object_position_calculator_node')
        self.create_subscription(BoundingBoxes, '/bounding_boxes', self.bounding_boxes_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info_hiroki', self.camera_info_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_image_callback, 10)
        self.publisher_ = self.create_publisher(ObjectInfoArray, '/object_info', 10)
        self.bridge = CvBridge()

        # BBoxで検出された物体のうち閾値以上のものをすべて格納するリスト（有効深度は考えない）
        self.object_info_array = []
        # 物体情報をまとめて送信するカスタムメッセージ（有効深度を考えて厳選されたもの）
        self.object_info = ObjectInfoArray()
        # 送信するカスタムメッセージにカメラ情報が含まれているかのフラグ
        self.camera_info_flag = False

        # YOLOXで検出できる物体をまとめたリスト
        self.categories = [
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
            "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
            "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
            "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
            "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
            "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
            "scissors", "teddy bear", "hair drier", "toothbrush"
        ]

    def bounding_boxes_callback(self, msg):
        self.get_logger().info('Start bounding_boxes_callback')

        # 検出物体のリストを初期化
        self.object_info = ObjectInfoArray()
        self.object_info_array = []
        self.camera_info_flag = False

        # BoundingBoxesメッセージがbounding_boxes属性を持つ場合
        if hasattr(msg, 'bounding_boxes'):
            for box in msg.bounding_boxes:

                # 物体の確率が閾値以下なら無視
                detection_threshold = 0.5
                if box.probability < detection_threshold:
                    continue

                # バウンディングボックスのクラス名によるIDを計算
                try:
                    object_id = self.categories.index(box.class_id)
                except ValueError:
                    self.get_logger().error(f"Unknown category: {box.class_id}.")
                    object_id = 0

                # バウンディングボックスの二次元座標を取得
                xmin = box.xmin
                ymin = box.ymin
                xmax = box.xmax
                ymax = box.ymax
                # バウンディングボックスの中心座標を計算
                bbox_center_x = (xmin + xmax) / 2
                bbox_center_y = (ymin + ymax) / 2
                # バウンディングボックスの高さと幅をまず保存（あとでdepthと合わせて実際のスケールに変換）
                bbox_width = xmax-xmin
                bbox_height = ymax-ymin


                # ObjectInfoメッセージの作成（pos_x,pos_yは一時的に二次元座標入れる，pos_zはmath.nan）
                object_info = ObjectInfo()
                object_info.object_name = box.class_id
                object_info.object_id = object_id
                object_info.object_pos_x = float(bbox_center_x)
                object_info.object_pos_y = float(bbox_center_y)
                object_info.object_pos_z = math.nan
                object_info.object_width = float(bbox_width)
                object_info.object_height = float(bbox_height)

                self.object_info_array.append(object_info)
        else:
            self.get_logger().warn("BoundingBoxesメッセージに'bounding_boxes'属性が存在しません。")

    def camera_info_callback(self, camera_info):
        # カメラの位置情報を保存
        self.object_info.camera_info = camera_info
        self.camera_info_flag = True
        #self.get_logger().info("camera info changed true")

    # depth画像情報を処理
    def depth_image_callback(self, msg):

        # カメラ情報がない場合はスキップ
        if self.camera_info_flag is False:
            return
        
        # self.get_logger().info("camera info is true")
        
        try:
            # OpenCV形式に変換
            depth_image = self.bridge.imgmsg_to_cv2(msg)

            # 各バウンディングボックスの中心座標での深度を取得
            for object_info in self.object_info_array:
                if object_info:
                    
                    # 必要なもの
                    bbox_center_x = int(object_info.object_pos_x)
                    bbox_center_y = int(object_info.object_pos_y)
                    bbox_width = object_info.object_width
                    bbox_height = object_info.object_height

                    # RGBとdepthの解像度とカメラの視野角
                    width, height = 640, 480
                    horizontal_angle = self.object_info.camera_info.horizontal_angle
                    vertical_angle = self.object_info.camera_info.vertical_angle

                    # 画像中心から端までの距離を1mとしたときの、画像中心から物体までの距離を計算（ピクセル距離を正規化）
                    difference_standard_x = (bbox_center_x - width/2) / (width/2)
                    difference_standard_y = (bbox_center_y - height/2) / (height/2)

                    # depth画像の視野角からそれぞれ真ん中から端までの距離が1mになるときのカメラの距離を計算
                    ### depth_standard_distance_x = 1.36 # とあるサイトの値、真偽は不明
                    standard_distance_x = 1/math.atan(horizontal_angle/2)
                    standard_distance_y = 1/math.atan(vertical_angle/2)

                    # 物体位置における深度データを取得（単位がmmのため、メートルに直す）
                    depth_at_bbox_center = float(depth_image[bbox_center_y, bbox_center_x])/1000.0
                    self.get_logger().info(object_info.object_name)
                    self.get_logger().info(f'depth :  {depth_at_bbox_center}')

                    # 画像中心から物体中心までの角度を計算
                    theta_x = math.atan(difference_standard_x/standard_distance_x)
                    theta_y = math.atan(difference_standard_y/standard_distance_y)

                    # depthから、物体の位置における画像中心までの距離を求める
                    depth_at_center = depth_at_bbox_center * math.cos(math.radians(theta_x)) * math.cos(math.radians(theta_y))

                    # 有効深度でなかったら処理をスキップ
                    valuable_depth = self.object_info.camera_info.valuable_depth
                    if(depth_at_center > valuable_depth):
                        continue

                    # 標準距離における物体の幅と高さを計算（ピクセル数を正規化）
                    object_standard_width = bbox_width / (width/2)
                    object_standard_height = bbox_height / (height/2)

                    # depth_centerと標準距離の比を用いてバウンディングボックスの高さと幅を、物体の実際の高さと幅に変換
                    object_width = object_standard_width * (depth_at_center/standard_distance_x)
                    object_height = object_standard_height * (depth_at_center/standard_distance_y)

                    # object_infoの更新
                    object_info.object_width = float(object_width)
                    object_info.object_height = float(object_height)

                    # depth_centerと標準距離の比を用いて画像中心から物体までの距離を実際の距離（v'-w'座標）に変換
                    # これが、v',w'平面座標における座標であることに注意
                    difference_x = difference_standard_x * (depth_at_center/standard_distance_x)
                    difference_y = difference_standard_y * (depth_at_center/standard_distance_y)

                    # カメラの位置を使用して三次元位置を計算
                    try:
                        yaw = self.object_info.camera_info.yaw
                        pitch = self.object_info.camera_info.pitch
                        roll = self.object_info.camera_info.roll

                        # 画像中心におけるx,y,z座標を計算
                        cx = self.object_info.camera_info.pos_x + (depth_at_center * np.cos(pitch) * np.cos(yaw))
                        cy = self.object_info.camera_info.pos_y + (depth_at_center * np.sin(pitch))
                        cz = self.object_info.camera_info.pos_z + (depth_at_center * np.cos(pitch) * np.sin(yaw))

                        # v',w'座標のdifference_x, difference_yを、roll=0のvw座標に変換
                        v = difference_x * np.cos(roll) - difference_y * np.sin(roll)
                        w = difference_x * np.sin(roll) + difference_y * np.cos(roll)

                        # vw座標をx,y,z座標に変換
                        x =  v * np.sin(yaw) + w * np.sin(pitch) * np.cos(yaw) + cx
                        y =  v * 0           - w * np.cos(pitch)               + cy
                        z = -v * np.cos(yaw) + w * np.sin(pitch) * np.sin(yaw) + cz

                        # 格納
                        object_info.object_pos_x = x
                        object_info.object_pos_y = y
                        object_info.object_pos_z = z

                        self.object_info.object_info_array.append(object_info)
                        
                    except Exception as e:
                        self.get_logger().error(f"Error calculating object position: {str(e)}")

            self.publisher_.publish(self.object_info)
            self.get_logger().info('Sending ObjectInfo: ')
            #self.get_logger().info(f'{self.object_info.camera_info}')
            #self.get_logger().info(f'{self.object_info.object_info_array}')
            # 検出物体のリストを初期化
            self.object_info = ObjectInfoArray()
            self.object_info_array = []
            self.camera_info_flag = False

        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPositionCalculatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()