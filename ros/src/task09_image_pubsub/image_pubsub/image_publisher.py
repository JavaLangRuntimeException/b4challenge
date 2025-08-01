#!/usr/bin/env python3
"""
課題9: 保存済みの画像ファイルをトピック通信し、jpgとして保存
Publisher: 画像ファイルをOpenCV形式で読込、cv_bridgeを使ってsensor_msgs/msg/Image形式に変換してPublish
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory


class ImagePublisher(Node):
    """
    画像ファイルを読み込んでPublishするクラス
    """

    def __init__(self):
        super().__init__('image_publisher')
        
        # パブリッシャーを作成
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        
        # cv_bridgeを初期化
        self.bridge = CvBridge()
        
        # タイマーを作成（2秒ごとに実行）
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # カウンター
        self.counter = 0
        
        # サンプル画像を準備
        self.prepare_sample_images()
        
        self.get_logger().info('Image Publisher node has been started.')

    def prepare_sample_images(self):
        """
        サンプル画像を作成・準備する
        """
        # パッケージのディレクトリを取得
        try:
            package_share_directory = get_package_share_directory('image_pubsub')
            self.images_dir = os.path.join(package_share_directory, 'images')
        except:
            # フォールバック：現在のディレクトリの images フォルダ
            self.images_dir = 'images'
        
        # imagesディレクトリを作成（存在しない場合）
        os.makedirs(self.images_dir, exist_ok=True)
        
        # サンプル画像リスト
        self.image_files = []
        
        # サンプル画像を生成
        for i in range(5):
            filename = f'sample_image_{i+1}.jpg'
            filepath = os.path.join(self.images_dir, filename)
            
            if not os.path.exists(filepath):
                # サンプル画像を作成
                img = self.create_sample_image(i)
                cv2.imwrite(filepath, img)
                self.get_logger().info(f'Created sample image: {filepath}')
            
            self.image_files.append(filepath)
        
        self.get_logger().info(f'Prepared {len(self.image_files)} sample images')

    def create_sample_image(self, index):
        """
        サンプル画像を作成
        
        Args:
            index (int): 画像のインデックス
            
        Returns:
            numpy.ndarray: OpenCV画像
        """
        # 400x300のカラー画像を作成
        img = np.zeros((300, 400, 3), dtype=np.uint8)
        
        # 背景色を設定
        colors = [
            (100, 100, 255),  # 赤系
            (100, 255, 100),  # 緑系
            (255, 100, 100),  # 青系
            (255, 255, 100),  # シアン系
            (255, 100, 255)   # マゼンタ系
        ]
        img[:] = colors[index % len(colors)]
        
        # テキストを描画
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, f'Sample Image {index + 1}', (50, 50), font, 1, (255, 255, 255), 2)
        cv2.putText(img, f'ROS2 Image Publisher', (50, 100), font, 0.8, (255, 255, 255), 2)
        cv2.putText(img, f'OpenCV + cv_bridge', (50, 150), font, 0.8, (255, 255, 255), 2)
        
        # 形状を描画
        if index == 0:
            cv2.circle(img, (300, 200), 50, (255, 255, 255), -1)
        elif index == 1:
            cv2.rectangle(img, (250, 170), (350, 230), (255, 255, 255), -1)
        elif index == 2:
            points = np.array([[300, 170], [260, 230], [340, 230]], np.int32)
            cv2.fillPoly(img, [points], (255, 255, 255))
        elif index == 3:
            cv2.ellipse(img, (300, 200), (50, 30), 0, 0, 360, (255, 255, 255), -1)
        else:
            # 星形
            pts = []
            for i in range(10):
                r = 50 if i % 2 == 0 else 25
                angle = i * np.pi / 5
                x = int(300 + r * np.cos(angle - np.pi/2))
                y = int(200 + r * np.sin(angle - np.pi/2))
                pts.append([x, y])
            cv2.fillPoly(img, [np.array(pts)], (255, 255, 255))
        
        return img

    def timer_callback(self):
        """
        タイマーコールバック関数
        定期的に画像をPublishする
        """
        if not self.image_files:
            self.get_logger().warn('No image files available')
            return
        
        # 現在の画像ファイルを選択
        image_file = self.image_files[self.counter % len(self.image_files)]
        
        try:
            # OpenCVで画像を読み込み
            cv_image = cv2.imread(image_file)
            
            if cv_image is None:
                self.get_logger().error(f'Failed to load image: {image_file}')
                return
            
            # OpenCV画像をROS2 Imageメッセージに変換
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # タイムスタンプを設定
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'
            
            # メッセージをPublish
            self.publisher_.publish(ros_image)
            
            # ログ出力
            self.get_logger().info(
                f'Published image [{self.counter}]: {os.path.basename(image_file)} '
                f'({cv_image.shape[1]}x{cv_image.shape[0]})'
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')
        
        self.counter += 1


def main(args=None):
    """
    メイン関数
    """
    rclpy.init(args=args)
    
    image_publisher = ImagePublisher()
    
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
