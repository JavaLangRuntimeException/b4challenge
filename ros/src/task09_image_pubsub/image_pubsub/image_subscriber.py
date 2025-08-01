#!/usr/bin/env python3
"""
課題9: 画像をSubscribeして表示およびファイルとして保存
Subscriber: 受け取ったsensor_msgs/msg/Imageをcv_bridgeでOpenCV形式に変換して表示およびファイル保存
"""

import rclpy
from rclpy.node import Node
import cv2
import os
from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImageSubscriber(Node):
    """
    画像をSubscribeして表示・保存するクラス
    """

    def __init__(self):
        super().__init__('image_subscriber')
        
        # サブスクライバーを作成
        self.subscription = self.create_subscription(
            Image,
            'image_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # cv_bridgeを初期化
        self.bridge = CvBridge()
        
        # 受信回数をカウント
        self.receive_count = 0
        
        # 保存ディレクトリを作成
        self.output_dir = 'received_images'
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info('Image Subscriber node has been started.')
        self.get_logger().info(f'Images will be saved to: {os.path.abspath(self.output_dir)}')
        self.get_logger().info('Waiting for image messages...')

    def listener_callback(self, msg):
        """
        画像メッセージ受信時のコールバック関数
        
        Args:
            msg (Image): 受信した画像メッセージ
        """
        self.receive_count += 1
        
        # 受信した画像情報をログ出力
        self.get_logger().info(
            f'[{self.receive_count}] Received image: '
            f'{msg.width}x{msg.height}, encoding={msg.encoding}'
        )
        
        # コンソールに詳細情報を出力
        print(f'\n=== Image #{self.receive_count} ===')
        print(f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        print(f'Frame ID: {msg.header.frame_id}')
        print(f'Dimensions: {msg.width}x{msg.height}')
        print(f'Encoding: {msg.encoding}')
        print(f'Step: {msg.step}')
        print(f'Data length: {len(msg.data)} bytes')
        
        try:
            # ROS2 ImageメッセージをOpenCV画像に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # OpenCV画像情報を表示
            print(f'OpenCV shape: {cv_image.shape}')
            print(f'OpenCV dtype: {cv_image.dtype}')
            
            # ファイル名を生成（タイムスタンプ付き）
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # ミリ秒まで
            filename = f'image_{self.receive_count:03d}_{timestamp}.jpg'
            filepath = os.path.join(self.output_dir, filename)
            
            # 画像をJPGファイルとして保存
            success = cv2.imwrite(filepath, cv_image)
            
            if success:
                print(f'Image saved as: {filepath}')
                
                # ファイルサイズを確認
                file_size = os.path.getsize(filepath)
                print(f'File size: {file_size} bytes')
            else:
                print(f'Failed to save image as: {filepath}')
            
            # 画像を表示（GUI環境がある場合）
            try:
                window_name = f'Received Image #{self.receive_count}'
                cv2.imshow(window_name, cv_image)
                
                # 画像に情報をオーバーレイして別ウィンドウで表示
                info_image = cv_image.copy()
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(info_image, f'Count: {self.receive_count}', (10, 30), font, 0.7, (0, 255, 0), 2)
                cv2.putText(info_image, f'Size: {msg.width}x{msg.height}', (10, 60), font, 0.7, (0, 255, 0), 2)
                cv2.putText(info_image, f'Encoding: {msg.encoding}', (10, 90), font, 0.7, (0, 255, 0), 2)
                
                cv2.imshow('Image with Info', info_image)
                cv2.waitKey(1)  # 短時間表示
                
            except cv2.error as e:
                print(f'Cannot display image (no GUI?): {e}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')
            print(f'Error processing image: {e}')
        
        print('=' * 40)
        
        # 統計情報を表示
        if self.receive_count % 5 == 0:
            print(f'\n>>> Statistics: Received {self.receive_count} images so far <<<')
            print(f'>>> Output directory: {os.path.abspath(self.output_dir)} <<<\n')


def main(args=None):
    """
    メイン関数
    """
    rclpy.init(args=args)
    
    image_subscriber = ImageSubscriber()
    
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # OpenCVウィンドウを閉じる
        cv2.destroyAllWindows()
        image_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
