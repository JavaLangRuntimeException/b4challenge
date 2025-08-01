#!/usr/bin/env python3
"""
課題5: カスタムメッセージを使った学生情報Publisher
学生証番号(数値)、名前(文字列)、画像を送ることができるメッセージをPublish
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from task05_student_msgs.msg import StudentInfo


class StudentPublisher(Node):
    """
    学生情報をPublishするクラス
    """

    def __init__(self):
        super().__init__('student_publisher')

        # パブリッシャーを作成
        self.publisher_ = self.create_publisher(StudentInfo, 'student_info', 10)

        # cv_bridgeを初期化
        self.bridge = CvBridge()

        # タイマーを作成（3秒ごとに実行）
        timer_period = 3.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # カウンター
        self.counter = 0

        self.get_logger().info('Student Publisher node has been started.')

    def create_sample_image(self, student_id, name):
        """
        サンプル画像を作成

        Args:
            student_id (int): 学生ID
            name (str): 学生名

        Returns:
            numpy.ndarray: OpenCV画像
        """
        # 300x200の白い画像を作成
        img = np.ones((200, 300, 3), dtype=np.uint8) * 255

        # 学生情報をテキストとして描画
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, f'ID: {student_id}', (10, 50), font, 0.7, (0, 0, 0), 2)
        cv2.putText(img, f'Name: {name}', (10, 100), font, 0.7, (0, 0, 0), 2)
        cv2.putText(img, f'Count: {self.counter}', (10, 150), font, 0.7, (0, 0, 0), 2)

        # 色付きの矩形を追加
        color = (
            int(255 * (student_id % 3) / 2),
            int(255 * ((student_id + 1) % 3) / 2),
            int(255 * ((student_id + 2) % 3) / 2)
        )
        cv2.rectangle(img, (200, 50), (280, 130), color, -1)

        return img

    def timer_callback(self):
        """
        タイマーコールバック関数
        定期的に学生情報をPublishする
        """
        # サンプル学生データ
        students_data = [
            {'id': 20240001, 'name': 'Tanaka Taro'},
            {'id': 20240002, 'name': 'Sato Hanako'},
            {'id': 20240003, 'name': 'Suzuki Ichiro'},
            {'id': 20240004, 'name': 'Watanabe Yuki'},
            {'id': 20240005, 'name': 'Ito Kazuko'}
        ]

        # 現在の学生データを選択
        current_student = students_data[self.counter % len(students_data)]

        # StudentInfoメッセージを作成
        msg = StudentInfo()
        msg.student_id = current_student['id']
        msg.name = current_student['name']

        # サンプル画像を作成
        cv_image = self.create_sample_image(msg.student_id, msg.name)

        # OpenCV画像をROS2 Imageメッセージに変換
        try:
            msg.image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # メッセージをPublish
        self.publisher_.publish(msg)

        # ログ出力
        self.get_logger().info(
            f'Publishing student info: ID={msg.student_id}, Name="{msg.name}", '
            f'Image size={cv_image.shape}'
        )

        self.counter += 1


def main(args=None):
    """
    メイン関数
    """
    rclpy.init(args=args)

    student_publisher = StudentPublisher()

    try:
        rclpy.spin(student_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        student_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
