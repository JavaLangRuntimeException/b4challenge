#!/usr/bin/env python3
"""
課題6: 学生情報を複数作成し、それらを1つにまとめて送受信できるメッセージをPublish
動的配列を利用
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from task05_student_msgs.msg import StudentInfo, StudentArray


class StudentArrayPublisher(Node):
    """
    学生情報配列をPublishするクラス
    """

    def __init__(self):
        super().__init__('student_array_publisher')

        # パブリッシャーを作成
        self.publisher_ = self.create_publisher(StudentArray, 'student_array', 10)

        # cv_bridgeを初期化
        self.bridge = CvBridge()

        # タイマーを作成（5秒ごとに実行）
        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # カウンター
        self.counter = 0

        # 学生データベース
        self.students_database = [
            {'id': 20240001, 'name': 'Tanaka Taro', 'grade': 'B4'},
            {'id': 20240002, 'name': 'Sato Hanako', 'grade': 'B4'},
            {'id': 20240003, 'name': 'Suzuki Ichiro', 'grade': 'M1'},
            {'id': 20240004, 'name': 'Watanabe Yuki', 'grade': 'M1'},
            {'id': 20240005, 'name': 'Ito Kazuko', 'grade': 'M2'},
            {'id': 20240006, 'name': 'Yamada Kenji', 'grade': 'B4'},
            {'id': 20240007, 'name': 'Nakamura Emi', 'grade': 'M1'},
            {'id': 20240008, 'name': 'Kobayashi Hiroshi', 'grade': 'B4'}
        ]

        self.get_logger().info('Student Array Publisher node has been started.')

    def create_sample_image(self, student_id, name, grade, index):
        """
        サンプル画像を作成

        Args:
            student_id (int): 学生ID
            name (str): 学生名
            grade (str): 学年
            index (int): 配列内のインデックス

        Returns:
            numpy.ndarray: OpenCV画像
        """
        # 250x180の白い画像を作成
        img = np.ones((180, 250, 3), dtype=np.uint8) * 255

        # 学生情報をテキストとして描画
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, f'ID: {student_id}', (10, 30), font, 0.5, (0, 0, 0), 1)
        cv2.putText(img, name, (10, 60), font, 0.5, (0, 0, 0), 1)
        cv2.putText(img, f'Grade: {grade}', (10, 90), font, 0.5, (0, 0, 0), 1)
        cv2.putText(img, f'Index: {index}', (10, 120), font, 0.5, (0, 0, 0), 1)
        cv2.putText(img, f'Batch: {self.counter}', (10, 150), font, 0.5, (0, 0, 0), 1)

        # 学年に応じて色を変える
        if grade == 'B4':
            color = (255, 100, 100)  # 青系
        elif grade == 'M1':
            color = (100, 255, 100)  # 緑系
        elif grade == 'M2':
            color = (100, 100, 255)  # 赤系
        else:
            color = (150, 150, 150)  # グレー

        cv2.rectangle(img, (150, 30), (230, 110), color, -1)

        return img

    def timer_callback(self):
        """
        タイマーコールバック関数
        定期的に学生情報配列をPublishする
        """
        # 送信する学生数を決定（2〜5人）
        num_students = 2 + (self.counter % 4)

        # StudentArrayメッセージを作成
        array_msg = StudentArray()

        # ヘッダーを設定
        array_msg.header = Header()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = 'student_array_frame'

        # 学生情報配列を作成
        array_msg.students = []

        for i in range(num_students):
            # データベースから学生を選択（循環）
            student_index = (self.counter * 3 + i) % len(self.students_database)
            student_data = self.students_database[student_index]

            # StudentInfoメッセージを作成
            student_info = StudentInfo()
            student_info.student_id = student_data['id']
            student_info.name = student_data['name']

            # サンプル画像を作成
            cv_image = self.create_sample_image(
                student_info.student_id,
                student_info.name,
                student_data['grade'],
                i
            )

            # OpenCV画像をROS2 Imageメッセージに変換
            try:
                student_info.image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            except Exception as e:
                self.get_logger().error(f'Failed to convert image for student {student_info.student_id}: {e}')
                continue

            # 配列に追加
            array_msg.students.append(student_info)

        # 総数を設定
        array_msg.total_count = len(array_msg.students)

        # メッセージをPublish
        self.publisher_.publish(array_msg)

        # ログ出力
        student_names = [s.name for s in array_msg.students]
        self.get_logger().info(
            f'Publishing student array (batch {self.counter}): '
            f'total_count={array_msg.total_count}, '
            f'students={student_names}'
        )

        self.counter += 1


def main(args=None):
    """
    メイン関数
    """
    rclpy.init(args=args)

    student_array_publisher = StudentArrayPublisher()

    try:
        rclpy.spin(student_array_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        student_array_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
