#!/usr/bin/env python3
"""
課題5: カスタムメッセージを使った学生情報Subscriber
学生証番号、名前、画像を受信して表示する
"""

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from task05_student_msgs.msg import StudentInfo


class StudentSubscriber(Node):
    """
    学生情報をSubscribeするクラス
    """

    def __init__(self):
        super().__init__('student_subscriber')

        # サブスクライバーを作成
        self.subscription = self.create_subscription(
            StudentInfo,
            'student_info',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # cv_bridgeを初期化
        self.bridge = CvBridge()

        # 受信回数をカウント
        self.receive_count = 0

        self.get_logger().info('Student Subscriber node has been started.')
        self.get_logger().info('Waiting for student info messages...')

    def listener_callback(self, msg):
        """
        学生情報メッセージ受信時のコールバック関数

        Args:
            msg (StudentInfo): 受信した学生情報メッセージ
        """
        self.receive_count += 1

        # 受信した学生情報をログ出力
        self.get_logger().info(
            f'[{self.receive_count}] Received student info: '
            f'ID={msg.student_id}, Name="{msg.name}"'
        )

        # コンソールにも詳細情報を出力
        print(f'\n=== Student Information #{self.receive_count} ===')
        print(f'Student ID: {msg.student_id}')
        print(f'Name: {msg.name}')
        print(f'Image size: {msg.image.width}x{msg.image.height}')
        print(f'Image encoding: {msg.image.encoding}')

        # 画像を処理
        try:
            # ROS2 ImageメッセージをOpenCV画像に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")

            # 画像情報を表示
            print(f'OpenCV image shape: {cv_image.shape}')

            # 画像をファイルとして保存
            filename = f'received_student_{msg.student_id}_{self.receive_count}.jpg'
            cv2.imwrite(filename, cv_image)
            print(f'Image saved as: {filename}')

            # 画像を表示（GUI環境がある場合）
            try:
                cv2.imshow(f'Student {msg.student_id}', cv_image)
                cv2.waitKey(1)  # 短時間表示
            except cv2.error as e:
                print(f'Cannot display image (no GUI?): {e}')

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

        print('=' * 40)


def main(args=None):
    """
    メイン関数
    """
    rclpy.init(args=args)

    student_subscriber = StudentSubscriber()

    try:
        rclpy.spin(student_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # OpenCVウィンドウを閉じる
        cv2.destroyAllWindows()
        student_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
