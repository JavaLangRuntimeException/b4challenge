#!/usr/bin/env python3
"""
課題6: 学生情報配列をSubscribeして表示する
動的配列を利用
"""

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from task05_student_msgs.msg import StudentArray


class StudentArraySubscriber(Node):
    """
    学生情報配列をSubscribeするクラス
    """

    def __init__(self):
        super().__init__('student_array_subscriber')

        # サブスクライバーを作成
        self.subscription = self.create_subscription(
            StudentArray,
            'student_array',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # cv_bridgeを初期化
        self.bridge = CvBridge()

        # 受信回数をカウント
        self.receive_count = 0

        self.get_logger().info('Student Array Subscriber node has been started.')
        self.get_logger().info('Waiting for student array messages...')

    def listener_callback(self, msg):
        """
        学生情報配列メッセージ受信時のコールバック関数

        Args:
            msg (StudentArray): 受信した学生情報配列メッセージ
        """
        self.receive_count += 1

        # 受信した学生情報配列をログ出力
        self.get_logger().info(
            f'[{self.receive_count}] Received student array: '
            f'total_count={msg.total_count}, actual_count={len(msg.students)}'
        )

        # コンソールに詳細情報を出力
        print(f'\n=== Student Array #{self.receive_count} ===')
        print(f'Header timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        print(f'Frame ID: {msg.header.frame_id}')
        print(f'Total count: {msg.total_count}')
        print(f'Actual students count: {len(msg.students)}')
        print('-' * 50)

        # 各学生の情報を処理
        for i, student in enumerate(msg.students):
            print(f'Student [{i}]:')
            print(f'  ID: {student.student_id}')
            print(f'  Name: {student.name}')
            print(f'  Image size: {student.image.width}x{student.image.height}')
            print(f'  Image encoding: {student.image.encoding}')

            # 画像を処理
            try:
                # ROS2 ImageメッセージをOpenCV画像に変換
                cv_image = self.bridge.imgmsg_to_cv2(student.image, "bgr8")

                # 画像をファイルとして保存
                filename = f'batch_{self.receive_count}_student_{student.student_id}_index_{i}.jpg'
                cv2.imwrite(filename, cv_image)
                print(f'  Image saved as: {filename}')

                # 画像を表示（GUI環境がある場合）
                try:
                    window_name = f'Batch {self.receive_count} - Student {i}: {student.name}'
                    cv2.imshow(window_name, cv_image)
                    cv2.waitKey(1)  # 短時間表示
                except cv2.error as e:
                    print(f'  Cannot display image (no GUI?): {e}')

            except Exception as e:
                self.get_logger().error(f'Failed to process image for student {student.student_id}: {e}')

            print()

        print('=' * 60)

        # 統計情報を表示
        if self.receive_count % 3 == 0:
            print(f'\n>>> Statistics: Received {self.receive_count} student arrays so far <<<\n')


def main(args=None):
    """
    メイン関数
    """
    rclpy.init(args=args)

    student_array_subscriber = StudentArraySubscriber()

    try:
        rclpy.spin(student_array_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # OpenCVウィンドウを閉じる
        cv2.destroyAllWindows()
        student_array_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
