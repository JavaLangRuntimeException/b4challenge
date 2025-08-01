#!/usr/bin/env python3
"""
簡単なPublisherテスト - ユーザー入力なしで自動メッセージ送信
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # パブリッシャーを作成
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # タイマーを作成（2秒ごとに実行）
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # メッセージカウンター
        self.i = 0

        self.get_logger().info('Simple Publisher node has been started.')

    def timer_callback(self):
        """
        タイマーコールバック関数
        自動的にメッセージをPublishする
        """
        # メッセージを作成
        msg = String()
        msg.data = f'Auto message #{self.i}: Hello from simple publisher!'

        # メッセージをPublish
        self.publisher_.publish(msg)

        # ログ出力
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.i += 1


def main(args=None):
    """
    メイン関数
    """
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
