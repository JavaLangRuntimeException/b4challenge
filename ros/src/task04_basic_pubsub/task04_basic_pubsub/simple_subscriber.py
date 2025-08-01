#!/usr/bin/env python3
"""
簡単なSubscriberテスト - メッセージ受信を分かりやすく表示
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """
    簡単なSubscriberクラス
    トピックからメッセージを受信し、明確に表示する
    """

    def __init__(self):
        super().__init__('simple_subscriber')

        # サブスクライバーを作成
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # 受信カウンター
        self.received_count = 0

        self.get_logger().info('🚀 Simple Subscriber node has been started!')
        self.get_logger().info('📡 Waiting for messages on /topic...')

    def listener_callback(self, msg):
        """
        メッセージ受信時のコールバック関数

        Args:
            msg (String): 受信したメッセージ
        """
        self.received_count += 1

        self.get_logger().info(f'✅ Message #{self.received_count} received: "{msg.data}"')

        # コンソールにも明確に出力
        print(f'🎯 RECEIVED MESSAGE #{self.received_count}: {msg.data}')
        print(f'📊 Total messages received so far: {self.received_count}')
        print('=' * 60)


def main(args=None):
    """
    メイン関数
    """
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        print('\n🛑 Subscriber stopped by user')
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
