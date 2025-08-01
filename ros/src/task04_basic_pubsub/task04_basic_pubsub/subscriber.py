#!/usr/bin/env python3
"""
課題4: 簡単なSubscriberの実装
Subscribeした文字列を表示する
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    最小限のSubscriberクラス
    トピックからメッセージを受信し、ログに表示する
    """

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # サブスクライバーを作成
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('Subscriber node has been started.')
        self.get_logger().info('Waiting for messages...')

    def listener_callback(self, msg):
        """
        メッセージ受信時のコールバック関数
        
        Args:
            msg (String): 受信したメッセージ
        """
        self.get_logger().info(f'I heard: "{msg.data}"')
        
        # コンソールにも出力
        print(f'Received message: {msg.data}')


def main(args=None):
    """
    メイン関数
    """
    rclpy.init(args=args)
    
    minimal_subscriber = MinimalSubscriber()
    
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
