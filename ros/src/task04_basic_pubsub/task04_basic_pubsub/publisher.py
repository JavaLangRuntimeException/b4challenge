#!/usr/bin/env python3
"""
課題4: 簡単なPublisherの実装
入力した文字列をPublishする
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    最小限のPublisherクラス
    ユーザーからの入力を受け取り、トピックにPublishする
    """

    def __init__(self):
        super().__init__('minimal_publisher')
        
        # パブリッシャーを作成
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # タイマーを作成（2秒ごとに実行）
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # メッセージカウンター
        self.i = 0
        
        self.get_logger().info('Publisher node has been started.')
        self.get_logger().info('Enter messages to publish (Ctrl+C to exit):')

    def timer_callback(self):
        """
        タイマーコールバック関数
        定期的にユーザー入力を受け取り、メッセージをPublishする
        """
        try:
            # ユーザーからの入力を受け取る
            user_input = input(f'Message {self.i}: ')
            
            # メッセージを作成
            msg = String()
            msg.data = f'[{self.i}] {user_input}'
            
            # メッセージをPublish
            self.publisher_.publish(msg)
            
            # ログ出力
            self.get_logger().info(f'Publishing: "{msg.data}"')
            
            self.i += 1
            
        except EOFError:
            # Ctrl+Dが押された場合
            self.get_logger().info('Input ended. Shutting down...')
            rclpy.shutdown()
        except KeyboardInterrupt:
            # Ctrl+Cが押された場合
            self.get_logger().info('KeyboardInterrupt received. Shutting down...')
            rclpy.shutdown()


def main(args=None):
    """
    メイン関数
    """
    rclpy.init(args=args)
    
    minimal_publisher = MinimalPublisher()
    
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
