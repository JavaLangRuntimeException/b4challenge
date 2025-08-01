#!/usr/bin/env python3
"""
課題8: launchファイルの書き方を理解する
基本的なPublisherとSubscriberを同時に実行するlaunchファイル
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    基本的なPublisher/Subscriberを同時に実行するlaunchファイル

    Returns:
        LaunchDescription: launchの設定
    """
    return LaunchDescription([
        # Publisher ノード
        Node(
            executable='/home/taramanjimacbookpro/ws/install/task04_basic_pubsub/bin/publisher',
            name='basic_publisher',
            output='screen',
            parameters=[],
            arguments=[],
        ),

        # Subscriber ノード
        Node(
            executable='/home/taramanjimacbookpro/ws/install/task04_basic_pubsub/bin/subscriber',
            name='basic_subscriber',
            output='screen',
            parameters=[],
            arguments=[],
        ),
    ])
