#!/usr/bin/env python3
"""
課題8: 画像Publisher/Subscriberを同時に実行するlaunchファイル
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    画像Publisher/Subscriberを同時に実行するlaunchファイル

    Returns:
        LaunchDescription: launchの設定
    """
    return LaunchDescription([
        # Image Publisher ノード
        Node(
            executable='/home/taramanjimacbookpro/ws/install/image_pubsub/bin/image_publisher',
            name='image_publisher_node',
            output='screen',
            parameters=[],
        ),

        # Image Subscriber ノード
        Node(
            executable='/home/taramanjimacbookpro/ws/install/image_pubsub/bin/image_subscriber',
            name='image_subscriber_node',
            output='screen',
            parameters=[],
        ),
    ])
