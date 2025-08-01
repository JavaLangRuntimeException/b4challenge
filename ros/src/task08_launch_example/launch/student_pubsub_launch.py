#!/usr/bin/env python3
"""
課題8: 学生情報Publisher/Subscriberを同時に実行するlaunchファイル
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    学生情報Publisher/Subscriberを同時に実行するlaunchファイル

    Returns:
        LaunchDescription: launchの設定
    """
    return LaunchDescription([
        # Student Info Publisher ノード
        Node(
            executable='/home/taramanjimacbookpro/ws/install/student_info_pubsub/bin/student_publisher',
            name='student_info_publisher',
            output='screen',
            parameters=[],
        ),

        # Student Info Subscriber ノード
        Node(
            executable='/home/taramanjimacbookpro/ws/install/student_info_pubsub/bin/student_subscriber',
            name='student_info_subscriber',
            output='screen',
            parameters=[],
        ),
    ])
