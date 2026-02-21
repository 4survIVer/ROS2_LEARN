#!/usr/bin/env python3
"""
Launch file for Behavior Tree Example 1
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bt_tutorials',
            executable='example1_basic_nodes',
            name='bt_example1',
            output='screen',
            emulate_tty=True  # For colored output
        )
    ])
