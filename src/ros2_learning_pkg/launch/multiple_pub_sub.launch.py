from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Sensor Publishers
        Node(
            package='ros2_learning_pkg',
            executable='sensor_publisher_1',
            name='sensor_publisher_1',
            output='screen'
        ),
        Node(
            package='ros2_learning_pkg',
            executable='sensor_publisher_2', 
            name='sensor_publisher_2',
            output='screen'
        ),
        
        # Subscribers
        Node(
            package='ros2_learning_pkg',
            executable='display_subscriber_1',
            name='display_subscriber_1',
            output='screen'
        ),
        Node(
            package='ros2_learning_pkg',
            executable='alert_subscriber_2',
            name='alert_subscriber_2',
            output='screen'
        )
    ])
