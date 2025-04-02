from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2web',
            executable='server',
            name='server',
            parameters=[{
                'topics': "[['/video/color/image_raw', 'IMAGE'], ['/chatter', 'CHATTER_123']]"
            }],
            output='screen'
        )
    ])
