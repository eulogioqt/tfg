from launch import LaunchDescription
from launch_ros.actions import Node
from hri_vision.human_face_detector import DetectorType


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hri_vision',
            executable='detector',
            name='detector',
            parameters=[{
                'detector_name': f"{DetectorType.YOLOV5}"
            }],
            output='screen'
        ),
        Node(
            package='ros2web',
            executable='server',
            name='r2w_server',
            output='screen'
        )
    ])
