from launch import LaunchDescription
from launch_ros.actions import Node

# Total time: 0.178
# Time lost: 0.004

def generate_launch_description():
    return LaunchDescription(
        [
            Node(  # FACE PIPELINE
                package="hri_vision",
                executable="detector",
                name="detector",
                output="screen",
                parameters=[{"active_cnn": False}, {"show_metrics": True}],
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
            Node(
                package="hri_vision",
                executable="recognizer",
                name="recognizer",
                output="screen",
                parameters=[{"show_metrics": True}],
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
            Node(
                package="hri_vision",
                executable="metrics",
                name="metrics",
                output="screen",
                prefix="xterm -hold -e",
                emulate_tty=True,
            ),
            Node(
                package='hri_vision',
                executable='camera',
                name='camera',
                output='screen',
                prefix="xterm -hold -e",
                emulate_tty=True,     
            ),
        ]
    )
