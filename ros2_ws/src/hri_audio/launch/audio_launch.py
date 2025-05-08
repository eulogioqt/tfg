from launch import LaunchDescription
from launch_ros.actions import Node

from speech_tools.models import TTS_MODELS, TTS_SPEAKERS

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hri_audio',
            executable='assistant_helper',
            name='assistant_helper',
            output='screen'
        ),
        Node(
            package='hri_audio',
            executable='assistant',
            name='assistant',
            output='screen'
        ),
        Node(
            package='speech_tools',
            executable='tts',
            name='tts',
            output='screen',
            parameters=[{
                "load_models": f"['{TTS_MODELS.XTTS}']",
                "active_model": f"{TTS_MODELS.XTTS}",
                "active_speaker": f"{TTS_SPEAKERS.XTTS.ALMA_MARIA}"
            }]
        ),
        Node(
            package='speech_tools',
            executable='stt',
            name='stt',
            output='screen',
        ),
        Node(
            package='hri_audio',
            executable='audio',
            name='audio',
            output='screen',
        ),
        Node(
            package='llm_tools',
            executable='llm',
            name='llm',
            output='screen'
        ),
        Node(
            package='sancho_ai',
            executable='sancho_ai',
            name='sancho_ai',
            output='screen'
        ),
    ])