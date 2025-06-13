"""TODO: Add module documentation."""
import os
from dotenv import load_dotenv
from launch import LaunchDescription
from launch_ros.actions import Node

from speech_tools.models import STT_MODELS, TTS_MODELS, TTS_SPEAKERS
from llm_tools.models import PROVIDER, MODELS

load_dotenv()
GOOGLE_STT_API_KEY = os.environ.get("GOOGLE_STT_API_KEY")
GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY")


def generate_launch_description():
"""TODO: Describe generate_launch_description.
"""
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
                "load_models": f"[['{TTS_MODELS.PIPER}', '']]",
                "active_model": f"{TTS_MODELS.PIPER}",
                "active_speaker": f"{TTS_SPEAKERS.PIPER.DAVEFX}"
            }]
        ),
        Node(
            package='speech_tools',
            executable='stt',
            name='stt',
            output='screen',
            parameters=[{
                "load_models": f"[['{STT_MODELS.GOOGLE}', '{GOOGLE_STT_API_KEY}']]",
                "active_model": f"{STT_MODELS.GOOGLE}"
            }]
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
            output='screen',
            parameters=[{
                "llm_load_models": f"[['{PROVIDER.GEMINI}', ['{MODELS.LLM.GEMINI.GEMINI_1_5_FLASH}'], '{GEMINI_API_KEY}']]",
                "llm_active_provider": f"{PROVIDER.GEMINI}",
                "llm_active_model": f"{MODELS.LLM.GEMINI.GEMINI_1_5_FLASH}",         
                #"embedding_load_models": f"[['{PROVIDER.GEMINI}', ['{MODELS.LLM.GEMINI.GEMINI_1_5_FLASH}'], '{GEMINI_API_KEY}']]",
                #"embedding_active_provider": f"{PROVIDER.GEMINI}",
                #"embedding_active_model": f"{MODELS.LLM.GEMINI.GEMINI_1_5_FLASH}",
            }]
        ),
        Node(
            package='sancho_ai',
            executable='sancho_ai',
            name='sancho_ai',
            output='screen'
        ),
        Node(
            package='face_controller',
            executable='face_node',
            name='face_node',
            output='screen'
        ),
    ])
