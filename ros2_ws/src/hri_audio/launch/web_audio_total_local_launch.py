import os
from dotenv import load_dotenv

from launch import LaunchDescription
from launch_ros.actions import Node

from speech_tools.models import STT_MODELS, TTS_MODELS, TTS_SPEAKERS
from llm_tools.models import PROVIDER, MODELS
from sancho_web.apis import API_LIST

load_dotenv()
GOOGLE_STT_API_KEY = os.environ.get("GOOGLE_STT_API_KEY")
GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY")

def generate_launch_description():
    return LaunchDescription([
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
            package='llm_tools',
            executable='llm',
            name='llm',
            output='screen',
            parameters=[{
                "llm_load_models": f"[['{PROVIDER.GEMINI}', ['{MODELS.LLM.GEMINI.GEMINI_FLASH}'], '{GEMINI_API_KEY}']]",
                "llm_active_provider": f"{PROVIDER.GEMINI}",
                "llm_active_model": f"{MODELS.LLM.GEMINI.GEMINI_FLASH}",         
            }]
        ),
        Node(
            package='sancho_ai',
            executable='sancho_ai',
            name='sancho_ai',
            output='screen'
        ),
        Node(
            package='ros2web',
            executable='server',
            name='r2w_server',
            output='screen'
        ),
        Node(
            package='sancho_web',
            executable='sancho_web',
            name='sancho_web',
            output='screen'
        ),
        Node(
            package='sancho_web',
            executable='api_rest',
            name='api_rest',
            output='screen',
            parameters=[{
                "apis": f"['{API_LIST.TTS_MODELS}', '{API_LIST.STT_MODELS}', '{API_LIST.LLM_MODELS}']"
            }]
        ),
        Node(
            package='mouth_controller',
            executable='mouth_node',
            name='mouth_node',
            output='screen'
        ),
    ])