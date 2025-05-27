import os
from dotenv import load_dotenv
from launch import LaunchDescription
from launch_ros.actions import Node

from sancho_cloud_mic_audio_launch.apis import API_LIST
from speech_tools.models import STT_MODELS, TTS_MODELS, TTS_SPEAKERS
from llm_tools.models import PROVIDER, MODELS

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
                #"embedding_load_models": f"[['{PROVIDER.GEMINI}', ['{MODELS.LLM.GEMINI.GEMINI_FLASH}'], '{GEMINI_API_KEY}']]",
                #"embedding_active_provider": f"{PROVIDER.GEMINI}",
                #"embedding_active_model": f"{MODELS.LLM.GEMINI.GEMINI_FLASH}",
            }]
        ),
        Node(
            package='hri_audio',
            executable='microphone',
            name='microphone',
            output='screen',
        ),
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
            package='sancho_ai',
            executable='sancho_ai',
            name='sancho_ai',
            output='screen'
        ),
        Node(
            package='ros2web',
            executable='server',
            name='r2w_server',
            parameters=[{
                'topics': "[['/camera/color/recognition', 'IMAGE'], ['/logic/info/actual_people', 'ACTUAL_PEOPLE']]"
            }],
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
                "apis": f"['{API_LIST.TTS_MODELS}', '{API_LIST.STT_MODELS}', '{API_LIST.LLM_MODELS}', '{API_LIST.FACEPRINTS}', '{API_LIST.SESSIONS}']"
            }]
        ),

        Node(
            package='hri_vision',
            executable='video',
            name='video',
            output='screen'
        ),
        Node(
            package='hri_vision',
            executable='detector',
            name='detector',
            output='screen'
        ),
        Node(
            package='hri_vision',
            executable='recognizer',
            name='recognizer',
            output='screen'
        ),
        Node(
            package='hri_vision',
            executable='logic',
            name='logic',
            output='screen',
        ),
        Node(
            package='hri_vision',
            executable='gui',
            name='gui',
            output='screen',
        ),
        Node(
            package='rumi_web',
            executable='session_manager',
            name='session_manager',
            output='screen'
        ),
        Node(
            package='face_controller',
            executable='face_node',
            name='face_node',
            output='screen'
        ),
    ])