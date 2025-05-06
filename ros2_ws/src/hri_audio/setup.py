import os
from glob import glob
from setuptools import find_packages,setup

package_name = 'hri_audio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/sounds", glob(package_name + '/sounds/*')),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mapir',
    maintainer_email='famoreno@uma.es',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'microphone_capturer= hri_audio.microphone_publisher:main',
            'microphone_test= hri_audio.microphone_test:main',
            'audio_direction= hri_audio.audio_direction:main',
            'test= hri_audio.microphone_test:main',
            'assistant= hri_audio.assistant:main',
            'assistant_llm= hri_audio.assistant_llm:main',
            'helper= hri_audio.assistant_helper:main',
            'whisper_stt= hri_audio.whisper_stt:main',
            'google_stt= hri_audio.google_stt:main',
            'hri_tts= hri_audio.hri_tts:main',
            'python_tts= hri_audio.python_tts:main',
            'voice_detection= hri_audio.voice_detection:main',
            'speaker_controller= hri_audio.speaker_controller:main',
        ],
    },
)
