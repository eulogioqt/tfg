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
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    package_data={
        package_name: ['sounds/*', 'utils/models/*'],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mapir',
    maintainer_email='famoreno@uma.es',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'assistant_helper=hri_audio.assistant_helper_node:main',
            'assistant=hri_audio.assistant_node:main',
            'audio=hri_audio.audio_node:main',
            'microphone=hri_audio.microphone_node:main'
        ],
    },
)
