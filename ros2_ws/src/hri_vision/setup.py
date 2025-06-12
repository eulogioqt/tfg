import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hri_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    package_data={
        package_name: ['models/*', 'fonts/*', 'database/*.json', 'gui/*.qss', 'gui/*.png'],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eulogioqt',
    maintainer_email='euquemada@gmail.com',
    description='Human face recognition',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'camera = hri_vision.camera_node:main',
            'video = hri_vision.video_node:main',
            'detector = hri_vision.human_face_detector:main',       
            'recognizer = hri_vision.human_face_recognizer:main',
            'logic = hri_vision.hri_logic:main',
            'gui = hri_vision.hri_gui:main',
            'test_detector = hri_vision.test_detector_node:main',
            'test_encoder = hri_vision.test_encoder_node:main'
        ],
    },
)
