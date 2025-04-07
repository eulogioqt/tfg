from glob import glob
from setuptools import find_packages, setup

package_name = 'sancho_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/commands", glob(package_name + '/commands/*.json'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eulogioqt',
    maintainer_email='euquemada@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sancho_web = sancho_web.sancho_web_node:main',
            'faceprint_api = sancho_web.faceprint_api_node:main'
        ],
    },
)
