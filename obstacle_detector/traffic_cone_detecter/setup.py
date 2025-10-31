from setuptools import setup
from glob import glob
import os

package_name = 'traffic_cone_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/weights', glob('weights/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Formula SAE Team',
    maintainer_email='formula@example.com',
    description='ROS2 node for traffic cone detection using YOLOv5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_camera_node = traffic_cone_detector.detect_camera_node:main',
        ],
    },
)
