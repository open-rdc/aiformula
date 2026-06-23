import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'obstacle_recognition'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data/weights'),  glob('data/weights/*.pth')),
        (os.path.join('share', package_name, 'YOLOX/exps'), glob('YOLOX/exps/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kasai',
    maintainer_email='kasaiatsuki@gmail.com',
    description='ROS2 package for obstacle recognition using YOLOX',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pylon_detector_node = obstacle_recognition.pylon_detector_node:main',
        ],
    },
)