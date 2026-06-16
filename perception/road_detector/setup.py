from setuptools import setup
from setuptools import find_packages
from glob import glob

package_name = 'road_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/data/weights', glob('data/weights/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kyo Yamashita',
    maintainer_email='s25s1045sc@chibateck.ac.jp',
    description='ROS2 node for YOLOP object detection and BEV transformation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'road_detector_node = road_detector.road_detector_node:main',
        ],
    },
)
