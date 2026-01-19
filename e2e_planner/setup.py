from setuptools import setup
import os
from glob import glob

package_name = 'e2e_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 'util'],
    package_dir={'util': 'scripts/util'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyo yamashita',
    maintainer_email='s21c1135sc@s.chibakoudai.jp',
    description='End-to-end learning path planner',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'inference_node = e2e_planner.inference_node:main',
            'image_binarizer_node = e2e_planner.image_binarizer_node:main',
        ],
    },
)
