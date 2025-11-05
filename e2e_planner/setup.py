from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'e2e_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyo yamashita',
    maintainer_email='s21c1135sc@s.chibakoudai.jp',
    description='End-to-end learning path planner',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inference_node = e2e_planner.inference_node:main',
        ],
    },
)
