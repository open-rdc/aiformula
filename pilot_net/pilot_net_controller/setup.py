from glob import glob

from setuptools import find_packages, setup

package_name = 'pilot_net_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/weights', glob('weights/*.npy')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyo',
    maintainer_email='s25s1045sc@chibatech.ac.jp',
    description='PilotNet (end-to-end) ROS2 inference node, NumPy-only.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'pilot_net = pilot_net_controller.pilot_net_controller_node:main'
        ],
    },
)
