from setuptools import find_packages, setup

package_name = 'rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/aiformula_topomap.osm']),
        ('share/' + package_name + '/launch', ['launch/train.launch.py', 'launch/drive.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='a.w.g.d0201@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'train = rl.train.train:main',
            'lane_observation = rl.utitites.lane_observation_node:main',
            'drive = rl.run.drive:main',
        ],
    },
)
