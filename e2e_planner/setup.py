from setuptools import setup
import os
from glob import glob

package_name = 'e2e_planner'

# scripts/ を e2e_collector パッケージとして公開し、
# data_collector / data_collector_sim を ros2 run から実行可能にする
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.placenav', 'util', 'e2e_collector'],
    package_dir={
        'util': 'scripts/util',
        'e2e_collector': 'scripts',
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'topomap'), glob('config/topomap/*.yaml')),
        (os.path.join('share', package_name, 'config', 'topomap', 'images'), glob('config/topomap/images/*.png')),
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
            # 実機用データ収集 (ZED カメラ)
            'data_collector     = e2e_collector.data_collector:main',
            # シミュレーター用データ収集
            'data_collector_sim = e2e_collector.data_collector_sim:main',
            # 実機用ウェイポイント推論
            'inference_node     = e2e_planner.inference_node:main',
            # シミュレーター用ウェイポイント推論 (旧)
            'inference_node_sim = e2e_planner.inference_node_sim:main',
            # シミュレーター用トポロジカルナビゲーション
            'topo_nav_node_sim  = e2e_planner.topo_nav_node_sim:main',
        ],
    },
)
