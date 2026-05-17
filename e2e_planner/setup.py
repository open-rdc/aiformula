from setuptools import setup
import os
from pathlib import Path

package_name = 'e2e_planner'
package_root = Path(__file__).resolve().parent


def source_glob(pattern):
    cwd = Path.cwd()
    return [os.path.relpath(path, cwd) for path in package_root.glob(pattern)]

# scripts/ を e2e_collector パッケージとして公開し、
# data_collector を ros2 run から実行可能にする
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.placenav', package_name + '.util', 'e2e_collector'],
    package_dir={
        'e2e_collector': 'scripts',
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), source_glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), source_glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'topomap'), source_glob('config/topomap/*.yaml')),
        (os.path.join('share', package_name, 'config', 'topomap', 'images'), source_glob('config/topomap/images/*.png')),
        (os.path.join('share', package_name, 'weights'), source_glob('weights/*.pt')),
        (os.path.join('share', package_name, 'weights'), source_glob('weights/*.json')),
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
            'data_collector     = e2e_collector.create_data:main',
            # 実機用ウェイポイント推論
            'inference_node     = e2e_planner.inference_node:main',
            # 学習・データセット前処理
            'train              = e2e_collector.train:main',
            'binarize_dataset   = e2e_collector.binarize_dataset:main',
            'create_topomap     = e2e_collector.create_topomap:main',
        ],
    },
)
