import os
import subprocess
import yaml
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'main_params.yaml'
    )
    # CANインタフェース起動ファイルのパス設定
    can_launch_path = os.path.join(
        get_package_share_directory('socketcan_interface'),
        'config',
        'can_up.sh'
    )

    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    # メイン実行機ノードの作成
    main_exec_node = Node(
        package = 'main_executor',
        executable = 'main_exec',
        parameters = [config_file_path],
        output='screen'
    )

    # 操縦機ノードの作成
    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        output='screen'
    )

    # 起動エンティティクラスの作成
    launch_discription = LaunchDescription()

    # 起動の追加
    if(launch_params['can'] is True):
        subprocess.run(['sudo', 'sh', can_launch_path])
    if(launch_params['joy'] is True):
        launch_discription.add_entity(joy_node)

    launch_discription.add_entity(main_exec_node)

    return launch_discription
