import os
import subprocess
import yaml
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'main_params.yaml'
    )

    convert_sim_to_vectornav_pose_path = os.path.join(
        "/home/mirai/ros2_ws/src/aiformula",
        'scripts',
        'convert_sim_to_vectornav_pose.py'
    )

    # 引数の宣言
    simulator_launch_arg = DeclareLaunchArgument(
        'simulator_launch_path',
        default_value=os.path.join(
            get_package_share_directory('simulator'),
            'launch',
            'gazebo_simulator.launch.py'
        ),
        description='Path to the Gazebo simulator launch file'
    )

    # シミュレータ起動ファイルのパス設定
    simulator_launch_path = LaunchConfiguration('simulator_launch_path')

    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']


    # メイン実行機のログレベルの設定
    log_level = LaunchConfiguration("log_level")
    # デフォルトのログレベルを'info'に設定
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )
    
    # シミュレータ起動アクション
    simulator_launch = ExecuteProcess(
        cmd=['ros2', 'launch', simulator_launch_path],
        output='screen'
    )
    
    # スクリプト実行アクション
    convert_sim_to_vectornav_pose = ExecuteProcess(
        cmd=['python3', convert_sim_to_vectornav_pose_path],
        output='screen'
    )

    # メイン実行機ノードの作成
    main_sim_exec_node = Node(
        package = 'main_executor',
        executable = 'main_sim_exec',
        parameters = [config_file_path],
        arguments=["--ros-args", "--log-level", log_level],
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
    # if(launch_params['can'] is True):
    #    subprocess.run(['sudo', 'sh', can_launch_path])
    # if(launch_params['joy'] is True):
    #    launch_discription.add_entity(joy_node)

    launch_discription.add_action(simulator_launch_arg)
    launch_discription.add_entity(simulator_launch)
    launch_discription.add_entity(convert_sim_to_vectornav_pose)
    launch_discription.add_action(log_level_arg)
    launch_discription.add_entity(main_sim_exec_node)

    return launch_discription
