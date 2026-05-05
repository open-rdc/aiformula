import os
import subprocess
import yaml
import launch
from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
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

    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    use_sim_time = launch_params.get('sim', False)

    # robot_state_publisher（URDF から TF を publish）
    urdf_path = os.path.join(
        get_package_share_directory('simulator'),
        'models', 'ai_car1', 'model.urdf',
    )
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # カメラフレームの静的TF（chassis → ai_car1/camera_depth_link/camera_depth_link）
    # Foxy uses positional args: x y z yaw pitch roll frame_id child_frame_id
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.055', '0.0', '0.54',
            '0', '0', '0',
            'chassis', 'ai_car1/camera_depth_link/camera_depth_link',
        ],
        output='screen',
    )

    # メイン実行機ノードの作成
    main_exec_node = Node(
        package = 'main_executor',
        executable = 'main_exec',
        parameters = [config_file_path],
        output='screen'
    )
    # socketcanノードの作成
    socketcan_node = Node(
        package = 'socketcan_interface',
        executable = 'socketcan_interface_node',
        parameters = [config_file_path],
        output='screen'
    )
    # 操縦機ノードの作成
    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        output='screen'
    )

    # vectornav起動の作成
    vectornav_launch = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([os.path.join(
            get_package_share_directory('vectornav'), 'launch/'),
            'vectornav.launch.py'])
    )
    # odrive起動の作成
    odrive_launch = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([os.path.join(
            get_package_share_directory('main_executor'), 'launch/'),
            'odrive_can_launch.yaml'])
    )

    # 起動エンティティクラスの作成
    launch_discription = LaunchDescription()

    # 起動の追加
    if(launch_params['joy'] is True):
        launch_discription.add_action(joy_node)
    if(launch_params['vectornav'] is True):
        launch_discription.add_action(vectornav_launch)
    if(launch_params['odrive'] is True):
        launch_discription.add_action(odrive_launch)
    if(launch_params['socketcan'] is True):
        launch_discription.add_action(socketcan_node)

    launch_discription.add_action(robot_state_publisher)
    launch_discription.add_action(camera_tf)
    launch_discription.add_action(main_exec_node)

    return launch_discription
