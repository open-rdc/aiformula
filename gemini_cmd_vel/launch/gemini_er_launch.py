import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _spawn_gemini_controller_node(context) -> List[ExecuteProcess]:
    api_key = LaunchConfiguration('gemini_api_key').perform(context)
    model = LaunchConfiguration('gemini_model').perform(context)

    cmd = [
        'ros2', 'run', 'gemini_cmd_vel', 'gemini_controller',
        '--ros-args',
    ]

    if api_key:
        cmd.extend(['--param', f'api_key:={api_key}'])
    if model:
        cmd.extend(['--param', f'model:={model}'])

    # ExecuteProcessを使用してノードを実行することで、必要に応じて引数を動的に渡しやすくしていますが、
    # Nodeアクションを使用することも可能です。ここではリファレンスファイルのパターンに従っていますが、
    # シェル機能が必要ない限り、PythonのランチファイルではNodeアクションを使用するのがより標準的です。
    # ただし、リファレンスでは `gemini_live_controller_box_node` に対して ExecuteProcess を使用していました。
    # より良い統合のためにNodeアクションを使用するか、特定の理由があった場合はリファレンスパターンに従います。
    # リファレンスでは `python3 -m ...` を使用していたため、ExecuteProcess が使われていました。
    # ここでは `gemini_controller` というエントリーポイントがあるため、Node を使用できます。
    
    node_params = []
    if api_key:
        node_params.append({'api_key': api_key})
    if model:
        node_params.append({'model': model})

    return [
        Node(
            package='gemini_cmd_vel',
            executable='gemini_vla_node',
            name='gemini_vla_node',
            output='screen',
            parameters=node_params
        )
    ]


def generate_launch_description():
    world_file_path = PathJoinSubstitution(
        [
            get_package_share_directory('simulator'),
            'world',
            'shihou_world.sdf',
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth_image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth_points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat',
            '/imu_raw@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'gemini_api_key',
                default_value=EnvironmentVariable('GEMINI_API_KEY', default_value=''),
                description='Google GeminiのAPIキー。Geminiへのリクエストを無効にする場合は空のままにしてください。',
            ),
            DeclareLaunchArgument(
                'gemini_model',
                default_value=EnvironmentVariable('GEMINI_MODEL', default_value='gemini-robotics-er-1.5-preview'),
                description='Gemini APIで使用するモデル名。',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('ros_gz_sim'),
                            'launch',
                        ),
                        '/gz_sim.launch.py',
                    ]
                ),
                launch_arguments=[('gz_args', [world_file_path, ' -r'])],
            ),
            bridge,
            OpaqueFunction(function=_spawn_gemini_controller_node),
        ]
    )
