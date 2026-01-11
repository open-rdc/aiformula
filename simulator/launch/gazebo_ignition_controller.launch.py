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


def _spawn_gemini_box_node(context) -> List[ExecuteProcess]:
    api_key = LaunchConfiguration('gemini_api_key').perform(context)
    model = LaunchConfiguration('gemini_model').perform(context)

    cmd = [
        'python3',
        '-m',
        'gemini_cmd_vel.gemini_controller_node',
        '--ros-args',
    ]

    if api_key:
        cmd.extend(['--param', f'api_key:={api_key}'])
    if model:
        cmd.extend(['--param', f'model:={model}'])

    return [ExecuteProcess(cmd=cmd, output='screen')]


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
                description='API key for Google Gemini. Leave blank to disable Gemini requests.',
            ),
            DeclareLaunchArgument(
                'gemini_model',
                default_value=EnvironmentVariable('GEMINI_MODEL', default_value=''),
                description=(
                    'Model name to use with the Gemini Live API. Either short name (recommended) '
                    'or full models/... form.'
                ),
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
            OpaqueFunction(function=_spawn_gemini_box_node),
        ]
    )
