from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

import os


def generate_launch_description():
    # Declare world argument (default: shihou_world.sdf)
    # Available options: shihou_world.sdf, classic_world_ignition.sdf
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='shihou_world.sdf',
        description='World file name (e.g., shihou_world.sdf, classic_world_ignition.sdf)'
    )

    world_file_path = PathJoinSubstitution([
        get_package_share_directory('simulator'),
        'world',
        LaunchConfiguration('world')
    ])

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
            '/model/ai_car1/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [world_file_path, ' -r'])]
        ),
        bridge
    ])
