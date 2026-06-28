from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

import os


def generate_launch_description():
    world_file_path = PathJoinSubstitution([
        get_package_share_directory('simulator'),
        'world',
        'shihou_world.sdf'
    ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/imu_raw@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

            # '/motor_spin_angle@std_msgs/msg/Float64@gz.msgs.Double',
            # '/motor_spin_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # '/caster_yaw_angle@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )
            ]),
            launch_arguments={
                'gz_args': [world_file_path, ' -r']
            }.items(),
        ),
        bridge
    ])