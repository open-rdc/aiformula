from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # === ワールドファイル ===
    world_file_path = PathJoinSubstitution([
        get_package_share_directory('simulator'),
        'world',
        'shihou_urdf_world.sdf'
    ])

    # === SDFモデルファイル ===
    sdf_file_path = os.path.join(
        get_package_share_directory('simulator'),
        'models',
        'ai_car1',
        'motor_spin.sdf'  # ← URDFではなくSDFを指定
    )

    # === GazeboにSDFモデルをスポーン ===
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_file_path,     # ← ここがURDFとの違い
            '-allow_renaming', 'false'
        ],
        output='screen'
    )

    # --- controller spawner ---
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    motor_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['motor_effort_controller'],
        output='screen'
    )

    # --- ros_gz_bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/imu_raw@sensor_msgs/msg/Imu@ignition.msgs.IMU'
        ],
        output='screen'
    )

    # === LaunchDescription ===
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[('gz_args', [world_file_path, ' -r'])]
        ),
        spawn_entity,                # ← SDFモデルをスポーン
        joint_state_spawner,
        motor_controller_spawner,
        bridge
    ])

