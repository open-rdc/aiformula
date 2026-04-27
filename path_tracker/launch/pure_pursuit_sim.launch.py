from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pure_pursuit = Node(
        package='path_tracker',
        executable='pure_pursuit_sim',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'lookahead_distance': 2.0,
            'steered_gain': 1.8,
            'wheelbase': 0.8,
            'steering_max.pos': 30.0,
            'linear_max.vel': 1.0,
        }],
        remappings=[
            ('/frenet_planner/path', '/e2e_planner/path'),
        ],
    )

    autonomous_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--rate', '10',
            '/autonomous', 'std_msgs/msg/Bool', '{data: true}',
        ],
        output='screen',
    )

    return LaunchDescription([
        autonomous_pub,
        pure_pursuit,
    ])
