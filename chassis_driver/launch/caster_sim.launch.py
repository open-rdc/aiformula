from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    caster_sim_node = Node(
        package='chassis_driver',
        executable='caster_sim_node',
        name='caster_sim_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    caster_sample_collector = Node(
        package='chassis_driver',
        executable='caster_sample_collector_sim',
        name='caster_sample_collector_sim',
        output='screen',
        parameters=[
            {'rate_hz': 50.0},
            {'use_sim_time': True},
            {'domain': 1},
            {'run_id': 1}
        ]
    )

    return LaunchDescription([
        caster_sim_node,
        caster_sample_collector
    ])

