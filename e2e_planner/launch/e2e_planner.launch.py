from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    sim_flag = LaunchConfiguration('sim_flag').perform(context)

    if sim_flag.lower() == 'true':
        executable_name = 'inference_node_sim'
        model_name = 'e2e_model (2).pt'
    else:
        executable_name = 'inference_node'
        model_name = 'e2e_model.pt'

    inference_node = Node(
        package='e2e_planner',
        executable=executable_name,
        name='inference_node',
        output='screen',
        parameters=[{
            'model_name': model_name,
            'interval_ms': 100,
        }]
    )

    return [inference_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_flag',
            default_value='false',
            description='Flag to use simulation inference node'
        ),
        OpaqueFunction(function=launch_setup)
    ])
