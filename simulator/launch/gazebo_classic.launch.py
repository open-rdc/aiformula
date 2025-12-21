from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    # simulator package paths
    simulator_share = get_package_share_directory('simulator')
    world_file_default = PathJoinSubstitution([simulator_share, 'world', 'shihou_world.sdf'])
    model_path = PathJoinSubstitution([simulator_share, 'models'])

    # gazebo_ros launch
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_launch = PathJoinSubstitution([gazebo_ros_share, 'launch', 'gazebo.launch.py'])

    # args
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=world_file_default,
        description='Full path to world (SDF/World) file'
    )
    declare_gui = DeclareLaunchArgument('gui', default_value='true', description='Set to "false" to run headless.')
    declare_verbose = DeclareLaunchArgument('verbose', default_value='true', description='Gazebo verbose output.')
    declare_pause = DeclareLaunchArgument('pause', default_value='false', description='Start paused.')

    # Ensure Gazebo can find models (and keep any existing paths)
    set_model_path = SetEnvironmentVariable(
    name='GAZEBO_MODEL_PATH',
    value=[model_path, ':', EnvironmentVariable('GAZEBO_MODEL_PATH')]
)


    # Include Gazebo classic launch
    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'verbose': LaunchConfiguration('verbose'),
            'pause': LaunchConfiguration('pause'),
        }.items()
    )

    return LaunchDescription([
        declare_world,
        declare_gui,
        declare_verbose,
        declare_pause,
        set_model_path,
        include_gazebo,
    ])
