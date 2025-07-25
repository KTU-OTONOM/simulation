from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
 
def generate_launch_description():
 
 
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package='simulation').find('simulation')
    world_file_name = 'new_world.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')

    declare_verbose_arg = DeclareLaunchArgument(
            'verbose',
            default_value='true',  # Varsayılan olarak --verbose açık
            description='Start Gazebo on verbose mode'
        )
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world,
                        'verbose': LaunchConfiguration('verbose')
                        }.items())

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
    ld = LaunchDescription()

    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_verbose_arg)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld



