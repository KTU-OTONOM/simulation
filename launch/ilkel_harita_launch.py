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

  urdf_path = FindPackageShare(package='via_description').find('via_description')
  urdf_path_ = os.path.join(urdf_path, 'urdf/via.urdf.xacro')

  world_file_name = 'ilkel_harita.world'
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
  
 

  traffic_lights_service = Node(package='simulation', executable='traffic_lights_service.py',
                                  name='traffic_lights_service', output='screen', parameters=[{"index": ""},])
  
  rviz2 = Node(package='rviz2', executable="rviz2",
                            name='rviz2',  output='screen')
  
  robot_state_publisher = Node(package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen', 
    parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf_path_]), value_type=str)}])
  
  
  config_dir = os.path.join(pkg_share, 'config')  

  gazebo_ros = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    name='gazebo_ros',
    output='screen',
    arguments=[
      '-x','22.0' ,
      '-y' ,'28.7' ,
      '-z', '0.05',
      '-Y' ,'3.14',
      '-topic' , 'robot_description',
      '-entity', 'via',
    ],
  )

  compressed_image_publisher = Node(
        package='image_transport',
        executable='republish',
        arguments=[
            'raw',
            'compressed',
            ],
        remappings=[
            ('/in', '/zed_cam/camera_sensor/image_raw'),
            ('/out/compressed', '/camera_compressed'),
        ]
    )


  ld = LaunchDescription()
  
  ld.add_action(robot_state_publisher)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
  ld.add_action(declare_verbose_arg)
 
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(traffic_lights_service)
  #ld.add_action(rviz2)
  ld.add_action(gazebo_ros)
  ld.add_action(compressed_image_publisher)

  return ld


