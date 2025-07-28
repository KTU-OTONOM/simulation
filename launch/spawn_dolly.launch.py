from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('via_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'dolly.urdf.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', xacro_file])
            }]
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'dolly',
                '-topic', 'robot_description',
                '-x', '2.0', '-y', '2.0', '-z', '0.0'
            ],
            output='screen'
        )
    ])

