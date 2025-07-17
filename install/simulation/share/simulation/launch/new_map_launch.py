from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    world_path = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'simulation',
        'worlds',
        'new_world.world'
    )

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['--verbose', world_path],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen'
        )
    ])

