import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():

  urdf_path = FindPackageShare(package='via_description').find('via_description')
  urdf_path_ = os.path.join(urdf_path, 'urdf/via.urdf.xacro')

  gazebo_ros = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    name='gazebo_ros',
    output='screen',
    arguments=[
      '-x','29.0' ,
      '-y' ,'29.25' ,
      '-z', '0.5',
      '-Y' ,'3.14',
      '-topic' , 'robot_description',
      '-entity', 'via',
    ],
  )

  ld = LaunchDescription()

  ld.add_action(gazebo_ros)

  return ld


