import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Xacro dosyasının tam yolu
  xacro_path = os.path.join(
      FindPackageShare('via_bringup').find('via_bringup'),
      'urdf', 'via_robot.xacro'
  )

  # robot_state_publisher ile robot_description parametresini oluştur
  robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{
          'robot_description': Command(['xacro ', xacro_path])  # ← düzeltildi
      }]
  )

  # Gazebo'ya spawn et
  spawn_entity_node = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      name='spawn_via',
      output='screen',
      arguments=[
        '-entity', 'via',
        '-topic', 'robot_description',
        '-x', '0',
        '-y', '0',
        '-z', '0.1',
        '-Y', '0'
     ]
  )

  # Gazebo'ya spawn işlemini 2 saniye gecikmeli yap
  delayed_spawn = TimerAction(
      period=2.0,
      actions=[spawn_entity_node]
  )

  return LaunchDescription([
      robot_state_publisher_node,
      delayed_spawn
  ])

