import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Path ke package
    pkg_path = get_package_share_directory('my_robot_pkg')
    
    # 1. Jalankan Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', 'worlds/empty.world'],
        output='screen'
    )
    
    # 2. Spawn robot contoh (differential drive)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'diffbot', '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )
    
    # 3. Jalankan node controller
    controller = Node(
        package='my_robot_pkg',
        executable='robot_controller',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_robot,
        controller
    ])