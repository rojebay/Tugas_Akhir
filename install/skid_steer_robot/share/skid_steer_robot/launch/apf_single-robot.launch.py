#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Package directory
    pkg_dir = get_package_share_directory('skid_steer_robot')
    
    # Declare launch argument for world selection
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.world',
        description='Choose world file (empty_world.world, 1simple_world.world, 2simple_world.world, 3last_world.world)'
    )
    
    # URDF file path
    urdf_file = os.path.join(pkg_dir, 'urdf', 'skid_steer_robot.urdf.xacro')
    #skid_steer_robot.urdf.xacro
    #skid_steer_follower_robot.urdf.xacro
    
    
    # Robot description
    robot_description = Command(['xacro ', urdf_file])
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': [pkg_dir, '/worlds/', LaunchConfiguration('world')],
            'verbose': 'true'
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', 'skid_steer_robot',
            '-topic', 'robot_description',
            '-x', '-4.0',
            '-y', '-4.0',
            '-z', '0.1'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Sensor streaming node (start immediately)
    sensor_streamer = Node(
        package='skid_steer_robot',
        executable='sensor_streamer',
        name='sensor_streamer',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # APF Navigation node (start after 3 seconds delay)
    apf_navigation = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='skid_steer_robot',
                executable='apf_navigation',
                name='apf_navigation',
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        world_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        sensor_streamer,
        apf_navigation
    ])