#!/usr/bin/env python3
# filepath: /home/rojebay/cobak/src/multi_agent/launch/multi_agent.launch.py

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Package directory - UPDATED to multi_agent
    pkg_dir = get_package_share_directory('multi_agent')
    
    # Declare launch argument for world selection
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.world',
        description='Choose world file(empty_world.world, 1simple_world.world, 2simple_world.world, 3last_world.world)'
    )
    
    # URDF file paths - UPDATED paths
    leader_urdf_file = os.path.join(pkg_dir, 'urdf', 'skid_steer_robot.urdf.xacro')
    follower_urdf_file = os.path.join(pkg_dir, 'urdf', 'skid_steer_follower_robot.urdf.xacro')
    
    # Robot descriptions
    leader_robot_description = Command(['xacro ', leader_urdf_file])
    follower_robot_description = Command(['xacro ', follower_urdf_file])
    
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
    
    # Leader robot group
    leader_group = GroupAction([
        PushRosNamespace('leader'),
        
        # Leader robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': leader_robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),
        
        # Spawn leader robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_leader',
            arguments=[
                '-entity', 'leader_robot',
                '-topic', '/leader/robot_description',
                '-x', '-4.0',   # Start position
                '-y', '-4.0',
                '-z', '0.1'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
    
    # Follower robot group
    follower_group = GroupAction([
        PushRosNamespace('follower'),
        
        # Follower robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': follower_robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),
        
        # Spawn follower robot (behind leader)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_follower',
            arguments=[
                '-entity', 'follower_robot',
                '-topic', '/follower/robot_description',
                '-x', '-6.0',   # Start 2m behind leader
                '-y', '-4.0',
                '-z', '0.1'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
    
    # Leader navigation node (start after 3 seconds) - UPDATED package name
    leader_navigation = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='multi_agent',  # UPDATED
                executable='leader_follower_navigation',
                name='leader_navigation',
                arguments=['leader', '0'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    # Follower navigation node (start after 5 seconds) - UPDATED package name
    follower_navigation = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='multi_agent',  # UPDATED
                executable='leader_follower_navigation',
                name='follower_navigation',
                arguments=['follower', '1'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    # Data logger for both robots - UPDATED package name
    data_logger_node = Node(
        package='multi_agent',  # UPDATED
        executable='data_logger',
        name='multi_agent_data_logger',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        gazebo_launch,
        leader_group,
        follower_group,
        leader_navigation,
        follower_navigation,
        data_logger_node
    ])