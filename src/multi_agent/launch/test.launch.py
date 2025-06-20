#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    return LaunchDescription([
        # Test leader navigation
        Node(
            package='multi_agent',
            executable='leader_follower_navigation',
            name='test_leader',
            arguments=['leader', '0'],
            output='screen'
        ),
    ])