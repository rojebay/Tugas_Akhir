#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Teleop twist keyboard
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        parameters=[{'use_sim_time': True}],
        output='screen',
        prefix='gnome-terminal -- '  # Run in separate terminal
    )
    
    return LaunchDescription([
        teleop_keyboard
    ])

# #!/usr/bin/env python3

# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
    
#     # Teleop twist keyboard
#     teleop_keyboard = Node(
#         package='teleop_twist_keyboard',
#         executable='teleop_twist_keyboard',
#         name='teleop_twist_keyboard',
#         parameters=[{'use_sim_time': True}],
#         output='screen',
#         # prefix='xterm -e'  # Run in separate terminal
#     )
    
#     return LaunchDescription([
#         teleop_keyboard
#     ])