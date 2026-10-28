import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('teleop_launch')

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/joy_teleop/cmd_vel'),
        DeclareLaunchArgument(
            'config_filepath',
            default_value=os.path.join(pkg_share, 'params', 'joy.yaml'),
            description='Joystick config file'),

        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[LaunchConfiguration('config_filepath')]
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[LaunchConfiguration('config_filepath')],
            remappings=[('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
        ),
    ])
