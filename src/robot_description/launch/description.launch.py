"""Publish Caster's model without depending on the old r2_shop packages."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model = os.path.join(get_package_share_directory('robot_description'),
                         'description', 'caster.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = ParameterValue(Command([
        'xacro ', model,
        ' lslidar:=', LaunchConfiguration('lslidar'),
        ' rtvsensor:=', LaunchConfiguration('rtvsensor'),
        ' simulator:=false',
        ' lidar_view_direction:=', LaunchConfiguration('lidar_view_direction'),
        ' lidar_view_width:=', LaunchConfiguration('lidar_view_width'),
    ]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lslidar', default_value='true'),
        DeclareLaunchArgument('rtvsensor', default_value='false'),
        DeclareLaunchArgument('lidar_view_direction', default_value='0.0'),
        DeclareLaunchArgument('lidar_view_width', default_value='6.283185307179586'),
        DeclareLaunchArgument('publish_joint_states', default_value='true',
                              description='Publish joint states from ODrive feedback, with defaults for passive joints'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'robot_description': robot_description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', output='screen',
             condition=IfCondition(LaunchConfiguration('publish_joint_states')),
             parameters=[{'use_sim_time': use_sim_time,
                          'source_list': ['/odrive/joint_states']}]),
    ])
