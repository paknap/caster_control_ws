import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    teleop_path = get_package_share_directory("teleop_launch")
    odrive_path = get_package_share_directory("odrive_ros")

    odrive_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(odrive_path, 'launch', 'odrive.launch.xml')
        )
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(teleop_path, 'launch', 'joy_launch.py')),
        launch_arguments={'cmd_vel_topic': '/odrive/cmd_vel'}.items()
    )

    return LaunchDescription([
        odrive_launch,
        teleop_launch
    ])