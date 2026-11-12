from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("rm_fire_control")) /
                 "config" / "fire_permission.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=config),
        Node(package="rm_fire_control", executable="fire_permission_node.py",
             name="fire_permission_preview", output="screen",
             parameters=[LaunchConfiguration("params_file")]),
    ])
