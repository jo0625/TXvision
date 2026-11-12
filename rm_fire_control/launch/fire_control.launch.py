from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory("rm_fire_control")) / "config" / "fire_control.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("publish_diagnostics", default_value="false"),
        DeclareLaunchArgument("calibration_verified", default_value="true"),
        Node(package="rm_fire_control", executable="fire_permission_node.py",
             name="fire_permission_control", output="screen",
             parameters=[config, {
                 "enabled": ParameterValue(LaunchConfiguration("enabled"), value_type=bool),
                 "calibration_verified": ParameterValue(LaunchConfiguration("calibration_verified"), value_type=bool)}]),
        Node(package="rm_serial_driver", executable="serial_bridge_node.py",
             name="serial_bridge", output="screen",
             parameters=[config, {"port": LaunchConfiguration("port"),
                                  "require_fire_permission": True, "receive_only": False,
                                  "publish_diagnostics": ParameterValue(
                                      LaunchConfiguration("publish_diagnostics"), value_type=bool)}]),
    ])
