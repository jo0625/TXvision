from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("latency_trace_path", default_value=""),
            DeclareLaunchArgument("latency_protocol", default_value="legacy"),
            Node(
                package="rm_serial_driver",
                executable="serial_bridge_node.py",
                name="serial_bridge",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("rm_serial_driver"), "config", "serial_bridge.yaml"]
                    ),
                    {"latency_trace_path": ParameterValue(
                        LaunchConfiguration("latency_trace_path"), value_type=str),
                     "latency_protocol": ParameterValue(
                        LaunchConfiguration("latency_protocol"), value_type=str)},
                ],
            )
        ]
    )
