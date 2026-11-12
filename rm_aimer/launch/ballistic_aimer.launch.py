"""Run the existing aimer with gravity compensation and explicit calibration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "projectile_speed", default_value="0.0",
            description="Configured speed in m/s (> 0) when no impact calibration file is given."
        ),
        DeclareLaunchArgument(
            "muzzle_frame", description="Existing TF frame at the physical projectile exit."
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("rm_aimer"), "config", "basic_aimer.yaml"
            ]),
            description="Base aimer parameters; ballistic arguments override this file.",
        ),
        DeclareLaunchArgument("speed_calibration_file", default_value=""),
        DeclareLaunchArgument("aim_frame", default_value="pitch_joint"),
        DeclareLaunchArgument("gravity_frame", default_value="odom"),
        DeclareLaunchArgument("gravity", default_value="9.81"),
        DeclareLaunchArgument("prediction_time", default_value="0.0"),
        DeclareLaunchArgument("output_topic", default_value="/aimer/aim_command"),
        DeclareLaunchArgument("publish_diagnostics", default_value="false"),
        Node(
            package="rm_aimer",
            executable="basic_aimer_node.py",
            name="aimer",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "ballistics_enabled": True,
                    "speed_calibration_file": ParameterValue(
                        LaunchConfiguration("speed_calibration_file"), value_type=str),
                    "publish_diagnostics": ParameterValue(
                        LaunchConfiguration("publish_diagnostics"), value_type=bool),
                    "projectile_speed": ParameterValue(
                        LaunchConfiguration("projectile_speed"), value_type=float
                    ),
                    "muzzle_frame": ParameterValue(
                        LaunchConfiguration("muzzle_frame"), value_type=str
                    ),
                    "aim_frame": ParameterValue(
                        LaunchConfiguration("aim_frame"), value_type=str
                    ),
                    "gravity_frame": ParameterValue(
                        LaunchConfiguration("gravity_frame"), value_type=str
                    ),
                    "gravity": ParameterValue(
                        LaunchConfiguration("gravity"), value_type=float
                    ),
                    "prediction_time": ParameterValue(
                        LaunchConfiguration("prediction_time"), value_type=float
                    ),
                    "output_topic": ParameterValue(
                        LaunchConfiguration("output_topic"), value_type=str
                    ),
                },
            ],
        ),
    ])
