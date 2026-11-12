from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("hik_camera")
    params_file = LaunchConfiguration("params_file")
    publish_bayer = LaunchConfiguration("publish_bayer")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "camera_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "publish_bayer",
                default_value="false",
                description=(
                    "Publish bayer_rggb8 instead of bgr8 to reduce image transport "
                    "bandwidth; the subscriber must perform Bayer conversion"
                ),
            ),
            Node(
                package="hik_camera",
                executable="hik_camera_node",
                name="hik_camera",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "publish_bayer": ParameterValue(
                            publish_bayer, value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
