from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rm_aimer",
                executable="basic_aimer_node.py",
                name="aimer",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("rm_aimer"), "config", "basic_aimer.yaml"]
                    )
                ],
            )
        ]
    )
