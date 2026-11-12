from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rm_serial_driver",
                executable="gimbal_tf_broadcaster.py",
                name="gimbal_tf",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("rm_serial_driver"), "config", "gimbal_tf.yaml"]
                    )
                ],
            )
        ]
    )
