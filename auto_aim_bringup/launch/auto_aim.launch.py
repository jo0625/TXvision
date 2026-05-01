from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("auto_aim_bringup")

    detector_params = LaunchConfiguration("detector_params")
    tracker_params = LaunchConfiguration("tracker_params")
    image_raw = LaunchConfiguration("image_raw")
    camera_info = LaunchConfiguration("camera_info")
    task_mode = LaunchConfiguration("task_mode")
    tf_topic = LaunchConfiguration("tf_topic")
    tf_static_topic = LaunchConfiguration("tf_static_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector_params",
                default_value=PathJoinSubstitution(
                    [bringup_share, "params", "detector_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "tracker_params",
                default_value=PathJoinSubstitution(
                    [bringup_share, "params", "tracker_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("image_raw", default_value="/image_raw"),
            DeclareLaunchArgument("camera_info", default_value="/camera_info"),
            DeclareLaunchArgument("task_mode", default_value="/task_mode"),
            DeclareLaunchArgument("tf_topic", default_value="/tf"),
            DeclareLaunchArgument("tf_static_topic", default_value="/tf_static"),
            Node(
                package="armor_detector",
                executable="armor_detector_node",
                name="armor_detector",
                output="screen",
                parameters=[detector_params],
                remappings=[
                    ("/image_raw", image_raw),
                    ("/camera_info", camera_info),
                    ("/task_mode", task_mode),
                ],
            ),
            Node(
                package="armor_tracker",
                executable="armor_tracker_node",
                name="armor_tracker",
                output="screen",
                parameters=[tracker_params],
                remappings=[
                    ("/tf", tf_topic),
                    ("/tf_static", tf_static_topic),
                ],
            ),
        ]
    )
