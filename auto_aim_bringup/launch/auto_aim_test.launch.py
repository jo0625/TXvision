from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
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
    static_tf_x = LaunchConfiguration("static_tf_x")
    static_tf_y = LaunchConfiguration("static_tf_y")
    static_tf_z = LaunchConfiguration("static_tf_z")
    static_tf_roll = LaunchConfiguration("static_tf_roll")
    static_tf_pitch = LaunchConfiguration("static_tf_pitch")
    static_tf_yaw = LaunchConfiguration("static_tf_yaw")
    static_tf_parent_frame = LaunchConfiguration("static_tf_parent_frame")
    static_tf_child_frame = LaunchConfiguration("static_tf_child_frame")
    start_static_tf = LaunchConfiguration("start_static_tf")
    start_test_image_publisher = LaunchConfiguration("start_test_image_publisher")
    test_image_script = LaunchConfiguration("test_image_script")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector_params",
                default_value=PathJoinSubstitution(
                    [bringup_share, "params", "test_images", "detector_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "tracker_params",
                default_value=PathJoinSubstitution(
                    [bringup_share, "params", "test_images", "tracker_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("image_raw", default_value="/image_raw"),
            DeclareLaunchArgument("camera_info", default_value="/camera_info"),
            DeclareLaunchArgument("task_mode", default_value="/task_mode"),
            DeclareLaunchArgument("tf_topic", default_value="/tf"),
            DeclareLaunchArgument("tf_static_topic", default_value="/tf_static"),
            DeclareLaunchArgument("start_static_tf", default_value="true"),
            DeclareLaunchArgument("static_tf_x", default_value="0"),
            DeclareLaunchArgument("static_tf_y", default_value="0"),
            DeclareLaunchArgument("static_tf_z", default_value="0"),
            DeclareLaunchArgument("static_tf_roll", default_value="0"),
            DeclareLaunchArgument("static_tf_pitch", default_value="0"),
            DeclareLaunchArgument("static_tf_yaw", default_value="0"),
            DeclareLaunchArgument("static_tf_parent_frame", default_value="odom"),
            DeclareLaunchArgument("static_tf_child_frame", default_value="camera_optical_frame"),
            DeclareLaunchArgument("start_test_image_publisher", default_value="true"),
            DeclareLaunchArgument(
                "test_image_script",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "ros_ws", "test_tools", "publish_test_images.py"]
                ),
            ),
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
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="test_camera_static_tf",
                output="screen",
                condition=IfCondition(start_static_tf),
                arguments=[
                    "--x",
                    static_tf_x,
                    "--y",
                    static_tf_y,
                    "--z",
                    static_tf_z,
                    "--roll",
                    static_tf_roll,
                    "--pitch",
                    static_tf_pitch,
                    "--yaw",
                    static_tf_yaw,
                    "--frame-id",
                    static_tf_parent_frame,
                    "--child-frame-id",
                    static_tf_child_frame,
                ],
            ),
            ExecuteProcess(
                cmd=["python3", test_image_script],
                output="screen",
                condition=IfCondition(start_test_image_publisher),
            ),
        ]
    )
