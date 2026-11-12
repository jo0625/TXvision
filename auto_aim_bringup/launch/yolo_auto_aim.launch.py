from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("auto_aim_bringup")

    yolo_params = LaunchConfiguration("yolo_params")
    tracker_params = LaunchConfiguration("tracker_params")
    image_raw = LaunchConfiguration("image_raw")
    camera_info = LaunchConfiguration("camera_info")
    tf_topic = LaunchConfiguration("tf_topic")
    tf_static_topic = LaunchConfiguration("tf_static_topic")
    start_tracker = LaunchConfiguration("start_tracker")
    start_target_watchdog = LaunchConfiguration("start_target_watchdog")
    publish_debug = LaunchConfiguration("publish_debug")
    target_timeout = LaunchConfiguration("target_timeout")
    raw_target_topic = LaunchConfiguration("raw_target_topic")
    safe_target_topic = LaunchConfiguration("safe_target_topic")
    start_static_tf = LaunchConfiguration("start_static_tf")
    static_tf_parent_frame = LaunchConfiguration("static_tf_parent_frame")
    static_tf_child_frame = LaunchConfiguration("static_tf_child_frame")
    static_tf_x = LaunchConfiguration("static_tf_x")
    static_tf_y = LaunchConfiguration("static_tf_y")
    static_tf_z = LaunchConfiguration("static_tf_z")
    static_tf_roll = LaunchConfiguration("static_tf_roll")
    static_tf_pitch = LaunchConfiguration("static_tf_pitch")
    static_tf_yaw = LaunchConfiguration("static_tf_yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "yolo_params",
                default_value=PathJoinSubstitution(
                    [bringup_share, "params", "yolo_detector_params.yaml"]
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
            DeclareLaunchArgument("tf_topic", default_value="/tf"),
            DeclareLaunchArgument("tf_static_topic", default_value="/tf_static"),
            DeclareLaunchArgument("start_tracker", default_value="true"),
            DeclareLaunchArgument("start_target_watchdog", default_value="true"),
            DeclareLaunchArgument("latency_trace_path", default_value=""),
            DeclareLaunchArgument(
                "publish_debug",
                default_value="true",
                description="Publish annotated detector images for interactive debugging",
            ),
            DeclareLaunchArgument("target_timeout", default_value="0.5"),
            DeclareLaunchArgument("raw_target_topic", default_value="/tracker/raw_target"),
            DeclareLaunchArgument("safe_target_topic", default_value="/tracker/target"),
            DeclareLaunchArgument("start_static_tf", default_value="false"),
            DeclareLaunchArgument("static_tf_parent_frame", default_value="odom"),
            DeclareLaunchArgument(
                "static_tf_child_frame", default_value="camera_optical_frame"
            ),
            DeclareLaunchArgument("static_tf_x", default_value="0"),
            DeclareLaunchArgument("static_tf_y", default_value="0"),
            DeclareLaunchArgument("static_tf_z", default_value="0"),
            # REP-103 optical (right/down/forward) -> odom (forward/left/up).
            DeclareLaunchArgument("static_tf_roll", default_value="-1.57079632679"),
            DeclareLaunchArgument("static_tf_pitch", default_value="0"),
            DeclareLaunchArgument("static_tf_yaw", default_value="-1.57079632679"),
            Node(
                package="yolo_armor_detector",
                executable="yolo_armor_detector_node.py",
                name="yolo_armor_detector",
                output="screen",
                parameters=[
                    yolo_params,
                    {
                        "image_topic": image_raw,
                        "camera_info_topic": camera_info,
                        "latency_trace_path": ParameterValue(
                            LaunchConfiguration("latency_trace_path"), value_type=str),
                        "publish_debug": ParameterValue(
                            publish_debug, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="armor_tracker",
                executable="armor_tracker_node",
                name="armor_tracker",
                output="screen",
                condition=IfCondition(start_tracker),
                parameters=[tracker_params],
                remappings=[
                    ("/tf", tf_topic),
                    ("/tf_static", tf_static_topic),
                    ("/tracker/target", raw_target_topic),
                ],
            ),
            Node(
                package="auto_aim_safety",
                executable="target_watchdog_node.py",
                name="target_watchdog",
                output="screen",
                condition=IfCondition(start_target_watchdog),
                parameters=[
                    {
                        "input_topic": raw_target_topic,
                        "output_topic": safe_target_topic,
                        "timeout": target_timeout,
                        "target_frame": static_tf_parent_frame,
                    }
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
        ]
    )
