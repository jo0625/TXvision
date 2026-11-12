from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("auto_aim_bringup")
    camera_share = FindPackageShare("hik_camera")
    project_root = LaunchConfiguration("project_root")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=str(Path(__file__).resolve().parents[2]),
            ),
            SetEnvironmentVariable(
                "PATH",
                [
                    str(Path(__file__).resolve().parents[3] / ".venv" / "bin") + ":",
                    EnvironmentVariable("PATH"),
                ],
            ),
            SetEnvironmentVariable(
                "YOLO_ARMOR_MODEL",
                PathJoinSubstitution(
                    [
                        project_root,
                        "deployment",
                        "yolo_v8_gray140_candidate_20260826",
                        "model.pt",
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [camera_share, "launch", "hik_camera.launch.py"]
                    )
                ),
                launch_arguments={
                    "publish_bayer": "true",
                    "params_file": PathJoinSubstitution(
                        [camera_share, "config", "camera_params.yaml"]
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "yolo_auto_aim.launch.py"]
                    )
                ),
                launch_arguments={
                    "yolo_params": PathJoinSubstitution(
                        [
                            bringup_share,
                            "params",
                            "yolo_detector_params_v8_frozen_candidate.yaml",
                        ]
                    ),
                    "tracker_params": PathJoinSubstitution(
                        [bringup_share, "params", "tracker_params.yaml"]
                    ),
                    "publish_debug": "true",
                    "start_static_tf": "true",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="vision_display_rviz",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [bringup_share, "rviz", "vision_display_test.rviz"]
                    ),
                ],
            ),
        ]
    )
