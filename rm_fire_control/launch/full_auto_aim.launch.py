"""Complete native pipeline; optical source and firing switches are explicit."""
from pathlib import Path
import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _preflight(context):
    value=lambda key: LaunchConfiguration(key).perform(context)
    required=[value("tf_params"),value("controller_params"),value("tracker_params")]
    nodes=[("rm_serial_driver","serial_bridge_node.py"),
           ("rm_serial_driver","gimbal_tf_broadcaster.py"),
           ("rm_fire_control","fire_permission_node.py"),
           ("rm_aimer","basic_aimer_node.py"),
           ("rm_aimer","controller_command_preview.py"),
           ("armor_tracker","armor_tracker_node"),
           ("auto_aim_safety","target_watchdog_node.py")]
    if value("record_run").lower()=="true":
        nodes.append(("rm_fire_control", "aim_run_recorder.py"))
    if value("start_camera").lower()=="true":nodes.append(("hik_camera","hik_camera_node"))
    if value("start_detector").lower()=="true":
        required.extend([value("detector_python"),value("model_path"),value("yolo_params")])
        nodes.append(("yolo_armor_detector","yolo_armor_detector_node.py"))
        if not os.access(value("detector_python"),os.X_OK):
            raise RuntimeError("Detector Python is not executable: "+value("detector_python"))
    for name in required:
        if not Path(name).is_file():raise RuntimeError("Required file not found: "+name)
    for package,name in nodes:
        path=Path(get_package_prefix(package))/"lib"/package/name
        if not path.is_file() or not os.access(path,os.X_OK):
            raise RuntimeError("Required node is not executable: "+str(path))
    return []


def _stop_on_exit(event, context):
    if not context.is_shutdown:
        return [EmitEvent(event=Shutdown(reason="A pipeline node exited; shutting down the complete pipeline"))]
    return []


def generate_launch_description():
    share=lambda pkg: Path(get_package_share_directory(pkg))
    cfg=lambda name: LaunchConfiguration(name)
    typed=lambda name,t: ParameterValue(cfg(name),value_type=t)
    include=lambda pkg,file,args: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share(pkg)/"launch"/file)),launch_arguments=args.items())
    project=cfg("project_root")
    return LaunchDescription([
        DeclareLaunchArgument("project_root",default_value=str(Path(__file__).resolve().parents[2])),
        DeclareLaunchArgument("port",default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("enabled",default_value="false",description="Allow angle transmission when the fire policy is satisfied"),
        DeclareLaunchArgument("calibration_verified",default_value="true"),
        DeclareLaunchArgument("projectile_speed",default_value="14.7",description="User-accepted preliminary impact-speed estimate, m/s"),
        DeclareLaunchArgument("muzzle_frame",default_value="muzzle"),
        DeclareLaunchArgument("start_camera",default_value="true"),
        DeclareLaunchArgument("start_detector",default_value="true"),
        DeclareLaunchArgument("publish_debug",default_value="true"),
        DeclareLaunchArgument("record_run", default_value="true",
                              description="Save target, ballistics, feedback and fire decisions"),
        DeclareLaunchArgument("run_log_root",
                              default_value=PathJoinSubstitution([project, "test_results", "aim_runs"])),
        DeclareLaunchArgument("run_log_max_mb", default_value="512"),
        DeclareLaunchArgument("detector_python",default_value=PathJoinSubstitution([project,"..",".venv","bin","python"])),
        DeclareLaunchArgument("model_path",default_value=PathJoinSubstitution([project,"deployment","yolo_v8_gray140_candidate_20260826","model.pt"])),
        DeclareLaunchArgument("yolo_params",default_value=str(share("auto_aim_bringup")/"params"/"yolo_detector_params_v8_frozen_candidate.yaml")),
        DeclareLaunchArgument("tracker_params",default_value=str(share("auto_aim_bringup")/"params"/"tracker_params.yaml")),
        DeclareLaunchArgument("tf_params",default_value=str(share("rm_serial_driver")/"config"/"gimbal_tf_provisional_handeye.yaml")),
        DeclareLaunchArgument("controller_params",default_value=str(share("rm_aimer")/"config"/"controller_preview.yaml")),
        OpaqueFunction(function=_preflight),
        RegisterEventHandler(OnProcessExit(on_exit=_stop_on_exit)),
        Node(package="rm_fire_control", executable="aim_run_recorder.py",
             name="aim_run_recorder", output="screen", condition=IfCondition(cfg("record_run")),
             parameters=[{"output_root": typed("run_log_root", str),
                          "max_log_mb": typed("run_log_max_mb", int)}]),
        include("rm_fire_control","fire_control.launch.py",{
            "port":cfg("port"),"enabled":cfg("enabled"),"calibration_verified":cfg("calibration_verified"),
            "publish_diagnostics":cfg("record_run")}),
        Node(package="rm_serial_driver",executable="gimbal_tf_broadcaster.py",name="gimbal_tf",output="screen",parameters=[cfg("tf_params")]),
        include("rm_aimer","ballistic_aimer.launch.py",{
            "projectile_speed":cfg("projectile_speed"),"muzzle_frame":cfg("muzzle_frame"),
            "aim_frame":"pitch_joint","gravity_frame":"odom","output_topic":"/aimer/aim_command",
            "publish_diagnostics":cfg("record_run")}),
        Node(package="rm_aimer",executable="controller_command_preview.py",name="controller_command_preview",
             output="screen",parameters=[cfg("controller_params"),
                                        {"publish_diagnostics":typed("record_run", bool)}]),
        Node(package="armor_tracker",executable="armor_tracker_node",name="armor_tracker",output="screen",
             parameters=[cfg("tracker_params")],remappings=[("/tracker/target","/tracker/raw_target")]),
        Node(package="auto_aim_safety",executable="target_watchdog_node.py",name="target_watchdog",output="screen",
             parameters=[{"input_topic":"/tracker/raw_target","output_topic":"/tracker/target","timeout":0.2,"target_frame":"odom"}]),
        Node(package="yolo_armor_detector",executable="yolo_armor_detector_node.py",name="yolo_armor_detector",output="screen",
             prefix=[cfg("detector_python")],condition=IfCondition(cfg("start_detector")),
             parameters=[cfg("yolo_params"),{"model":typed("model_path",str),"publish_debug":typed("publish_debug",bool)}]),
        Node(package="hik_camera",executable="hik_camera_node",name="hik_camera",output="screen",
             condition=IfCondition(cfg("start_camera")),
             parameters=[str(share("hik_camera")/"config"/"camera_params.yaml"),{"publish_bayer":True}]),
    ])
