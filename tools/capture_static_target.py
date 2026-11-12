#!/usr/bin/env python3
"""Capture the unchanged vision pipeline with the existing receive-only bridge."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'yolo_armor_detector/scripts'))
from stability_trace import BufferedTrace, stamp_ns


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--domain', type=int, default=198)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--warmup', type=float, default=15.0)
    parser.add_argument('--stationary-confirmed', action='store_true')
    args = parser.parse_args()
    if not args.stationary_confirmed or args.duration<30 or args.repeats<1 or args.warmup<0:
        parser.error('Confirm fixed camera/board and safely paused mechanics; duration >=30 s')
    if args.output.exists():
        parser.error('Use a new output directory')
    args.output.mkdir(parents=True)
    args.output = args.output.resolve()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
    # A local independent domain; never start the existing motion relay.
    os.environ.pop('CYCLONEDDS_URI', None)
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from auto_aim_interfaces.msg import Armors, AimCommand, Target, GimbalFeedback
    from sensor_msgs.msg import CameraInfo
    from tf2_msgs.msg import TFMessage

    trace = BufferedTrace(args.output/'pipeline.csv', capacity=200000)
    counts = Counter()
    stop = False
    def request_stop(*_):
        nonlocal stop
        stop = True
    rclpy.init()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = rclpy.create_node('static_target_observer')
    def receive(kind, msg):
        counts[kind] += 1
        common = dict(source_stamp_ns=stamp_ns(msg.header), frame_id=msg.header.frame_id)
        if kind in ('raw_target', 'safe_target'):
            trace.emit(kind, **common, target_id=msg.id, tracking=msg.tracking,
                x_m=msg.position.x,y_m=msg.position.y,z_m=msg.position.z,
                yaw_rad=msg.yaw, v_yaw=msg.v_yaw, armor_count=msg.armors_num,
                radius_1_m=msg.radius_1,radius_2_m=msg.radius_2,dz_m=msg.dz)
        elif kind in ('aimer_command','preview_command','bridge_input'):
            trace.emit(kind, **common, valid=msg.valid, yaw_rad=msg.yaw,pitch_rad=msg.pitch)
        elif kind=='feedback':
            trace.emit(kind, **common, feedback_sequence=msg.sequence,
                mcu_timestamp_ms=msg.mcu_timestamp_ms,yaw_rad=msg.yaw,pitch_rad=msg.pitch,
                quaternion_wxyz=[msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z],
                mode=msg.mode)
        elif kind=='armors':
            counts['nonempty_armors'] += bool(msg.armors)
            trace.emit(kind, **common, armor_count=len(msg.armors), armors=[dict(
                number=a.number, armor_type=a.type,
                position=[a.pose.position.x,a.pose.position.y,a.pose.position.z],
                quaternion=[a.pose.orientation.x,a.pose.orientation.y,a.pose.orientation.z,a.pose.orientation.w],
                semantic_corners=[[p.x,p.y] for p in a.kpts]) for a in msg.armors])
        else:
            trace.emit(kind, **common,width=msg.width,height=msg.height,k=list(msg.k),d=list(msg.d))
    subs = []
    for cls, topic, kind in [
        (CameraInfo,'/camera_info','camera_info'), (Armors,'/detector/armors','armors'),
        (Target,'/tracker/raw_target','raw_target'), (Target,'/tracker/target','safe_target'),
        (AimCommand,'/aimer/aim_command','aimer_command'),
        (AimCommand,'/controller_command_preview/command','preview_command'),
        (AimCommand,'/serial_bridge/aim_command','bridge_input'),
        (GimbalFeedback,'/serial_bridge/feedback','feedback')]:
        subs.append(node.create_subscription(cls,topic,lambda m,k=kind:receive(k,m),qos_profile_sensor_data))
    def receive_tf(msg):
        for tf in msg.transforms:
            trace.emit('tf',source_stamp_ns=stamp_ns(tf.header),frame_id=tf.header.frame_id,
                child=tf.child_frame_id,translation_m=[tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z],
                rotation_xyzw=[tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w])
    subs.append(node.create_subscription(TFMessage,'/tf',receive_tf,qos_profile_sensor_data))
    children=[]
    logs=[]
    commands=[]
    def start(name, executable, params, extra=(), interpreter='/usr/bin/python3'):
        command=([interpreter,str(executable)] if str(executable).endswith('.py') else [str(executable)])
        command+=['--ros-args']
        for param in params:
            command+=['--params-file',str(ROOT/param)]
        command+=list(extra)
        log=(args.output/(name+'.log')).open('w')
        logs.append(log)
        commands.append(dict(name=name,args=command))
        children.append((name,subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)))
    windows=[]
    error=None
    def spin_until(deadline):
        while not stop and time.monotonic()<deadline:
            for name,proc in children:
                if proc.poll() is not None:
                    raise RuntimeError(f'{name} exited {proc.returncode}; see its log')
            rclpy.spin_once(node,timeout_sec=0.05)
    try:
        serial=ROOT/'install/rm_serial_driver/lib/rm_serial_driver'
        aimer=ROOT/'install/rm_aimer/lib/rm_aimer'
        start('serial',serial/'serial_bridge_node.py',['rm_serial_driver/config/serial_bridge.yaml'],
            ['-p','transport:=serial','-p',f'port:={args.port}','-p','receive_only:=true',
             '-p',f'latency_trace_path:={args.output}/serial.csv'])
        start('tf',serial/'gimbal_tf_broadcaster.py',['rm_serial_driver/config/gimbal_tf_provisional_handeye.yaml'])
        start('tracker',ROOT/'install/armor_tracker/lib/armor_tracker/armor_tracker_node',
              ['auto_aim_bringup/params/tracker_params.yaml'],['-r','/tracker/target:=/tracker/raw_target'])
        start('watchdog',ROOT/'install/auto_aim_safety/lib/auto_aim_safety/target_watchdog_node.py',[],
              ['-p','input_topic:=/tracker/raw_target','-p','output_topic:=/tracker/target'])
        start('aimer',aimer/'basic_aimer_node.py',
            ['rm_aimer/config/basic_aimer.yaml','rm_aimer/config/controller_preview.yaml'],
            ['-p',f'stability_trace_path:={args.output}/aimer.csv'])
        start('preview',aimer/'controller_command_preview.py',['rm_aimer/config/controller_preview.yaml'],
            ['-p',f'stability_trace_path:={args.output}/preview.csv'])
        detector=ROOT/'install/yolo_armor_detector/lib/yolo_armor_detector/yolo_armor_detector_node.py'
        os.environ['YOLO_ARMOR_MODEL']=str(ROOT/'deployment/yolo_v8_gray140_candidate_20260826/model.pt')
        start('detector',detector,['auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml'],
            ['-p','publish_debug:=false','-p','diagnostics_period:=1000000',
             '-p',f'stability_trace_path:={args.output}/detector.csv'],
            interpreter=str(ROOT.parent/'.venv/bin/python'))
        start('camera',ROOT/'install/hik_camera/lib/hik_camera/hik_camera_node',
            ['hik_camera/config/camera_params.yaml'],['-p','publish_bayer:=true',
                '-p',f'stability_trace_path:={args.output}/camera.csv'])
        spin_until(time.monotonic()+args.warmup)
        if not counts['camera_info'] or not counts['armors']:
            raise RuntimeError('Camera or detector did not produce data during warmup')
        if not counts['nonempty_armors']:
            raise RuntimeError('No valid armor during warmup; capture is incomplete, check target before a formal trial')
        for run in range(1,args.repeats+1):
            if stop:
                break
            begin=time.monotonic_ns()
            trace.emit('run_start',run=run,host_ns=begin)
            print(json.dumps(dict(event='run_start',run=run,output=str(args.output))),flush=True)
            spin_until(time.monotonic()+args.duration)
            end=time.monotonic_ns()
            windows.append(dict(run=run,start_ns=begin,end_ns=end,duration_s=(end-begin)/1e9,
                                complete=(end-begin)>=args.duration*1e9))
            trace.emit('run_end',run=run,host_ns=end)
    except Exception as exc:
        error=str(exc)
    finally:
        # Direct child executables receive one SIGINT; no launch forwarding double signal.
        for _,proc in reversed(children):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        for name,proc in reversed(children):
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                proc.terminate()
                proc.wait(timeout=5)
        trace.context.update(windows=windows,counts=dict(counts),stationary_user_confirmed=True,
                             mechanics='user confirmed paused; host bridge receive_only=true',
                             actual_sends='none requested; actual sending chain not exercised',error=error)
        trace.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for log in logs:
            log.close()
        config_files=['hik_camera/config/camera_params.yaml',
            'auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml',
            'auto_aim_bringup/params/tracker_params.yaml','rm_aimer/config/basic_aimer.yaml',
            'rm_aimer/config/controller_preview.yaml','rm_serial_driver/config/gimbal_tf_provisional_handeye.yaml',
            'rm_serial_driver/config/serial_bridge.yaml']
        snapshots=args.output/'configuration'
        snapshots.mkdir()
        hashes={}
        for name in config_files:
            content=(ROOT/name).read_bytes()
            (snapshots/(name.replace('/','__'))).write_bytes(content)
            hashes[name]=hashlib.sha256(content).hexdigest()
        manifest=dict(schema=1,windows=windows,error=error,counts=dict(counts),
            domain=args.domain,commands=commands,configuration_sha256=hashes,
            process_exit_codes={name:proc.returncode for name,proc in children},
            stationary_user_confirmed=True,bridge_receive_only=True,new_motion_commands=0)
        (args.output/'capture.json').write_text(json.dumps(manifest,indent=2)+'\n')
        print(json.dumps(manifest,indent=2),flush=True)
    return 1 if error else 0


if __name__=='__main__':
    raise SystemExit(main())
