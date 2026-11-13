#!/usr/bin/env python3
"""Capture host latency using existing nodes and bounded dynamic_follow_test relay.
No diagnostic wire protocol, control parameter or PID changes.
"""
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

SETTINGS = json.loads(Path(__file__).with_name('runtime_paths.json').read_text())
ROOT = Path(SETTINGS['runtime_root'])
DEPENDENCY_ROOT = Path(SETTINGS['dependency_root'])
TF_PARAMS = Path(SETTINGS['tf_params'])
TRACKER_PARAMS = Path(SETTINGS['tracker_params'])
AIMER_PARAMS = Path(__file__).parent / SETTINGS['aimer_params']
PREVIEW_PARAMS = Path(__file__).parent / SETTINGS['preview_params']
sys.path.insert(0, str(ROOT / 'yolo_armor_detector/scripts'))
from stability_trace import BufferedTrace, stamp_ns


def main():
    parser = argparse.ArgumentParser(description='Post-fix real-chain validation; explicit receive-only or gated motion')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--domain', type=int, default=198)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--warmup', type=float, default=15.0)
    parser.add_argument('--receive-only', action='store_true')
    parser.add_argument('--boundaries', action='store_true')
    parser.add_argument('--debugger', action='store_true', help='Own tracker with gdb; collect stack after observation window, then stop that inferior')
    parser.add_argument('--trace-off', action='store_true', help='Disable component trace hooks; retain the same independent observer')
    parser.add_argument('--expected-tf2', type=Path, required=True)
    parser.add_argument('--motion', action='store_true')
    parser.add_argument('--static-gate', type=Path)
    parser.add_argument('--prediction-time', type=float, default=None)
    args = parser.parse_args()
    if args.motion and args.prediction_time is None:
        parser.error('Motion requires explicit prediction-time; no unvalidated horizon is guessed')
    if args.prediction_time is None:
        args.prediction_time = 0.0
    if not 0 <= args.prediction_time < 1:
        parser.error('prediction-time must be finite and within [0, 1) seconds')
    if args.receive_only == args.motion:
        parser.error('Select exactly one of --receive-only or --motion')
    if args.motion:
        if not args.static_gate or not json.loads(args.static_gate.read_text()).get('motion_permitted'):
            parser.error('Motion requires recorded passing static verification gate')
    if args.duration<10 or args.repeats!=1 or args.warmup<0:
        parser.error('Use one bounded capture with duration >=10 s')
    if args.output.exists():
        parser.error('Use a new output directory')
    args.output.mkdir(parents=True)
    args.output = args.output.resolve()
    (args.output/'host_version.txt').write_text(subprocess.check_output(['uname','-a'],text=True)+subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True))
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
    # Isolate this capture from unrelated ROS command publishers.
    os.environ.pop('CYCLONEDDS_URI', None)
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from auto_aim_interfaces.msg import Armors, AimCommand, Target, GimbalFeedback
    from sensor_msgs.msg import CameraInfo
    from tf2_msgs.msg import TFMessage

    trace = BufferedTrace(args.output/'pipeline.csv', capacity=500000)
    counts = Counter()
    latest = {}
    stop = False
    def request_stop(*_):
        nonlocal stop
        stop = True
    rclpy.init()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = rclpy.create_node('full_chain_observer')
    def receive(kind, msg):
        counts[kind] += 1
        latest[kind] = (time.monotonic(), msg)
        common = dict(source_stamp_ns=stamp_ns(msg.header), frame_id=msg.header.frame_id, observer_ros_ns=node.get_clock().now().nanoseconds)
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
        if args.trace_off:
            extra = [v for i,v in enumerate(extra) if not (v.startswith(('stability_trace_path:=', 'latency_trace_path:=')) or (v=='-p' and i+1<len(extra) and extra[i+1].startswith(('stability_trace_path:=','latency_trace_path:='))))]
        if name == 'aimer':
            executable = Path(__file__).with_name('aimer_validation.py')
        command=([interpreter,str(executable)] if str(executable).endswith('.py') else [str(executable)])
        command+=['--ros-args']
        for param in params:
            command+=['--params-file',str(ROOT/param)]
        command+=list(extra)
        overrides = {'serial': ['command_rate:=200.0', 'latency_trace_capacity:=500000'],
                     'camera': ['frame_rate:=240.0'],
                     'aimer': [f'prediction_time:={args.prediction_time}']}
        for value in overrides.get(name, []):
            command += ['-p', value]
        if args.boundaries and not args.trace_off and name in ('tracker','detector'):
            command += ['-p', 'diagnostic_boundaries:=true']
        if name == 'tracker' and not args.debugger:
            command = ['/usr/bin/python3', '/home/jo/Documents/Codex/2026-10-15/pid-sdk-pnp-11-usb-sdk/outputs/post_fix_baseline_20261016/tracker_exec.py'] + command
        if args.debugger and name == 'tracker':
            command = ['gdb', '--batch', '-ex', 'set pagination off',
                       '-ex', 'set confirm off', '-ex', 'set debuginfod enabled off',
                       '-ex', 'handle SIGINT stop print nopass', '-ex', 'run',
                       '-ex', 'thread apply all bt',
                       '-ex', 'source ' + str(Path(__file__).with_name('lock_owners.gdb')),
                       '-ex', 'kill', '-ex', 'quit', '--args'] + command
        log=(args.output/(name+'.log')).open('w')
        logs.append(log)
        commands.append(dict(name=name,args=command))
        child_env=os.environ.copy()
        if name == 'aimer':
            child_env.update(AIMER_VALIDATION_TOPOLOGY='dedicated', AIMER_VALIDATION_THREADS='1', AIMER_VALIDATION_DIRECTORY=str(args.output), AIMER_VALIDATION_BOUNDARIES='0' if args.trace_off else '1')
        children.append((name,subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=child_env)))
        (args.output/'launch_commands.json').write_text(json.dumps(commands,indent=2))
        (args.output/'processes.json').write_text(json.dumps([dict(name=n,pid=p.pid) for n,p in children],indent=2))
    def verify_library(stage):
        proc = next(p for n,p in children if n == 'tracker')
        maps = Path(f'/proc/{proc.pid}/maps').read_text()
        (args.output/f'tracker_maps_{stage}.txt').write_text(maps)
        paths = sorted({line.split()[-1] for line in maps.splitlines() if '/libtf2' in line})
        records = {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}
        (args.output/f'loaded_libraries_{stage}.json').write_text(json.dumps(records,indent=2))
        expected = args.expected_tf2.resolve()
        loaded = [Path(p).resolve() for p in paths if Path(p).name == 'libtf2.so']
        lock=json.loads((DEPENDENCY_ROOT/'outputs/tf_deadlock_fix/dependency_lock.json').read_text())
        if not any(value==next(iter(lock['overlay_files'].values())) and Path(key).resolve()==expected for key,value in records.items()):
            raise RuntimeError('Runtime tf2 hash differs from reviewed patch')
        if loaded != [expected]:
            raise RuntimeError(f'Wrong tf2 library: {loaded}, expected {expected}')
    windows=[]
    error=None
    cleanup_errors=[]
    relay=None
    monitoring=False
    def save_stall_evidence():
        if relay is not None and relay.poll() is None:
            relay.send_signal(signal.SIGINT)
            relay.wait(timeout=3)
        proc=next(p for n,p in children if n=='tracker')
        snap=dict(host_ns=time.monotonic_ns(),pid=proc.pid,reason='raw_target missing >0.5s while nonempty Armors fresh; not yet a deadlock diagnosis')
        try:
            command=['gdb','--batch','-ex','set pagination off','-ex','set confirm off',
                     '-ex','set debuginfod enabled off','-ex',f'attach {proc.pid}',
                     '-ex','thread apply all bt', '-ex','source '+str(DEPENDENCY_ROOT/'outputs/pnp_stall_diagnosis/lock_owners.gdb'),'-ex','detach']
            with (args.output/'stall_stack.log').open('w') as stream:
                result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,timeout=15)
            snap['debugger_exit_code']=result.returncode
        except Exception as exc:
            snap['error']=str(exc)
        (args.output/'stall_capture.json').write_text(json.dumps(snap,indent=2))
    def spin_until(deadline):
        while not stop and time.monotonic()<deadline:
            for name,proc in children:
                if proc.poll() is not None and not (name=='relay' and proc.returncode==0):
                    raise RuntimeError(f'{name} exited {proc.returncode}; see its log')
            rclpy.spin_once(node,timeout_sec=0.05)
            if monitoring:
                now=time.monotonic()
                arm_t,arm=latest.get('armors',(0,None))
                raw_t,_=latest.get('raw_target',(0,None))
                if arm is not None and arm.armors and now-arm_t<.2 and now-raw_t>.5:
                    trace.emit('stall_detected', raw_gap_s=now-raw_t, armors_gap_s=now-arm_t)
                    save_stall_evidence()
                    raise RuntimeError('Output stall: evidence captured; subsequent motion prohibited')
    try:
        serial=ROOT/'install/rm_serial_driver/lib/rm_serial_driver'
        aimer=ROOT/'install/rm_aimer/lib/rm_aimer'
        start('serial',serial/'serial_bridge_node.py',['rm_serial_driver/config/serial_bridge.yaml'],
            ['-p','transport:=serial','-p',f'port:={args.port}','-p',f'receive_only:={str(args.receive_only).lower()}','-p','latency_protocol:=legacy',
             '-p',f'latency_trace_path:={args.output}/serial.csv'])
        start('tf',serial/'gimbal_tf_broadcaster.py',[TF_PARAMS])
        start('tracker',ROOT/'install/armor_tracker/lib/armor_tracker/armor_tracker_node',
              [TRACKER_PARAMS],['-r','/tracker/target:=/tracker/raw_target','-p',f'latency_trace_path:={args.output}/tracker.csv'])
        start('watchdog',ROOT/'install/auto_aim_safety/lib/auto_aim_safety/target_watchdog_node.py',[],
              ['-p','input_topic:=/tracker/raw_target','-p','output_topic:=/tracker/target'])
        start('aimer',aimer/'basic_aimer_node.py',
            [AIMER_PARAMS, PREVIEW_PARAMS],
            ['-p',f'stability_trace_path:={args.output}/aimer.csv'])
        start('preview',aimer/'controller_command_preview.py',[PREVIEW_PARAMS],
            ['-p',f'stability_trace_path:={args.output}/preview.csv'])
        detector=ROOT/'install/yolo_armor_detector/lib/yolo_armor_detector/yolo_armor_detector_node.py'
        os.environ['YOLO_ARMOR_MODEL']=str(ROOT/'deployment/yolo_v8_gray140_candidate_20260826/model.pt')
        start('detector',detector,['auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml'],
            ['-p','publish_debug:=false','-p','diagnostics_period:=1000000',
             '-p',f'latency_trace_path:={args.output}/pnp.csv'],
            interpreter=str(ROOT.parent/'.venv/bin/python'))
        start('camera',ROOT/'install/hik_camera/lib/hik_camera/hik_camera_node',
            ['hik_camera/config/camera_params.yaml'],['-p','publish_bayer:=true',
                '-p',f'stability_trace_path:={args.output}/camera.csv'])
        warm_start=time.monotonic_ns()
        spin_until(time.monotonic()+args.warmup)
        (args.output/'warmup.json').write_text(json.dumps(dict(start_ns=warm_start,end_ns=time.monotonic_ns(),configured_s=args.warmup)))
        verify_library('warmup')
        if not counts['camera_info'] or not counts['armors']:
            raise RuntimeError('Camera or detector did not produce data during warmup')
        if not counts['nonempty_armors']:
            raise RuntimeError('No valid armor during warmup; capture is incomplete, check target before a formal trial')
        if args.receive_only:
            feedback_time, feedback_message = latest.get('feedback', (0, None))
            if feedback_message is None or time.monotonic() - feedback_time > .15:
                raise RuntimeError('No fresh MCU feedback during warmup; read-only tracker diagnosis lacks its dynamic TF input. Check serial.log and device access.')
        if stop:
            raise RuntimeError('Capture interrupted before relay start')
        if not args.receive_only:
            fb_time, fb = latest.get('feedback', (0, None))
            pv_time, pv = latest.get('preview_command', (0, None))
            if fb is None or time.monotonic()-fb_time > .15 or fb.mode != 6:
                raise RuntimeError('Fresh vision-mode (6) feedback required; relay not started')
            if pv is None or time.monotonic()-pv_time > .1 or not pv.valid:
                raise RuntimeError('No fresh valid preview command; relay not started')
            # This is the existing relay, with all existing validity checks and user-authorized 60/30 deg
            # excursion limits. Never enable its --no-angle-limits option here.
            relay_command = ['/usr/bin/python3', str(ROOT/'tools/dynamic_follow_test.py'),
                             '--duration', str(args.duration), '--start-delay', '1',
                             '--max-yaw-excursion-deg','60','--max-pitch-excursion-deg','30',
                             '--output', str(args.output/'relay.json')]
            relay_log = (args.output/'relay.log').open('w')
            logs.append(relay_log)
            commands.append(dict(name='relay',args=relay_command))
            relay = subprocess.Popen(relay_command,stdout=relay_log,stderr=subprocess.STDOUT)
            children.append(('relay', relay))
            (args.output/'launch_commands.json').write_text(json.dumps(commands,indent=2))
            print('Existing bounded relay started; command_rate override 200 Hz, legacy 11 bytes',flush=True)
        monitoring=True
        for run in range(1,args.repeats+1):
            if stop:
                break
            begin=time.monotonic_ns()
            trace.emit('run_start',run=run,host_ns=begin)
            (args.output/'window_start.json').write_text(json.dumps(dict(start_ns=begin)))
            print(json.dumps(dict(event='run_start',run=run,output=str(args.output))),flush=True)
            spin_until(time.monotonic()+args.duration)
            # Let the existing relay finish its own duration and flush without a
            # racing SIGINT. Startup time is included only in the run window.
            if relay is not None and not stop:
                deadline=time.monotonic()+5
                while relay.poll() is None and time.monotonic()<deadline and not stop:
                    spin_until(time.monotonic()+.05)
                if relay.poll() is None:
                    raise RuntimeError('Relay did not finish within its bounded window')
                if relay.returncode != 0:
                    raise RuntimeError('Relay exited with error')
            end=time.monotonic_ns()
            windows.append(dict(run=run,start_ns=begin,end_ns=end,duration_s=(end-begin)/1e9,
                                complete=(end-begin)>=args.duration*1e9))
            trace.emit('run_end',run=run,host_ns=end)
            (args.output/'window_end.json').write_text(json.dumps(dict(end_ns=end)))
        verify_library('end')
    except Exception as exc:
        error=str(exc)
    finally:
        if (args.output/'window_start.json').exists() and not windows:
            begin=json.loads((args.output/'window_start.json').read_text())['start_ns']
            end=time.monotonic_ns()
            windows.append(dict(run=1,start_ns=begin,end_ns=end,duration_s=(end-begin)/1e9,complete=False))
            (args.output/'window_end.json').write_text(json.dumps(dict(end_ns=end)))
        if args.debugger:
            for name, proc in children:
                if name != 'tracker' or proc.poll() is not None:
                    continue
                diagnostic = dict(host_ns=time.monotonic_ns(), debugger_pid=proc.pid,
                                  method='SIGINT debugger-owned tracker after observation; stack then kill inferior')
                try:
                    child_ids = Path(f'/proc/{proc.pid}/task/{proc.pid}/children').read_text().split()
                    diagnostic['inferior_pids'] = [int(pid) for pid in child_ids]
                    trace.emit('debugger_interrupt_after_window', **diagnostic)
                    for pid in diagnostic['inferior_pids']:
                        os.kill(pid, signal.SIGINT)
                    proc.wait(timeout=10)
                    diagnostic['debugger_exit_code'] = proc.returncode
                except (OSError, subprocess.TimeoutExpired) as exc:
                    diagnostic['error'] = str(exc)
                    cleanup_errors.append('debugger stack capture: ' + str(exc))
                (args.output/'stack_capture.json').write_text(json.dumps(diagnostic,indent=2))
        # Direct child executables receive one SIGINT; no launch forwarding double signal.
        for _,proc in reversed(children):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        for name,proc in reversed(children):
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                cleanup_errors.append(name+': SIGINT exit timeout')
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    cleanup_errors.append(name+': SIGTERM exit timeout; killed')
                    proc.kill()
                    proc.wait(timeout=5)
        trace.context.update(windows=windows,counts=dict(counts),capture_scope="host_latency",
                             mechanics='receive-only' if args.receive_only else 'real-image relay; yaw +/-60 deg, pitch +/-30 deg from initial pose',
                             actual_sends='see serial.csv; no diagnostic ACK requested',error=error)
        trace.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for log in logs:
            log.close()
        config_files={
            'hik_camera/config/camera_params.yaml': ROOT/'hik_camera/config/camera_params.yaml',
            'auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml': ROOT/'auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml',
            'auto_aim_bringup/params/tracker_params.yaml': TRACKER_PARAMS,
            'rm_aimer/config/basic_aimer.yaml': AIMER_PARAMS,
            'rm_aimer/config/controller_preview.yaml': PREVIEW_PARAMS,
            'rm_serial_driver/config/gimbal_tf.yaml': TF_PARAMS,
            'rm_serial_driver/config/serial_bridge.yaml': ROOT/'rm_serial_driver/config/serial_bridge.yaml',
        }
        snapshots=args.output/'configuration'
        snapshots.mkdir()
        hashes={}
        for name, path in config_files.items():
            content=path.read_bytes()
            (snapshots/(name.replace('/','__'))).write_bytes(content)
            hashes[name]=hashlib.sha256(content).hexdigest()
        manifest=dict(aimer_tf_topology='dedicated', aimer_executor_threads=1, test_overrides=dict(command_rate_hz=200.0, camera_frame_rate_requested=240.0, prediction_time_s=args.prediction_time, relay_yaw_excursion_deg=60, relay_pitch_excursion_deg=30),schema=1,windows=windows,error=error,cleanup_errors=cleanup_errors,counts=dict(counts),
            domain=args.domain,commands=commands,configuration_sha256=hashes,configuration_paths={name:str(path) for name,path in config_files.items()},
            process_exit_codes={name:proc.returncode for name,proc in children},
            capture_scope='full_chain_observation',run_id=args.output.name,bridge_receive_only=args.receive_only,component_trace_enabled=not args.trace_off,diagnostic_boundaries_enabled=args.boundaries and not args.trace_off,debugger_owns_tracker=args.debugger,motion_source='none' if args.receive_only else 'real-image relay; yaw60/pitch30; other defaults unchanged',clock=dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),time_namespace=os.readlink('/proc/self/ns/time'),host_clock='CLOCK_MONOTONIC',host_resolution_s=time.get_clock_info('monotonic').resolution))
        (args.output/'capture.json').write_text(json.dumps(manifest,indent=2)+'\n')
        print(json.dumps(manifest,indent=2),flush=True)
    # A child failing during shutdown must fail validation as well.
    return 1 if error or cleanup_errors or any(proc.returncode != 0 for _,proc in children) else 0


if __name__=='__main__':
    raise SystemExit(main())
