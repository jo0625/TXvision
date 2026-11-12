#!/usr/bin/env python3
"""Publish a reviewed pitch step sequence through the existing serial bridge."""
from __future__ import annotations
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "rm_serial_driver/scripts"))
from protocol import encode_aim_command
from record_wall_clock import local_time


def sequence(step_deg, hold_s, repeats):
    phases=[dict(step_id="baseline", pitch_delta_deg=0.0, hold_s=hold_s)]
    for i in range(1, repeats+1):
        for name,delta in (("up",-step_deg),("return_up",0.0),("down",step_deg),("return_down",0.0)):
            phases.append(dict(step_id=f"pitch_{name}_{i:02d}", pitch_delta_deg=delta, hold_s=hold_s))
    return phases


def execute(args, phases):
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    os.environ.pop("CYCLONEDDS_URI", None)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
    from std_msgs.msg import String
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/"plan.json").write_text(json.dumps(dict(phases=phases, domain=args.domain, command_hz=200,
        reference="current feedback angle at start; yaw held constant", data_kind="hardware_run"),ensure_ascii=False,indent=2))
    rclpy.init()
    node=rclpy.create_node("pitch_step_response_test")
    publisher=node.create_publisher(AimCommand,"/serial_bridge/aim_command",qos_profile_sensor_data)
    state=dict(feedback=None,feedback_at=0.0,phase=None,stop=False)
    events=[]
    result=dict(status="preparing", phases=phases, started_local=local_time(time.time_ns()))
    rows_file=(args.output/"steps.csv").open("x",newline="")
    fields=["step_id","pitch_delta_deg","yaw_deg","pitch_deg","requested_local","first_tx_sequence",
            "write_started_local","write_completed_local","write_completed_unix_ns","wire_hex"]
    writer=csv.DictWriter(rows_file,fieldnames=fields);writer.writeheader();rows_file.flush()
    def feedback(msg):
        state["feedback"],state["feedback_at"]=msg,time.monotonic()
    def transmitted(msg):
        event=json.loads(msg.data)
        events.append(event)
        phase=state["phase"]
        if phase is None or "first_tx_sequence" in phase:
            return
        if event["wire_hex"] != phase["wire_hex"] or event["write_started_unix_ns"] < phase["requested_unix_ns"]:
            return
        phase.update(first_tx_sequence=event["event_sequence"],write_started_local=local_time(event["write_started_unix_ns"]),
                     write_completed_local=local_time(event["write_completed_unix_ns"]),write_completed_unix_ns=event["write_completed_unix_ns"])
        writer.writerow({key:phase[key] for key in fields});rows_file.flush()
        print(json.dumps(dict(event="STEP_WRITTEN",**{key:phase[key] for key in fields}),ensure_ascii=False),flush=True)
    node.create_subscription(GimbalFeedback,"/serial_bridge/feedback",feedback,qos_profile_sensor_data)
    node.create_subscription(String,"/serial_bridge/tx_event",transmitted,qos_profile_sensor_data)
    def stop(*_):state["stop"]=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def spin_until(deadline):
        while time.monotonic()<deadline and not state["stop"]:
            rclpy.spin_once(node,timeout_sec=min(.005,max(0,deadline-time.monotonic())))
    try:
        deadline=time.monotonic()+5
        while not state["stop"]:
            spin_until(time.monotonic()+.02)
            if state["feedback"] is not None and publisher.get_subscription_count()>0 and node.count_publishers("/serial_bridge/tx_event")>0:
                break
            if time.monotonic()>deadline:raise RuntimeError(f"Missing startup endpoints: feedback={state['feedback'] is not None}, command_subscribers={publisher.get_subscription_count()}, tx_publishers={node.count_publishers('/serial_bridge/tx_event')}")
        if state["stop"]:
            result["status"]="cancelled_before_commands"
            return
        f=state["feedback"]
        if f.mode!=6:raise RuntimeError(f"Expected vision-control mode 6, received {f.mode}")
        base_yaw,base_pitch=float(f.yaw),float(f.pitch)
        result.update(baseline_yaw_deg=math.degrees(base_yaw),baseline_pitch_deg=math.degrees(base_pitch),status="running")
        print(json.dumps(dict(event="BASELINE",yaw_deg=result["baseline_yaw_deg"],pitch_deg=result["baseline_pitch_deg"],time=local_time(time.time_ns())),ensure_ascii=False),flush=True)
        for phase in phases:
            if state["stop"]:break
            target_pitch=base_pitch+math.radians(phase["pitch_delta_deg"])
            phase.update(yaw_deg=math.degrees(base_yaw),pitch_deg=math.degrees(target_pitch),
                         requested_unix_ns=time.time_ns(),wire_hex=encode_aim_command(base_yaw,target_pitch).hex())
            phase["requested_local"]=local_time(phase["requested_unix_ns"])
            state["phase"]=phase
            end=time.monotonic()+phase["hold_s"]
            next_send=time.monotonic()
            while time.monotonic()<end and not state["stop"]:
                if time.monotonic()-state["feedback_at"]>.2 or state["feedback"].mode!=6:
                    raise RuntimeError("Feedback stale beyond existing 0.2s timeout or vision mode exited")
                if time.monotonic()>=next_send:
                    msg=AimCommand(valid=True,yaw=base_yaw,pitch=target_pitch)
                    msg.header.stamp=node.get_clock().now().to_msg()
                    msg.header.frame_id="controller_joint_targets_preview"
                    publisher.publish(msg)
                    next_send=time.monotonic()+.005
                spin_until(min(next_send,end))
            if not state["stop"] and "first_tx_sequence" not in phase:
                raise RuntimeError("No matching native serial write recorded for "+phase["step_id"])
        result["status"]="interrupted" if state["stop"] else "complete"
    except BaseException as exc:
        result.update(status="failed",error=repr(exc))
        raise
    finally:
        publisher.publish(AimCommand(valid=False))
        rclpy.spin_once(node,timeout_sec=.05)
        result["finished_local"]=local_time(time.time_ns())
        (args.output/"result.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
        (args.output/"tx_events.json").write_text(json.dumps(events,ensure_ascii=False)+"\n")
        rows_file.close();node.destroy_node();rclpy.shutdown()
        print(json.dumps(dict(event="STEP_TEST_FINISHED",status=result["status"],time=result["finished_local"]),ensure_ascii=False),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step-deg",type=float,default=3)
    parser.add_argument("--hold",type=float,default=2)
    parser.add_argument("--repeats",type=int,default=3)
    parser.add_argument("--domain",type=int,default=198)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--execute",action="store_true")
    args=parser.parse_args()
    if not math.isfinite(args.step_deg) or args.step_deg<=0 or not math.isfinite(args.hold) or args.hold<=0 or args.repeats<=0:
        parser.error("step, hold and repeats must be positive")
    phases=sequence(args.step_deg,args.hold,args.repeats)
    if not args.execute:
        print(json.dumps(dict(mode="plan_only",phases=phases,duration_s=sum(p["hold_s"] for p in phases)),ensure_ascii=False,indent=2))
        return
    if args.output is None:parser.error("--output is required with --execute")
    execute(args,phases)


if __name__=="__main__":main()
