"""Run the actual permission publisher and bridge on a PTY; no hardware."""
import json
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
import time

os.environ.update(ROS_DOMAIN_ID="215", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
import rclpy
from auto_aim_interfaces.msg import AimCommand
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from protocol import crc16

ROOT=Path(__file__).resolve().parents[2]


def test_permission_to_real_serial_bytes_and_revocation(tmp_path):
    master,slave=pty.openpty()
    os.set_blocking(master,False)
    processes=[];logs=[]
    def start(script, args):
        log=(tmp_path/(script.stem+".log")).open("w");logs.append(log)
        p=subprocess.Popen([sys.executable,str(script),"--ros-args",*args],
                           stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
        processes.append(p);return p
    bridge=start(ROOT/"rm_serial_driver/scripts/serial_bridge_node.py",[
        "-p",f"port:={os.ttyname(slave)}","-p","require_fire_permission:=true",
        "-p","fire_permission_timeout:=0.1","-p","command_rate:=100.0",
        "-p","publish_diagnostics:=true"])
    producer=start(ROOT/"rm_fire_control/scripts/fire_permission_node.py",[
        "-r","__node:=fire_permission_control","-p","enabled:=true",
        "-p","calibration_verified:=true","-p","preview_only:=false",
        "-p","command_topic:=/fire_test/command","-p","decision_topic:=/fire_control/decision"])
    rclpy.init();node=rclpy.create_node("fire_gate_pty_probe")
    publisher=node.create_publisher(AimCommand,"/fire_test/command",qos_profile_sensor_data)
    bypass=node.create_publisher(AimCommand,"/serial_bridge/aim_command",qos_profile_sensor_data)
    decisions=[]
    tx_events=[]
    node.create_subscription(String, "/serial_bridge/tx_event",
                             lambda m: tx_events.append(json.loads(m.data)), qos_profile_sensor_data)
    node.create_subscription(String,"/fire_control/decision",lambda m:decisions.append(json.loads(m.data)),100)
    seq=0
    def pump(duration,valid=True,mode=6,send=True):
        nonlocal seq
        data=bytearray();end=time.monotonic()+duration
        while time.monotonic()<end:
            seq+=1
            body=b"\xb5"+struct.pack("<II6fB",seq,seq*10,1.,0.,0.,0.,.25,-.125,mode)
            os.write(master,body+struct.pack("<H",crc16(body))+b"\x5b\x5a")
            message=AimCommand(valid=valid,yaw=.25,pitch=-.125)
            message.header.stamp=node.get_clock().now().to_msg()
            message.header.frame_id="controller_joint_targets_preview"
            if send:publisher.publish(message)
            bypass.publish(AimCommand(valid=True,yaw=2.,pitch=1.))
            until=time.monotonic()+.01
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.001)
            try:data.extend(os.read(master,65536))
            except BlockingIOError:pass
        return bytes(data)
    try:
        pump(1.2,send=False)
        assert bridge.poll() is None and producer.poll() is None
        assert bypass.get_subscription_count()==0
        assert pump(.15,send=False)==b""
        wire=pump(.55)
        expected=struct.pack("<BffBB",0xa5,.25,-.125,0x5a,0x5b)
        assert expected in wire
        assert decisions[-1]["allowed"] and decisions[-1]["preview_only"] is False
        assert len(wire)%11==0 and wire==expected*(len(wire)//11)
        assert tx_events
        assert all(event["event"] == "host_write_returned"
                   and event["wire_hex"] == expected.hex()
                   and event["bytes_written"] == 11
                   and event["command_stamp_ns"] > 0
                   and event["permission_stamp_ns"] > 0 for event in tx_events)
        assert decisions[-1]["feedback_mode"] == 6
        assert decisions[-1]["feedback_stamp_ns"] > 0
        pump(.1,valid=False)
        assert not decisions[-1]["allowed"]
        count_after_revoke = len(tx_events)
        assert pump(.2,valid=False)==b""
        assert len(tx_events) == count_after_revoke
        assert expected in pump(.4)
        pump(.1,mode=3)
        assert pump(.15,mode=3)==b""
        assert expected in pump(.4)
        # Producer failure must stop transmission despite continued fresh feedback and bypass commands.
        producer.send_signal(signal.SIGINT);producer.wait(timeout=5)
        pump(.25)
        assert pump(.2)==b""
        assert bridge.poll() is None
    finally:
        node.destroy_node();rclpy.shutdown()
        for p in processes:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();p.wait()
        for log in logs:log.close()
        os.close(master);os.close(slave)
