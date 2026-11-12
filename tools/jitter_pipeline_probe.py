#!/usr/bin/env python3
"""Record the read-only aiming pipeline on one monotonic timeline."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from auto_aim_interfaces.msg import (
    AimCommand,
    Armors,
    GimbalFeedback,
    Target,
    TrackerInfo,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def quaternion_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def distribution(values: list[float]) -> dict | None:
    if not values:
        return None
    return {
        "n": len(values),
        "mean": statistics.fmean(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


class JitterPipelineProbe(Node):
    def __init__(self) -> None:
        super().__init__("jitter_pipeline_probe")
        self.started_mono_ns = time.monotonic_ns()
        self.events: dict[str, list[dict]] = {}

        subscriptions = (
            (CameraInfo, "/camera_info", self.camera_callback),
            (Armors, "/detector/armors", self.armors_callback),
            (Target, "/tracker/raw_target", lambda msg: self.target_callback("raw_target", msg)),
            (Target, "/tracker/target", lambda msg: self.target_callback("target", msg)),
            (TrackerInfo, "/tracker/info", self.tracker_info_callback),
            (AimCommand, "/aimer/aim_command", lambda msg: self.command_callback("aimer", msg)),
            (
                AimCommand,
                "/controller_command_preview/command",
                lambda msg: self.command_callback("preview", msg),
            ),
            (
                AimCommand,
                "/serial_bridge/aim_command",
                lambda msg: self.command_callback("bridge_input", msg),
            ),
            (Bool, "/serial_bridge/command_active", self.command_active_callback),
            (String, "/controller_command_preview/status", self.status_callback),
            (GimbalFeedback, "/serial_bridge/feedback", self.feedback_callback),
            (TFMessage, "/tf", self.tf_callback),
        )
        self._subscriptions = [
            self.create_subscription(msg_type, topic, callback, qos_profile_sensor_data)
            for msg_type, topic, callback in subscriptions
        ]

    def base(self, message=None) -> dict:
        row = {
            "mono_ns": time.monotonic_ns(),
            "wall_ns": time.time_ns(),
        }
        if message is not None and hasattr(message, "header"):
            row["stamp_ns"] = stamp_ns(message)
        return row

    def append(self, name: str, row: dict) -> None:
        self.events.setdefault(name, []).append(row)

    def camera_callback(self, message: CameraInfo) -> None:
        self.append("camera", self.base(message))

    def armors_callback(self, message: Armors) -> None:
        row = self.base(message)
        row["armors"] = [
            {
                "number": armor.number,
                "type": armor.type,
                "distance_to_image_center": armor.distance_to_image_center,
                "position": [
                    armor.pose.position.x,
                    armor.pose.position.y,
                    armor.pose.position.z,
                ],
                "quaternion": [
                    armor.pose.orientation.x,
                    armor.pose.orientation.y,
                    armor.pose.orientation.z,
                    armor.pose.orientation.w,
                ],
                "normal_z": 2.0
                * (
                    armor.pose.orientation.x * armor.pose.orientation.z
                    - armor.pose.orientation.w * armor.pose.orientation.y
                ),
                "yaw": quaternion_yaw(armor.pose.orientation),
            }
            for armor in message.armors
        ]
        self.append("armors", row)

    def target_callback(self, name: str, message: Target) -> None:
        row = self.base(message)
        row.update(
            tracking=bool(message.tracking),
            id=message.id,
            armors_num=message.armors_num,
            position=[message.position.x, message.position.y, message.position.z],
            velocity=[message.velocity.x, message.velocity.y, message.velocity.z],
            yaw=message.yaw,
            v_yaw=message.v_yaw,
            radius_1=message.radius_1,
            radius_2=message.radius_2,
            dz=message.dz,
        )
        self.append(name, row)

    def tracker_info_callback(self, message: TrackerInfo) -> None:
        row = self.base()
        row.update(
            position_diff=message.position_diff,
            yaw_diff=message.yaw_diff,
            position=[message.position.x, message.position.y, message.position.z],
            yaw=message.yaw,
        )
        self.append("tracker_info", row)

    def command_callback(self, name: str, message: AimCommand) -> None:
        row = self.base(message)
        row.update(
            valid=bool(message.valid),
            frame_id=message.header.frame_id,
            yaw=message.yaw,
            pitch=message.pitch,
        )
        self.append(name, row)

    def status_callback(self, message: String) -> None:
        row = self.base()
        row["status"] = message.data
        self.append("preview_status", row)

    def command_active_callback(self, message: Bool) -> None:
        row = self.base()
        row["active"] = bool(message.data)
        self.append("command_active", row)

    def feedback_callback(self, message: GimbalFeedback) -> None:
        row = self.base(message)
        row.update(
            sequence=message.sequence,
            mcu_timestamp_ms=message.mcu_timestamp_ms,
            yaw=message.yaw,
            pitch=message.pitch,
            mode=message.mode,
        )
        self.append("feedback", row)

    def tf_callback(self, message: TFMessage) -> None:
        received = self.base()
        for transform in message.transforms:
            row = dict(received)
            row.update(
                stamp_ns=int(transform.header.stamp.sec) * 1_000_000_000
                + int(transform.header.stamp.nanosec),
                parent=transform.header.frame_id,
                child=transform.child_frame_id,
                yaw=quaternion_yaw(transform.transform.rotation),
            )
            self.append("tf", row)

    def print_diagnostics(
        self,
        window_s: float,
        feedback_stale_ms: float,
        minimum_feedback_hz: float,
        gap_warning_ms: float,
        pipeline_latency_warning_ms: float,
        pipeline_max_latency_warning_ms: float,
        angle_settle_ms: float,
        angle_error_warning_deg: float,
    ) -> None:
        now_ns = time.monotonic_ns()
        cutoff_ns = now_ns - int(window_s * 1e9)
        elapsed_s = (now_ns - self.started_mono_ns) / 1e9
        observation_window_s = min(window_s, max(elapsed_s, 1e-6))
        diagnostic_window_ready = elapsed_s >= window_s
        recent = {
            name: [row for row in rows if row["mono_ns"] >= cutoff_ns]
            for name, rows in self.events.items()
        }
        status = {}
        problems = []
        for name in (
            "camera", "armors", "raw_target", "target", "aimer", "preview",
            "bridge_input", "command_active", "feedback",
        ):
            rows = recent.get(name, [])
            gaps_ms = [
                (current["mono_ns"] - previous["mono_ns"]) / 1e6
                for previous, current in zip(rows, rows[1:])
            ]
            age_ms = None if not rows else (now_ns - rows[-1]["mono_ns"]) / 1e6
            status[name] = {
                "samples": len(rows),
                "rate_hz": round(len(rows) / observation_window_s, 2),
                "age_ms": None if age_ms is None else round(age_ms, 1),
                "max_gap_ms": None if not gaps_ms else round(max(gaps_ms), 1),
            }
            if name in ("aimer", "preview", "bridge_input"):
                status[name]["valid_samples"] = sum(
                    bool(row["valid"]) for row in rows
                )
                status[name]["invalid_samples"] = sum(
                    not bool(row["valid"]) for row in rows
                )
            elif name == "command_active":
                status[name]["state"] = None if not rows else rows[-1]["active"]

        if diagnostic_window_ready:
            if not recent.get("camera"):
                problems.append({
                    "code": "camera_stream_missing",
                    "location": "camera driver -> /camera_info",
                    "description": "相机数据流中断：诊断窗口内没有相机帧元数据",
                    "impact": "检测、跟踪和瞄准链路都没有新图像可处理",
                    "likely_causes": "相机取流超时、USB/设备断开、相机节点异常或 DDS 配置不一致",
                    "measured_hz": status["camera"]["rate_hz"],
                    "window_s": window_s,
                })
            elif not recent.get("armors"):
                problems.append({
                    "code": "detector_output_missing",
                    "location": "/camera_info -> /detector/armors",
                    "description": "相机有数据，但检测器没有发布装甲板消息",
                    "impact": "跟踪器无法获得检测结果",
                    "likely_causes": "检测节点停止、订阅/话题不匹配、推理阻塞或回调积压",
                    "camera_hz": status["camera"]["rate_hz"],
                    "detector_hz": status["armors"]["rate_hz"],
                })
            elif not recent.get("raw_target"):
                problems.append({
                    "code": "tracker_output_missing",
                    "location": "/detector/armors -> /tracker/raw_target",
                    "description": "检测器有输出，但跟踪器没有发布原始目标",
                    "impact": "无法生成稳定目标状态和预测角度",
                    "likely_causes": "跟踪节点停止、TF 不可用、消息过滤队列阻塞或话题不匹配",
                    "detector_hz": status["armors"]["rate_hz"],
                    "tracker_hz": status["raw_target"]["rate_hz"],
                })
            elif not recent.get("target"):
                problems.append({
                    "code": "target_filter_output_missing",
                    "location": "/tracker/raw_target -> /tracker/target",
                    "description": "原始跟踪目标存在，但过滤后的目标话题没有输出",
                    "impact": "瞄准器无法接收最终目标状态",
                    "likely_causes": "目标 watchdog/过滤节点停止、话题重映射错误或回调阻塞",
                    "raw_target_hz": status["raw_target"]["rate_hz"],
                    "target_hz": status["target"]["rate_hz"],
                })
            elif not recent.get("aimer"):
                problems.append({
                    "code": "aimer_output_missing",
                    "location": "/tracker/target -> /aimer/aim_command",
                    "description": "跟踪器有输出，但瞄准器没有发布角度指令",
                    "impact": "桥接器没有可用的 yaw/pitch 目标",
                    "likely_causes": "瞄准节点停止、目标话题未连接或解算回调异常",
                    "target_hz": status["target"]["rate_hz"],
                    "aimer_hz": status["aimer"]["rate_hz"],
                })

        feedback = recent.get("feedback", [])
        if not feedback:
            if diagnostic_window_ready:
                problems.append({
                    "code": "feedback_missing",
                    "location": "serial_bridge -> gimbal feedback",
                    "description": "串口反馈中断：诊断窗口内没有收到云台反馈",
                    "impact": "无法得到实际 yaw/pitch，不能判断执行延迟或跟随误差",
                    "likely_causes": "下位机断开、串口桥停止、固件未发送或接收链路阻塞",
                    "window_s": window_s,
                })
        else:
            feedback_age_ms = (now_ns - feedback[-1]["mono_ns"]) / 1e6
            if feedback_age_ms > feedback_stale_ms:
                problems.append({
                    "code": "feedback_stale",
                    "location": "serial_bridge -> gimbal feedback",
                    "description": "串口反馈过期：最新云台反馈超过允许时延",
                    "impact": "RViz 与控制状态会显示旧角度，表现为落后或停顿",
                    "likely_causes": "串口接收暂停、桥接进程调度延迟或消息积压",
                    "measured_ms": round(feedback_age_ms, 1),
                    "threshold_ms": feedback_stale_ms,
                })
            feedback_hz = len(feedback) / observation_window_s
            if diagnostic_window_ready and feedback_hz < minimum_feedback_hz:
                problems.append({
                    "code": "feedback_rate_low",
                    "location": "serial_bridge -> gimbal feedback",
                    "description": "云台反馈频率过低",
                    "impact": "实际角度更新变稀，显示和闭环判断会出现阶梯感",
                    "likely_causes": "下位机发送周期过长、丢包、串口读取或系统调度拥塞",
                    "measured_hz": round(feedback_hz, 2),
                    "threshold_hz": minimum_feedback_hz,
                })
            gaps_ms = [
                (current["mono_ns"] - previous["mono_ns"]) / 1e6
                for previous, current in zip(feedback, feedback[1:])
            ]
            if gaps_ms and max(gaps_ms) > gap_warning_ms:
                problems.append({
                    "code": "feedback_gap",
                    "location": "serial_bridge -> gimbal feedback",
                    "description": "云台反馈出现过长的到达间隔",
                    "impact": "角度曲线会短暂停住后跳变，体感为变向卡顿",
                    "likely_causes": "串口瞬时丢包、桥接等待、CPU 调度抖动或链路阻塞",
                    "measured_ms": round(max(gaps_ms), 1),
                    "threshold_ms": gap_warning_ms,
                })

        pipeline = {}
        arrivals = {
            name: {row["stamp_ns"]: row["mono_ns"] for row in recent.get(name, [])}
            for name in ("armors", "raw_target", "target", "aimer", "preview")
        }
        for source, destination in zip(
            ("armors", "raw_target", "target", "aimer"),
            ("raw_target", "target", "aimer", "preview"),
        ):
            samples = [
                (arrivals[destination][stamp] - arrival_ns) / 1e6
                for stamp, arrival_ns in arrivals[source].items()
                if stamp in arrivals[destination]
            ]
            if samples:
                stage = f"{source}_to_{destination}_ms"
                pipeline[stage] = {
                    "mean": round(statistics.fmean(samples), 2),
                    "p95": round(percentile(samples, 0.95), 2),
                    "max": round(max(samples), 2),
                    "samples": len(samples),
                }
                p95_high = pipeline[stage]["p95"] > pipeline_latency_warning_ms
                max_high = pipeline[stage]["max"] > pipeline_max_latency_warning_ms
                if p95_high or max_high:
                    problems.append({
                        "code": "pipeline_stage_latency_high",
                        "location": f"{source} -> {destination}",
                        "description": "视觉链路阶段存在持续或瞬时高延迟",
                        "impact": "目标角度会落后于当前画面，瞬时尖峰会表现为一次卡顿",
                        "likely_causes": "该阶段计算耗时、ROS 回调调度延迟或旧消息积压",
                        "trigger": "+".join(
                            name
                            for name, active in (("p95", p95_high), ("max", max_high))
                            if active
                        ),
                        "mean_ms": pipeline[stage]["mean"],
                        "p95_ms": pipeline[stage]["p95"],
                        "max_ms": pipeline[stage]["max"],
                        "samples": pipeline[stage]["samples"],
                        "p95_threshold_ms": pipeline_latency_warning_ms,
                        "max_threshold_ms": pipeline_max_latency_warning_ms,
                    })

        angle = None
        valid_commands = {
            name: [row for row in recent.get(name, []) if bool(row["valid"])]
            for name in ("bridge_input", "preview", "aimer")
        }
        settle_cutoff_ns = now_ns - int(angle_settle_ms * 1e6)
        settled_commands = {
            name: [row for row in rows if row["mono_ns"] <= settle_cutoff_ns]
            for name, rows in valid_commands.items()
        }
        source_name = next(
            (
                name
                for name in ("bridge_input", "preview", "aimer")
                if settled_commands[name]
            ),
            None,
        )
        active_rows = recent.get("command_active", [])
        write_active = bool(active_rows and active_rows[-1]["active"])
        write_state_age_ms = (
            None
            if not active_rows
            else round((now_ns - active_rows[-1]["mono_ns"]) / 1e6, 1)
        )
        if (
            diagnostic_window_ready
            and valid_commands["bridge_input"]
            and not write_active
        ):
            problems.append({
                "code": "bridge_command_inactive",
                "location": "serial bridge command input -> Windows COM write",
                "description": "桥接器收到有效指令，但当前没有确认串口写入成功",
                "impact": "算法目标角不会传到下位机，云台可能保持不动",
                "likely_causes": "桥接处于只读模式、COM 写入超时、指令过期或 Windows 辅助进程拒绝",
                "valid_input_samples": len(valid_commands["bridge_input"]),
                "command_active": write_active,
                "state_age_ms": write_state_age_ms,
            })
        if source_name and feedback:
            command = settled_commands[source_name][-1]
            eligible_feedback = [
                row
                for row in feedback
                if row["mono_ns"] - command["mono_ns"] >= int(angle_settle_ms * 1e6)
            ]
            if not eligible_feedback:
                command = None
            else:
                actual = eligible_feedback[0]
        else:
            command = None
        if command is not None:
            yaw_error_deg = math.degrees(angle_delta(command["yaw"], actual["yaw"]))
            pitch_error_deg = math.degrees(command["pitch"] - actual["pitch"])
            sample_dt_ms = (actual["mono_ns"] - command["mono_ns"]) / 1e6
            angle = {
                "source": source_name,
                "comparison_scope": (
                    "bridge_write_state_active"
                    if source_name == "bridge_input" and write_active
                    else (
                        "bridge_input_not_write_confirmed"
                        if source_name == "bridge_input"
                        else "preview_only_not_sent_to_bridge"
                    )
                ),
                "command_active": write_active,
                "command_active_age_ms": write_state_age_ms,
                "settle_ms": angle_settle_ms,
                "target_yaw_deg": round(math.degrees(command["yaw"]), 3),
                "actual_yaw_deg": round(math.degrees(actual["yaw"]), 3),
                "yaw_error_deg": round(yaw_error_deg, 3),
                "target_pitch_deg": round(math.degrees(command["pitch"]), 3),
                "actual_pitch_deg": round(math.degrees(actual["pitch"]), 3),
                "pitch_error_deg": round(pitch_error_deg, 3),
                "feedback_minus_command_arrival_ms": round(sample_dt_ms, 1),
                "note": "Arrival-time pairing; curve lag is calculated after capture",
            }
            if source_name == "bridge_input" and write_active and max(
                abs(yaw_error_deg), abs(pitch_error_deg)
            ) > angle_error_warning_deg:
                problems.append({
                    "code": "confirmed_command_angle_error_large",
                    "location": "bridge input while write state is active -> gimbal feedback",
                    "description": "桥接写入状态为 active 时，目标角与反馈角差值过大",
                    "impact": "云台尚未跟上指令，或控制/通信延迟明显",
                    "likely_causes": "执行器响应慢、命令积压、单位/零点错误或反馈配对时间不准",
                    "yaw_error_deg": round(yaw_error_deg, 3),
                    "pitch_error_deg": round(pitch_error_deg, 3),
                    "threshold_deg": angle_error_warning_deg,
                    "sample_dt_ms": round(sample_dt_ms, 1),
                    "settle_ms": angle_settle_ms,
                })

        print("DIAG " + json.dumps({
            "window_s": window_s,
            "observed_s": round(observation_window_s, 3),
            "window_ready": diagnostic_window_ready,
            "topics": status,
            "pipeline": pipeline,
            "angles": angle,
            "problems": problems,
        }, ensure_ascii=False, separators=(",", ":")), flush=True)
        status_parts = []
        for name, values in status.items():
            age = "missing" if values["age_ms"] is None else f"{values['age_ms']}ms"
            max_gap = (
                "missing"
                if values["max_gap_ms"] is None
                else f"{values['max_gap_ms']}ms"
            )
            command_counts = ""
            if "valid_samples" in values:
                command_counts = (
                    f", valid={values['valid_samples']}, "
                    f"invalid={values['invalid_samples']}"
                )
            elif "state" in values:
                command_counts = f", state={values['state']}"
            status_parts.append(
                f"{name}: rate={values['rate_hz']}Hz, age={age}, "
                f"max_gap={max_gap}, samples={values['samples']}{command_counts}"
            )
        print(
            "STATUS " + " | ".join(status_parts),
            flush=True,
        )
        for stage, values in pipeline.items():
            print(
                f"LATENCY stage={stage}, mean={values['mean']}ms, "
                f"p95={values['p95']}ms, max={values['max']}ms, "
                f"samples={values['samples']}, warning_threshold="
                f"p95>{pipeline_latency_warning_ms}ms or "
                f"max>{pipeline_max_latency_warning_ms}ms",
                flush=True,
            )
        if angle is not None:
            print(
                f"ANGLE source={angle['source']}, "
                f"scope={angle['comparison_scope']}, "
                f"command_active={angle['command_active']}, "
                f"command_active_age={angle['command_active_age_ms']}ms, "
                f"settle={angle['settle_ms']}ms, "
                f"target_yaw={angle['target_yaw_deg']}deg, "
                f"actual_yaw={angle['actual_yaw_deg']}deg, "
                f"yaw_error={angle['yaw_error_deg']}deg, "
                f"target_pitch={angle['target_pitch_deg']}deg, "
                f"actual_pitch={angle['actual_pitch_deg']}deg, "
                f"pitch_error={angle['pitch_error_deg']}deg, "
                f"feedback_minus_command_arrival="
                f"{angle['feedback_minus_command_arrival_ms']}ms",
                flush=True,
            )
        for problem in problems:
            print(
                "PROBLEM "
                + json.dumps(problem, ensure_ascii=False, separators=(",", ":")),
                flush=True,
            )
            parameters = ", ".join(
                f"{key}={value}"
                for key, value in problem.items()
                if key not in {
                    "code", "location", "description", "impact", "likely_causes"
                }
            )
            print(
                f"PROBLEM_DETAIL code={problem['code']} | "
                f"位置={problem['location']} | 问题={problem['description']} | "
                f"参数={parameters or '无'} | 影响={problem['impact']} | "
                f"可能原因={problem['likely_causes']}",
                flush=True,
            )


def summarize(events: dict[str, list[dict]], duration_s: float) -> dict:
    summary: dict[str, object] = {"duration_s": duration_s, "topics": {}}
    for name, rows in events.items():
        gaps_ms = [
            (current["mono_ns"] - previous["mono_ns"]) / 1e6
            for previous, current in zip(rows, rows[1:])
        ]
        stamped_rows = rows
        if name == "tf":
            stamped_rows = [row for row in rows if row["child"] == "pitch_joint"]
        stamp_steps_ms = [
            (current["stamp_ns"] - previous["stamp_ns"]) / 1e6
            for previous, current in zip(stamped_rows, stamped_rows[1:])
            if "stamp_ns" in previous and "stamp_ns" in current
        ]
        topic = {
            "n": len(rows),
            "rate_hz": len(rows) / duration_s,
            "arrival_gap_ms": distribution(gaps_ms),
        }
        if stamp_steps_ms:
            topic["stamp_step_ms"] = distribution(stamp_steps_ms)
            topic["nonpositive_stamp_steps"] = sum(step <= 0.0 for step in stamp_steps_ms)
            topic["minimum_stamp_step_ms"] = min(stamp_steps_ms)
        summary["topics"][name] = topic

    armors = events.get("armors", [])
    populated = [row for row in armors if row["armors"]]
    armor_yaw_steps = [
        abs(angle_delta(current["armors"][0]["yaw"], previous["armors"][0]["yaw"]))
        for previous, current in zip(armors, armors[1:])
        if previous["armors"] and current["armors"]
    ]
    summary["detector"] = {
        "populated_frames": len(populated),
        "empty_frames": len(armors) - len(populated),
        "first_armor_yaw_step_deg": distribution(
            [math.degrees(value) for value in armor_yaw_steps]
        ),
        "yaw_steps_over_1_rad": sum(value > 1.0 for value in armor_yaw_steps),
    }

    for name in ("raw_target", "target"):
        rows = events.get(name, [])
        tracking = [row for row in rows if row["tracking"]]
        yaw_steps = [
            abs(angle_delta(current["yaw"], previous["yaw"]))
            for previous, current in zip(rows, rows[1:])
            if previous["tracking"] and current["tracking"]
        ]
        summary[name] = {
            "tracking_true": len(tracking),
            "tracking_false": len(rows) - len(tracking),
            "tracking_transitions": sum(
                current["tracking"] != previous["tracking"]
                for previous, current in zip(rows, rows[1:])
            ),
            "tracking_yaw_step_deg": distribution(
                [math.degrees(value) for value in yaw_steps]
            ),
        }

    for name in ("aimer", "preview", "bridge_input"):
        rows = events.get(name, [])
        valid = [row for row in rows if row["valid"]]
        summary[name] = {
            "valid": len(valid),
            "invalid": len(rows) - len(valid),
            "valid_yaw_step_deg": distribution(
                [
                    math.degrees(abs(angle_delta(current["yaw"], previous["yaw"])))
                    for previous, current in zip(rows, rows[1:])
                    if previous["valid"] and current["valid"]
                ]
            ),
            "valid_pitch_step_deg": distribution(
                [
                    math.degrees(abs(current["pitch"] - previous["pitch"]))
                    for previous, current in zip(rows, rows[1:])
                    if previous["valid"] and current["valid"]
                ]
            ),
        }

    tf_rows = [row for row in events.get("tf", []) if row["child"] == "pitch_joint"]
    tf_skews_ms = []
    tf_index = 0
    latest_tf_stamp = None
    for armor in armors:
        while tf_index < len(tf_rows) and tf_rows[tf_index]["mono_ns"] <= armor["mono_ns"]:
            stamp = tf_rows[tf_index]["stamp_ns"]
            latest_tf_stamp = stamp if latest_tf_stamp is None else max(latest_tf_stamp, stamp)
            tf_index += 1
        if latest_tf_stamp is not None:
            tf_skews_ms.append((armor["stamp_ns"] - latest_tf_stamp) / 1e6)
    summary["tf"] = {
        "armor_minus_latest_pitch_tf_ms": distribution(tf_skews_ms),
        "minimum_armor_minus_latest_pitch_tf_ms": min(tf_skews_ms, default=None),
        "armor_frames_older_than_latest_tf": sum(value < 0.0 for value in tf_skews_ms),
        "armor_frames_newer_than_latest_tf": sum(value > 0.0 for value in tf_skews_ms),
        "armor_frames_over_20ms_newer": sum(value > 20.0 for value in tf_skews_ms),
        "armor_frames_over_50ms_newer": sum(value > 50.0 for value in tf_skews_ms),
    }

    stamp_arrivals = {
        name: {row["stamp_ns"]: row["mono_ns"] for row in events.get(name, [])}
        for name in ("armors", "raw_target", "target", "aimer", "preview")
    }
    pipeline = {}
    for source, destination in zip(
        ("armors", "raw_target", "target", "aimer"),
        ("raw_target", "target", "aimer", "preview"),
    ):
        latencies = [
            (stamp_arrivals[destination][stamp] - arrival) / 1e6
            for stamp, arrival in stamp_arrivals[source].items()
            if stamp in stamp_arrivals[destination]
        ]
        pipeline[f"{source}_to_{destination}_ms"] = distribution(latencies)
    summary["pipeline"] = pipeline
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--report-interval", type=float, default=2.0)
    parser.add_argument("--diagnostic-window", type=float, default=5.0)
    parser.add_argument("--feedback-stale-ms", type=float, default=200.0)
    parser.add_argument("--minimum-feedback-hz", type=float, default=15.0)
    parser.add_argument("--gap-warning-ms", type=float, default=150.0)
    parser.add_argument("--pipeline-latency-warning-ms", type=float, default=50.0)
    parser.add_argument("--pipeline-max-latency-warning-ms", type=float, default=100.0)
    parser.add_argument("--angle-settle-ms", type=float, default=100.0)
    parser.add_argument("--angle-error-warning-deg", type=float, default=2.0)
    args = parser.parse_args()
    if min(
        args.duration,
        args.report_interval,
        args.diagnostic_window,
        args.feedback_stale_ms,
        args.minimum_feedback_hz,
        args.gap_warning_ms,
        args.pipeline_latency_warning_ms,
        args.pipeline_max_latency_warning_ms,
        args.angle_settle_ms,
        args.angle_error_warning_deg,
    ) <= 0.0:
        parser.error("duration and diagnostic parameters must be positive")

    rclpy.init()
    node = JitterPipelineProbe()
    deadline = time.monotonic() + args.duration
    next_report = time.monotonic() + args.report_interval
    print(
        "DIAGNOSTIC_CONFIG "
        f"duration={args.duration}s, report_interval={args.report_interval}s, "
        f"window={args.diagnostic_window}s, "
        f"feedback_stale_threshold={args.feedback_stale_ms}ms, "
        f"minimum_feedback_rate={args.minimum_feedback_hz}Hz, "
        f"feedback_gap_threshold={args.gap_warning_ms}ms, "
        f"pipeline_p95_threshold={args.pipeline_latency_warning_ms}ms, "
        f"pipeline_max_threshold={args.pipeline_max_latency_warning_ms}ms, "
        f"angle_settle={args.angle_settle_ms}ms, "
        f"confirmed_angle_error_threshold={args.angle_error_warning_deg}deg",
        flush=True,
    )
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_report:
                node.print_diagnostics(
                    args.diagnostic_window,
                    args.feedback_stale_ms,
                    args.minimum_feedback_hz,
                    args.gap_warning_ms,
                    args.pipeline_latency_warning_ms,
                    args.pipeline_max_latency_warning_ms,
                    args.angle_settle_ms,
                    args.angle_error_warning_deg,
                )
                next_report = time.monotonic() + args.report_interval
    finally:
        elapsed = (time.monotonic_ns() - node.started_mono_ns) / 1e9
        result = {
            "summary": summarize(node.events, elapsed),
            "events": node.events,
        }
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result["summary"], indent=2))


if __name__ == "__main__":
    main()
