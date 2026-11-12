#!/usr/bin/env python3
"""Preview controller angles on a separate topic without serial output."""

from collections import deque
import json
import math
import time

import rclpy
from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from controller_angles import FeedbackSample, controller_targets, match_feedback


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ControllerCommandPreview(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('controller_command_preview', parameter_overrides=parameter_overrides)
        stability_path = str(self.declare_parameter('stability_trace_path', '').value)
        self.stability_trace = None
        self.stability_source = None
        self.stability_feedback = {}
        if stability_path:
            from aim_stability_trace import BufferedTrace
            self.stability_trace = BufferedTrace(stability_path)
        self.expected_frame = str(self.declare_parameter('input_frame', 'pitch_joint').value)
        input_topic = str(self.declare_parameter('input_topic', '/aimer/aim_command').value)
        feedback_topic = str(self.declare_parameter('feedback_topic', '/serial_bridge/feedback').value)
        self.offset = float(self.declare_parameter('pitch_raw_offset_rad', 0.319451511).value)
        self.scale = float(self.declare_parameter('pitch_raw_scale', 1.023925824751).value)
        self.yaw_sign = float(self.declare_parameter('yaw_raw_sign', 1.0).value)
        self.max_age = float(self.declare_parameter('max_age', 0.2).value)
        self.max_skew = float(self.declare_parameter('max_pair_skew', 0.05).value)
        if not self.expected_frame or not all(math.isfinite(v) for v in
                (self.offset, self.scale, self.yaw_sign, self.max_age, self.max_skew)):
            raise ValueError('Preview parameters must be finite and input_frame nonempty')
        if self.scale == 0 or self.yaw_sign not in (-1.0, 1.0) or min(self.max_age, self.max_skew) <= 0:
            raise ValueError('Invalid preview calibration or timeout')
        self.conversion_pub = (
            self.create_publisher(String, '~/conversion', qos_profile_sensor_data)
            if self.declare_parameter('publish_diagnostics', False).value else None)
        self.conversion_sequence = 0
        self.feedback_headers = {}
        self.samples = deque(maxlen=100)
        self.command_at = float('-inf')
        self.feedback_at = float('-inf')
        self.command_stamp = 0.0
        self.status = None
        self.command_pub = self.create_publisher(AimCommand, '~/command', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '~/status', 1)
        self.create_subscription(AimCommand, input_topic, self.command_callback, qos_profile_sensor_data)
        self.create_subscription(GimbalFeedback, feedback_topic, self.feedback_callback,
                                 qos_profile_sensor_data)
        self.create_timer(0.02, self.check_timeout)
        self.create_timer(1.0, self.republish_status)
        self.publish('waiting_for_data')

    def republish_status(self):
        self.status_pub.publish(String(data=self.status))

    def publish(self, status, angles=None, stamp=None):
        message = AimCommand()
        message.header.stamp = stamp or self.get_clock().now().to_msg()
        message.header.frame_id = 'controller_joint_targets_preview'
        message.valid = angles is not None and status == 'candidate_unverified'
        if angles is not None:
            message.yaw, message.pitch = angles
        trace = getattr(self, 'stability_trace', None)
        if trace:
            trace.emit('preview_output', source_stamp_ns=self.stability_source,
                       frame_id=message.header.frame_id, valid=message.valid,
                       status=status, yaw_rad=message.yaw if message.valid else None,
                       pitch_rad=message.pitch if message.valid else None)
        self.command_pub.publish(message)
        if status != self.status:
            self.status = status
            self.status_pub.publish(String(data=status))

    def feedback_callback(self, message):
        sample = FeedbackSample(stamp_seconds(message.header.stamp), message.yaw, message.pitch)
        trace = getattr(self, 'stability_trace', None)
        if trace:
            row = trace.emit('preview_feedback', feedback_stamp_ns=
                message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec,
                feedback_sequence=message.sequence, mcu_timestamp_ms=message.mcu_timestamp_ms,
                yaw_rad=message.yaw, pitch_rad=message.pitch, mode=message.mode)
            self.stability_feedback[sample.stamp] = row
            if len(self.stability_feedback)>200:
                del self.stability_feedback[next(iter(self.stability_feedback))]
        if not all(math.isfinite(v) for v in (sample.stamp, sample.yaw, sample.pitch)):
            self.samples.clear()
            self.feedback_at = float('-inf')
            self.publish('invalid_feedback')
            return
        if self.samples and sample.stamp <= self.samples[-1].stamp:
            self.samples.clear()
        if self.conversion_pub is not None:
            self.feedback_headers[sample.stamp] = dict(
                feedback_stamp_ns=message.header.stamp.sec * 1_000_000_000
                                  + message.header.stamp.nanosec,
                feedback_sequence=message.sequence)
            if len(self.feedback_headers) > 200:
                del self.feedback_headers[next(iter(self.feedback_headers))]
        self.samples.append(sample)
        self.feedback_at = time.monotonic()

    def command_callback(self, message):
        trace = getattr(self, 'stability_trace', None)
        if trace:
            self.stability_source = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            trace.emit("preview_input", source_stamp_ns=self.stability_source)
        self.command_at = time.monotonic()
        self.command_stamp = stamp_seconds(message.header.stamp)
        if not message.valid:
            self.publish('invalid_input_command')
            return
        if message.header.frame_id != self.expected_frame:
            self.publish('input_frame_mismatch')
            return
        try:
            sample = match_feedback(self.samples, self.command_stamp,
                                    self.get_clock().now().nanoseconds * 1e-9,
                                    self.max_age, self.max_skew)
            angles = controller_targets(message.yaw, message.pitch, sample.yaw, sample.pitch,
                                        self.offset, self.scale, self.yaw_sign)
            if trace:
                feedback_row = self.stability_feedback.get(sample.stamp) or {}
                trace.emit('preview_conversion', source_stamp_ns=self.stability_source,
                    frame_id='controller_joint_targets_preview',
                    input_yaw_rad=message.yaw, input_pitch_rad=message.pitch,
                    yaw_rad=angles[0], pitch_rad=angles[1],
                    feedback_yaw_rad=sample.yaw, feedback_pitch_rad=sample.pitch,
                    feedback_stamp_ns=feedback_row.get('feedback_stamp_ns'),
                    feedback_sequence=feedback_row.get('feedback_sequence'),
                    feedback_received_ns=feedback_row.get('host_ns'))
        except ValueError as exc:
            self.publish(str(exc))
            return
        self.publish('candidate_unverified', angles, message.header.stamp)
        if self.conversion_pub is not None:
            self.conversion_sequence += 1
            payload = dict(
                schema=1, event_sequence=self.conversion_sequence, valid=True,
                source_stamp_ns=message.header.stamp.sec * 1_000_000_000
                                + message.header.stamp.nanosec,
                host_monotonic_ns=time.monotonic_ns(),
                input_frame=self.expected_frame,
                frame_id='controller_joint_targets_preview',
                input_yaw_rad=message.yaw, input_pitch_rad=message.pitch,
                yaw_rad=angles[0], pitch_rad=angles[1],
                feedback_yaw_rad=sample.yaw, feedback_pitch_rad=sample.pitch,
                pitch_raw_offset_rad=self.offset, pitch_raw_scale=self.scale,
                yaw_raw_sign=self.yaw_sign, **self.feedback_headers.get(sample.stamp, {}))
            self.conversion_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def check_timeout(self):
        if self.status != 'candidate_unverified':
            return
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if (now - min(self.command_at, self.feedback_at) > self.max_age or
                not 0 <= ros_now - self.command_stamp <= self.max_age):
            self.publish('data_timeout')

    def destroy_node(self):
        if getattr(self, 'stability_trace', None):
            self.stability_trace.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = ControllerCommandPreview()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
