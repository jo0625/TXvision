#!/usr/bin/env python3
"""Bridge the robot USB CDC feedback and aim command protocol to ROS 2."""

from __future__ import annotations

import json
import time

import rclpy
from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import serial
from std_msgs.msg import Bool, String

from protocol import AIM_COMMAND, CommandGate, FeedbackParser
from fire_transmit_gate import FireTransmitGate
from windows_serial import WindowsSerialReader
from control_latency_trace import ControlLatencyTrace
from control_latency_protocol import encode_command as encode_diagnostic_command


class SerialBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("serial_bridge")
        self.port_name = str(self.declare_parameter("port", "/dev/ttyACM0").value)
        self.transport = str(self.declare_parameter("transport", "serial").value)
        self.receive_only = bool(self.declare_parameter("receive_only", False).value)
        if self.transport not in ("serial", "windows", "windows_duplex"):
            raise ValueError("transport must be serial, windows, or windows_duplex")
        if self.transport == "windows":
            self.receive_only = True
        self.baud = int(self.declare_parameter("baud", 115200).value)
        self.frame_id = str(self.declare_parameter("frame_id", "gimbal_imu").value)
        self.read_period = float(self.declare_parameter("read_period", 0.002).value)
        self.reconnect_period = float(self.declare_parameter("reconnect_period", 1.0).value)
        self.feedback_timeout = float(self.declare_parameter("feedback_timeout", 0.2).value)
        self.command_timeout = float(self.declare_parameter("command_timeout", 0.1).value)
        self.command_rate = float(self.declare_parameter("command_rate", 50.0).value)
        if self.transport == "windows_duplex" and self.command_timeout > 1.0:
            raise ValueError("windows_duplex command_timeout must be <= 1 second")
        self.require_fire_permission = bool(self.declare_parameter("require_fire_permission", False).value)
        self.fire_gate = None
        if self.require_fire_permission:
            if self.transport != "serial":
                raise ValueError("Fire permission gating supports native serial only")
            self.fire_gate = FireTransmitGate(
                float(self.declare_parameter("fire_permission_timeout", 0.1).value),
                float(self.declare_parameter("fire_command_max_age", 0.2).value),
                str(self.declare_parameter("fire_command_frame", "controller_joint_targets_preview").value))
        self.fire_permission_topic = str(self.declare_parameter(
            "fire_permission_topic", "/fire_control/decision").value)
        self.fire_required_mode = int(self.declare_parameter("fire_required_mode", 6).value)
        self.last_feedback_mode = None
        trace_path = str(self.declare_parameter("latency_trace_path", "").value)
        trace_protocol = str(self.declare_parameter("latency_protocol", "legacy").value)
        if trace_protocol != "legacy" and not trace_path:
            raise ValueError("diagnostic protocol requires an enabled latency trace")
        if self.require_fire_permission and trace_path:
            raise ValueError("Latency tracing is not supported in fire-gated mode")
        if self.require_fire_permission and trace_protocol != "legacy":
            raise ValueError("Fire permission gating requires the legacy 11-byte wire protocol")
        self.latency_trace = ControlLatencyTrace(
            trace_path,
            capacity=int(self.declare_parameter("latency_trace_capacity", 200000).value),
            ack_timeout_s=float(self.declare_parameter("latency_ack_timeout", 1.0).value),
            protocol=trace_protocol,
            context=dict(transport=self.transport, port=self.port_name, baud=self.baud,
                         command_rate_requested_hz=self.command_rate,
                         command_timeout_s=self.command_timeout,
                         read_period_s=self.read_period, receive_only=self.receive_only),
        ) if trace_path else None
        if self.baud <= 0 or min(
            self.read_period,
            self.reconnect_period,
            self.feedback_timeout,
            self.command_timeout,
            self.command_rate,
        ) <= 0.0:
            raise ValueError("baud, periods, timeouts, and command_rate must be positive")

        self.feedback_pub = self.create_publisher(
            GimbalFeedback, "~/feedback", qos_profile_sensor_data
        )
        self.imu_pub = self.create_publisher(Imu, "~/imu", qos_profile_sensor_data)
        self.connected_pub = self.create_publisher(Bool, "~/connected", 1)
        self.feedback_alive_pub = self.create_publisher(Bool, "~/feedback_alive", 1)
        self.command_active_pub = self.create_publisher(Bool, "~/command_active", 1)
        self.tx_event_pub = (
            self.create_publisher(String, "~/tx_event", qos_profile_sensor_data)
            if self.declare_parameter("publish_diagnostics", False).value else None)
        self.tx_event_sequence = 0
        if not self.receive_only:
            if self.require_fire_permission:
                self.create_subscription(String, self.fire_permission_topic,
                                         self.fire_permission_callback, 1)
            else:
                self.create_subscription(
                    AimCommand, "~/aim_command", self.command_callback, qos_profile_sensor_data
                )

        self.port = None
        self.parser = FeedbackParser()
        self.command_gate = self.fire_gate.command_gate if self.fire_gate else CommandGate()
        self.last_open_attempt = float("-inf")
        self.last_feedback_at = float("-inf")
        self.connected_state = None
        self.feedback_alive_state = None
        self.command_active_state = None
        self.last_pose_warning = float("-inf")

        # Wall-clock corrections in WSL must not pause serial I/O or health checks.
        self.io_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.read_period, self.read_serial, clock=self.io_clock)
        if not self.receive_only:
            self.create_timer(1.0 / self.command_rate, self.write_command, clock=self.io_clock)
        self.create_timer(
            min(0.05, self.feedback_timeout / 2.0), self.update_health, clock=self.io_clock)
        self.create_timer(1.0, self.republish_states, clock=self.io_clock)
        self.publish_state("connected", False)
        self.publish_state("feedback_alive", False)
        self.publish_state("command_active", False)
        mode = "Receive-only" if self.receive_only else "Receive-only until a valid command arrives"
        self.get_logger().info(f"{mode}: {self.transport} {self.port_name} at {self.baud}")

    def publish_state(self, name: str, value: bool) -> None:
        attribute = f"{name}_state"
        if getattr(self, attribute) == value:
            return
        setattr(self, attribute, value)
        publisher = getattr(self, f"{name}_pub")
        publisher.publish(Bool(data=value))

    def republish_states(self) -> None:
        for name in ("connected", "feedback_alive", "command_active"):
            value = getattr(self, f"{name}_state")
            if value is not None:
                getattr(self, f"{name}_pub").publish(Bool(data=value))

    def open_serial(self, now: float) -> None:
        if self.port is not None or now - self.last_open_attempt < self.reconnect_period:
            return
        self.last_open_attempt = now
        try:
            if self.transport == "windows":
                self.port = WindowsSerialReader(self.port_name, self.baud)
            elif self.transport == "windows_duplex":
                if self.receive_only:
                    self.port = WindowsSerialReader(self.port_name, self.baud)
                else:
                    from windows_duplex import WindowsSerialDuplex

                    self.port = WindowsSerialDuplex(self.port_name, self.baud)
            else:
                self.port = serial.Serial(
                    self.port_name,
                    baudrate=self.baud,
                    timeout=0,
                    write_timeout=0.05,
                    exclusive=True,
                )
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f"Cannot open {self.port_name}: {exc}")
            self.publish_state("connected", False)
            return
        self.parser = FeedbackParser()
        if self.fire_gate:
            self.fire_gate.invalidate("connection_reset")
        self.last_feedback_mode = None
        self.command_gate.invalidate()
        self.last_feedback_at = float("-inf")
        self.publish_state("connected", True)
        self.get_logger().info(f"Opened {self.port_name}")

    def close_serial(self, reason: str) -> None:
        trace = getattr(self, "latency_trace", None)
        if trace:
            trace.emit("serial_error", reason=reason)
            trace.disconnected()
        if self.port is not None:
            try:
                self.port.close()
            except (serial.SerialException, OSError):
                pass
        self.port = None
        self.parser = FeedbackParser()
        if self.fire_gate:
            self.fire_gate.invalidate("connection_reset")
        self.last_feedback_mode = None
        self.command_gate.invalidate()
        self.last_feedback_at = float("-inf")
        self.publish_state("connected", False)
        self.publish_state("feedback_alive", False)
        self.publish_state("command_active", False)
        self.get_logger().error(f"Serial connection closed: {reason}")

    def service_duplex_writes(self) -> None:
        if self.transport != "windows_duplex" or self.receive_only or self.port is None:
            return
        self.port.service_writes()
        for result in self.port.take_write_results():
            trace = getattr(self, "latency_trace", None)
            if trace:
                trace.helper_result(result)
            if result.error is None:
                self.publish_state("command_active", True)
            else:
                self.publish_state("command_active", False)
                self.get_logger().warning(f"Dropped aim command: {result.error}")

    def read_serial(self) -> None:
        now = time.monotonic()
        self.open_serial(now)
        if self.port is None:
            return
        try:
            self.service_duplex_writes()
            waiting = self.port.in_waiting
            if waiting <= 0:
                return
            data = self.port.read(min(waiting, 4096))
            received_ns = time.monotonic_ns()
        except (serial.SerialException, OSError) as exc:
            self.close_serial(str(exc))
            return
        trace = getattr(self, "latency_trace", None)
        if trace:
            data = trace.receive(data, received_ns)
        parse_begin_ns = time.monotonic_ns() if trace else 0
        frames = self.parser.feed(data)
        parsed_ns = time.monotonic_ns() if trace else 0
        if trace:
            trace.emit("rx_chunk", host_ns=received_ns, byte_count=len(data), raw_hex=data.hex(),
                       parse_begin_ns=parse_begin_ns, parsed_ns=parsed_ns,
                       crc_errors=self.parser.crc_errors, tail_errors=self.parser.tail_errors,
                       discarded_bytes=self.parser.discarded_bytes)
        for frame in frames:
            if trace:
                trace.emit("feedback", host_ns=received_ns, feedback_sequence=frame.sequence,
                           mcu_timestamp_ms=frame.mcu_timestamp_ms, parsed_ns=parsed_ns,
                           parse_begin_ns=parse_begin_ns, yaw=frame.yaw, pitch=frame.pitch, mode=frame.mode)
            self.last_feedback_mode = frame.mode
            if getattr(self, "fire_gate", None) and frame.mode != self.fire_required_mode:
                self.revoke_fire_permission("wrong_feedback_mode")
            self.last_feedback_at = time.monotonic()
            self.publish_state("feedback_alive", True)
            self.publish_feedback(frame)

    def publish_feedback(self, frame) -> None:
        stamp = self.get_clock().now().to_msg()
        message = GimbalFeedback()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.sequence = frame.sequence
        message.mcu_timestamp_ms = frame.mcu_timestamp_ms
        qw, qx, qy, qz = frame.quaternion_wxyz
        message.orientation.w = qw
        message.orientation.x = qx
        message.orientation.y = qy
        message.orientation.z = qz
        message.yaw = frame.yaw
        message.pitch = frame.pitch
        message.mode = frame.mode
        self.feedback_pub.publish(message)

        if frame.quaternion_is_unit:
            imu = Imu()
            imu.header = message.header
            imu.orientation = message.orientation
            imu.angular_velocity_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0
            self.imu_pub.publish(imu)
        elif time.monotonic() - self.last_pose_warning > 2.0:
            self.last_pose_warning = time.monotonic()
            self.get_logger().warning(
                "Feedback quaternion is non-finite or not unit length; IMU message suppressed"
            )

    def revoke_fire_permission(self, reason):
        had_command = self.fire_gate.has_command
        self.fire_gate.invalidate(reason)
        self.publish_state("command_active", False)
        if had_command and self.port is not None:
            try:
                self.port.reset_output_buffer()
            except (serial.SerialException, OSError) as exc:
                self.close_serial(str(exc))

    def fire_permission_callback(self, message: String) -> None:
        if self.receive_only or self.fire_gate is None:
            return
        if self.port is None:
            self.revoke_fire_permission("disconnected")
            return
        had_command = self.fire_gate.has_command
        accepted = self.fire_gate.update(message.data, time.monotonic(),
                                         self.get_clock().now().nanoseconds)
        if not accepted:
            # update() already invalidated the cached frame; flush pending OS bytes too.
            self.publish_state("command_active", False)
            if had_command:
                try:
                    self.port.reset_output_buffer()
                except (serial.SerialException, OSError) as exc:
                    self.close_serial(str(exc))
        trace = getattr(self, "latency_trace", None)
        if trace:
            trace.emit("fire_permission", accepted=accepted, reason=self.fire_gate.reason)

    def command_callback(self, message: AimCommand) -> None:
        # Prevent direct callers as well as ROS topic publishers from bypassing the gate.
        if getattr(self, "fire_gate", None) is not None:
            return
        trace = getattr(self, "latency_trace", None)
        if trace:
            trace.new_source(message)
        if self.port is None:
            self.command_gate.invalidate()
            if trace:
                trace.source_result(False, "disconnected")
            return
        if trace:
            accepted = self.command_gate.update(
                message.valid, message.yaw, message.pitch, time.monotonic(), timing=trace.packed)
            trace.source_result(accepted, "" if accepted else "invalid_or_nonfinite")
        else:
            accepted = self.command_gate.update(
                message.valid, message.yaw, message.pitch, time.monotonic())
        if not accepted:
            if self.transport == "windows_duplex" and self.port is not None:
                if trace:
                    trace.helper_drop(self.port.pending_write, "windows_pending_cancelled")
                self.port.cancel_pending_write()
            self.publish_state("command_active", False)
            if message.valid:
                self.get_logger().error("Rejected non-finite aim command")

    def write_command(self) -> None:
        if self.receive_only:
            return
        now = time.monotonic()
        if getattr(self, "fire_gate", None) is not None:
            if (self.last_feedback_mode != self.fire_required_mode
                    or not 0 <= now - self.last_feedback_at <= self.feedback_timeout):
                self.revoke_fire_permission("feedback_unavailable_or_wrong_mode")
                return
            had_command = self.fire_gate.has_command
            packet = self.fire_gate.packet(now, self.get_clock().now().nanoseconds)
            if had_command and packet is None and self.port is not None:
                try:
                    self.port.reset_output_buffer()
                except (serial.SerialException, OSError) as exc:
                    self.close_serial(str(exc))
        else:
            packet = self.command_gate.packet(now, self.command_timeout)
        trace = getattr(self, "latency_trace", None)
        if trace:
            trace.expire(time.monotonic_ns())
        if packet is None or self.port is None:
            if trace:
                trace.retire_source("source_expired_or_disconnected")
            if self.transport == "windows_duplex" and self.port is not None:
                if trace:
                    trace.helper_drop(self.port.pending_write, "windows_pending_cancelled")
                self.port.cancel_pending_write()
            self.publish_state("command_active", False)
            return
        marks = trace.begin_send() if trace else None
        if trace:
            marks["timer_take_ns"] = time.monotonic_ns()
            marks["send_slot_depth"] = 1
        if trace:
            marks['legacy_wire_hex'] = packet.hex()
        if trace and trace.protocol == "diagnostic_v1":
            marks["wire_pack_start_ns"] = time.monotonic_ns()
            packet = encode_diagnostic_command(marks["wire_session"], marks["wire_sequence"], packet)
            marks["wire_pack_end_ns"] = time.monotonic_ns()
        if trace:
            marks["wire_hex"] = packet.hex()
        try:
            if self.transport == "windows_duplex":
                remaining = self.command_gate.remaining_time(time.monotonic(), self.command_timeout)
                if remaining <= 0:
                    if trace:
                        trace.emit("send_expired_before_call", **marks)
                    self.publish_state("command_active", False)
                    return
                if trace:
                    trace.helper_drop(self.port.pending_write, "windows_pending_overwritten")
                    marks["call_before_ns"] = time.monotonic_ns()
                request_id = self.port.queue_write(packet, timeout=remaining)
                if trace:
                    marks["call_after_ns"] = time.monotonic_ns()
                    trace.requests[request_id] = marks["send_id"]
                    trace.send_returned(marks, "host_queued", submitted=True)
                self.service_duplex_writes()
                return
            else:
                if trace:
                    marks["call_before_ns"] = time.monotonic_ns()
                if getattr(self, "tx_event_pub", None) is not None:
                    write_started_unix_ns = time.time_ns()
                written = self.port.write(packet)
                if getattr(self, "tx_event_pub", None) is not None:
                    write_completed_unix_ns = time.time_ns()
                if trace:
                    marks["call_after_ns"] = time.monotonic_ns()
                    marks["bytes_written"] = written
            if written != len(packet):
                raise serial.SerialTimeoutException(
                    f"short write: {written}/{len(packet)} bytes"
                )
        except TimeoutError as exc:
            if trace and "call_after_ns" not in marks:
                marks["call_after_ns"] = time.monotonic_ns()
                trace.send_returned(marks, "host_timeout", submitted=False)
            self.publish_state("command_active", False)
            self.get_logger().warning(f"Dropped expired aim command: {exc}")
            return
        except (serial.SerialException, OSError) as exc:
            if trace and self.transport != "windows_duplex":
                marks.setdefault("call_after_ns", time.monotonic_ns())
                trace.send_returned(marks, "host_error_or_partial_write", submitted=False)
            elif trace and "call_after_ns" not in marks:
                marks["call_after_ns"] = time.monotonic_ns()
                trace.send_returned(marks, "host_queue_error", submitted=False)
            self.close_serial(str(exc))
            return
        if trace:
            trace.send_returned(marks, "host_write_returned", submitted=True)
        self.publish_state("command_active", True)
        # This event describes a completed native host write, never an MCU ACK.
        if self.transport == "serial" and getattr(self, "tx_event_pub", None) is not None:
            self.tx_event_sequence += 1
            angles = AIM_COMMAND.unpack(packet)[1:3] if len(packet) == AIM_COMMAND.size else (None, None)
            gate = getattr(self, "fire_gate", None)
            payload = dict(
                schema=1, event_sequence=self.tx_event_sequence,
                event="host_write_returned", host_monotonic_ns=time.monotonic_ns(),
                write_started_unix_ns=write_started_unix_ns,
                write_completed_unix_ns=write_completed_unix_ns,
                ros_stamp_ns=self.get_clock().now().nanoseconds,
                command_stamp_ns=gate.command_stamp_ns if gate else None,
                permission_stamp_ns=gate.decision_ns if gate else None,
                yaw_rad=angles[0], pitch_rad=angles[1], bytes_written=written,
                wire_hex=packet.hex(), transport=self.transport)
            self.tx_event_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def update_health(self) -> None:
        trace = getattr(self, "latency_trace", None)
        if trace:
            trace.expire(time.monotonic_ns())
        alive = time.monotonic() - self.last_feedback_at <= self.feedback_timeout
        self.publish_state("feedback_alive", alive)

    def destroy_node(self):
        if getattr(self, "fire_gate", None) is not None:
            self.revoke_fire_permission("shutdown")
        if self.port is not None:
            self.port.close()
            self.port = None
        trace = getattr(self, "latency_trace", None)
        if trace:
            trace.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = SerialBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == "__main__":
    main()
