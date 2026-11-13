#!/usr/bin/env python3
"""Compute yaw and optional ballistic pitch for the nearest reconstructed armor."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from auto_aim_interfaces.msg import AimCommand, Target
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
import tf2_geometry_msgs  # Registers PointStamped conversions with tf2.
from tf2_ros import Buffer, TransformException, TransformListener

from aimer_geometry import (
    Point3,
    geometric_angles,
    reconstruct_armor_points,
    select_closest_forward_point,
)

from ballistic_solver import solve_ballistic_pitch
from impact_speed_calibration import load_speed_calibration


class BasicAimerNode(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__("aimer", parameter_overrides=parameter_overrides)
        stability_path = str(self.declare_parameter("stability_trace_path", "").value)
        self.stability_trace = None
        self.stability_source = None
        if stability_path:
            from aim_stability_trace import BufferedTrace
            self.stability_trace = BufferedTrace(stability_path)
        self.input_topic = str(
            self.declare_parameter("input_topic", "/tracker/target").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/aimer/aim_command").value
        )
        self.aim_frame = str(self.declare_parameter("aim_frame", "gimbal_link").value)
        self.target_timeout = float(self.declare_parameter("target_timeout", 0.2).value)
        self.check_period = float(self.declare_parameter("check_period", 0.02).value)
        self.tf_timeout = float(self.declare_parameter("tf_timeout", 0.02).value)
        self.prediction_time = float(
            self.declare_parameter("prediction_time", 0.0).value
        )
        self.minimum_forward = float(
            self.declare_parameter("minimum_forward", 0.05).value
        )
        self.minimum_distance = float(
            self.declare_parameter("minimum_distance", 0.1).value
        )
        self.ballistics_enabled = bool(
            self.declare_parameter("ballistics_enabled", False).value
        )
        self.gravity_frame = str(
            self.declare_parameter("gravity_frame", "odom").value
        )
        self.muzzle_frame = str(
            self.declare_parameter("muzzle_frame", "").value
        )
        self.projectile_speed = float(
            self.declare_parameter("projectile_speed", 0.0).value
        )
        self.gravity = float(self.declare_parameter("gravity", 9.81).value)
        self.projectile_speed_source = str(
            self.declare_parameter("projectile_speed_source", "configured").value)
        calibration_path = str(self.declare_parameter("speed_calibration_file", "").value)
        self.speed_calibration_id = ""
        self.speed_calibration_gear = ""
        if calibration_path:
            fit = load_speed_calibration(Path(calibration_path), self.gravity)
            self.projectile_speed = fit["speed_mps"]
            self.projectile_speed_source = "impact_fit"
            self.speed_calibration_id = fit["calibration_id"]
            self.speed_calibration_gear = fit["gear"]
            self.set_parameters([
                Parameter("projectile_speed", value=self.projectile_speed),
                Parameter("projectile_speed_source", value=self.projectile_speed_source)])
            self.get_logger().info(
                f"Impact-fit effective speed: {self.projectile_speed:.3f} m/s, "
                f"gear={fit['gear']}, shots={fit['sample_count']}, "
                f"height RMS={fit['height_residual_rms_m']*1000:.2f} mm")
            for warning in fit["warnings"]:
                self.get_logger().warning(f"Speed calibration: {warning}")
        if min(
            self.target_timeout,
            self.check_period,
            self.tf_timeout,
            self.minimum_forward,
            self.minimum_distance,
        ) <= 0.0:
            raise ValueError("timeouts and minimum distances must be positive")
        if self.prediction_time < 0.0:
            raise ValueError("prediction_time must not be negative")
        if self.ballistics_enabled and (
            not math.isfinite(self.projectile_speed)
            or not math.isfinite(self.gravity)
            or self.projectile_speed <= 0.0
            or self.gravity <= 0.0
        ):
            raise ValueError("ballistics require positive projectile_speed and gravity")
        if self.ballistics_enabled and (not self.gravity_frame or not self.muzzle_frame):
            raise ValueError("ballistics require gravity_frame and muzzle_frame")

        self.solution_pub = (
            self.create_publisher(String, "~/solution", qos_profile_sensor_data)
            if self.declare_parameter("publish_diagnostics", False).value else None)
        self.solution_sequence = 0
        self.solution_source_ns = None
        self.ballistic_observation = {}
        self.publisher = self.create_publisher(
            AimCommand, self.output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            Target, self.input_topic, self.target_callback, qos_profile_sensor_data
        )
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_target_at = float("-inf")
        self.timeout_published = False
        self.last_warning_at = float("-inf")
        self.create_timer(self.check_period, self.check_timeout)
        self.get_logger().info(
            f"Aimer ({'ballistic' if self.ballistics_enabled else 'geometric'}): "
            f"{self.input_topic} -> {self.output_topic}, "
            f"aim_frame={self.aim_frame}; serial output is not connected"
        )

    def publish_solution(self, valid, reason, **fields):
        if self.solution_pub is None:
            return
        self.solution_sequence += 1
        payload = dict(
            schema=1, event_sequence=self.solution_sequence,
            source_stamp_ns=self.solution_source_ns,
            ros_stamp_ns=self.get_clock().now().nanoseconds,
            host_monotonic_ns=time.monotonic_ns(), valid=valid, reason=reason,
            frame_id=self.aim_frame, gravity_frame=self.gravity_frame,
            muzzle_frame=self.muzzle_frame, ballistics_enabled=self.ballistics_enabled,
            projectile_speed_mps=self.projectile_speed, gravity_mps2=self.gravity,
            projectile_speed_source=self.projectile_speed_source,
            speed_calibration_id=self.speed_calibration_id,
            speed_calibration_gear=self.speed_calibration_gear,
            prediction_time_s=self.prediction_time, **fields)
        self.solution_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def publish_invalid(self, reason="invalid_target") -> None:
        trace = getattr(self, 'stability_trace', None)
        if trace:
            trace.emit('aimer_invalid', source_stamp_ns=self.stability_source)
        command = AimCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self.aim_frame
        command.valid = False
        self.publisher.publish(command)
        self.publish_solution(False, reason)

    def warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warning_at >= 1.0:
            self.last_warning_at = now
            self.get_logger().warning(message)

    def transform_point(self, point: Point3, target: Target, destination: str) -> Point3:
        if not target.header.frame_id:
            raise ValueError("target frame_id is empty")
        if target.header.frame_id == destination:
            return point
        stamped = PointStamped()
        stamped.header = target.header
        stamped.point.x = point.x
        stamped.point.y = point.y
        stamped.point.z = point.z
        transformed = self.tf_buffer.transform(
            stamped, destination, timeout=Duration(seconds=self.tf_timeout)
        )
        return Point3(
            transformed.point.x, transformed.point.y, transformed.point.z
        )

    def muzzle_origin(self, target: Target) -> Point3:
        stamped = PointStamped()
        stamped.header.stamp = target.header.stamp
        stamped.header.frame_id = self.muzzle_frame
        return self.transform_point(Point3(0.0, 0.0, 0.0), stamped, self.gravity_frame)

    def ballistic_angles(self, point: Point3, target: Target, muzzle: Point3):
        world_point = self.transform_point(point, target, self.gravity_frame)
        relative = Point3(
            world_point.x - muzzle.x,
            world_point.y - muzzle.y,
            world_point.z - muzzle.z,
        )
        world_yaw, line_pitch = geometric_angles(relative)
        world_pitch, flight_time = solve_ballistic_pitch(
            relative, self.projectile_speed, self.gravity
        )
        direction = Vector3Stamped()
        direction.header.stamp = target.header.stamp
        direction.header.frame_id = self.gravity_frame
        direction.vector.x = math.cos(world_pitch) * math.cos(world_yaw)
        direction.vector.y = math.cos(world_pitch) * math.sin(world_yaw)
        direction.vector.z = math.sin(world_pitch)
        local = self.tf_buffer.transform(
            direction, self.aim_frame, timeout=Duration(seconds=self.tf_timeout)
        )
        yaw, pitch = geometric_angles(
            Point3(local.vector.x, local.vector.y, local.vector.z)
        )
        if self.solution_pub is not None:
            self.ballistic_observation = dict(
                muzzle_position_gravity_m=[muzzle.x, muzzle.y, muzzle.z],
                target_position_gravity_m=[world_point.x, world_point.y, world_point.z],
                relative_target_gravity_m=[relative.x, relative.y, relative.z],
                distance_from_muzzle_m=math.sqrt(relative.x**2 + relative.y**2 + relative.z**2),
                horizontal_distance_m=math.hypot(relative.x, relative.y),
                line_of_sight_pitch_gravity_rad=line_pitch,
                ballistic_pitch_gravity_rad=world_pitch,
                gravity_compensation_rad=world_pitch - line_pitch)
        return yaw, pitch, flight_time

    def target_callback(self, target: Target) -> None:
        self.solution_source_ns = (
            target.header.stamp.sec * 1_000_000_000 + target.header.stamp.nanosec)
        self.ballistic_observation = {}
        trace = getattr(self, 'stability_trace', None)
        if trace:
            self.stability_source = target.header.stamp.sec*1_000_000_000+target.header.stamp.nanosec
            trace.emit('aimer_input', source_stamp_ns=self.stability_source,
                       target_id=target.id, frame_id=target.header.frame_id,
                       tracking=target.tracking, armor_count=target.armors_num)
        self.last_target_at = time.monotonic()
        self.timeout_published = False
        if not target.tracking:
            self.publish_invalid("not_tracking")
            return
        try:
            muzzle = self.muzzle_origin(target) if self.ballistics_enabled else None
            flight_time = 0.0
            for iteration in range(10 if self.ballistics_enabled else 1):
                candidates = reconstruct_armor_points(
                    Point3(target.position.x, target.position.y, target.position.z),
                    Point3(target.velocity.x, target.velocity.y, target.velocity.z),
                    target.yaw,
                    target.v_yaw,
                    target.armors_num,
                    target.radius_1,
                    target.radius_2,
                    target.dz,
                    self.prediction_time + flight_time,
                )
                transformed = [
                    self.transform_point(point, target, self.aim_frame)
                    for point in candidates
                ]
                aim_point = select_closest_forward_point(
                    transformed, self.minimum_forward, self.minimum_distance
                )
                selected_index = transformed.index(aim_point)
                if not self.ballistics_enabled:
                    yaw, pitch = geometric_angles(aim_point)
                    flight_time = None
                    break
                yaw, pitch, next_flight_time = self.ballistic_angles(
                    candidates[selected_index], target, muzzle
                )
                if abs(next_flight_time - flight_time) < 0.001:
                    flight_time = next_flight_time
                    break
                flight_time = next_flight_time
            else:
                raise ValueError("ballistic flight time did not converge")
            if trace:
                trace.emit('aimer_solution', source_stamp_ns=self.stability_source,
                    target_id=target.id, frame_id=self.aim_frame,
                    ballistic_iterations=iteration + 1,
                    selected_armor_index=selected_index,
                    candidates_m=[(p.x,p.y,p.z) for p in transformed],
                    aim_x_m=aim_point.x, aim_y_m=aim_point.y, aim_z_m=aim_point.z,
                    distance_m=(aim_point.x**2+aim_point.y**2+aim_point.z**2)**0.5,
                    yaw_rad=yaw, pitch_rad=pitch, flight_time_s=flight_time)
        except (ValueError, TransformException) as exc:
            if trace:
                trace.emit('aimer_failure', source_stamp_ns=self.stability_source, reason=str(exc))
            self.warn_throttled(f"Cannot produce aim command: {exc}")
            self.publish_invalid(str(exc))
            return

        command = AimCommand()
        command.header.stamp = target.header.stamp
        command.header.frame_id = self.aim_frame
        command.valid = True
        command.yaw = yaw
        command.pitch = pitch
        self.publisher.publish(command)
        self.publish_solution(
            True, "solved", target_id=target.id,
            yaw_rad=yaw, pitch_rad=pitch, flight_time_s=flight_time,
            ballistic_iterations=iteration + 1, selected_armor_index=selected_index,
            selected_point_aim_m=[aim_point.x, aim_point.y, aim_point.z],
            **self.ballistic_observation)

    def check_timeout(self) -> None:
        if self.timeout_published:
            return
        if time.monotonic() - self.last_target_at <= self.target_timeout:
            return
        if self.last_target_at == float("-inf"):
            return
        self.publish_invalid("target_timeout")
        self.timeout_published = True
        self.get_logger().warning("Target timed out; published valid=false")

    def destroy_node(self):
        if getattr(self, 'stability_trace', None):
            self.stability_trace.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = BasicAimerNode()
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
