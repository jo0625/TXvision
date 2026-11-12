"""Fresh, command-bound fire permits for the native serial transmitter."""
import json
import math
from protocol import CommandGate


class FireTransmitGate:
    def __init__(self, permission_timeout=0.1, command_max_age=0.2,
                 expected_frame="controller_joint_targets_preview"):
        if not all(math.isfinite(x) and x > 0 for x in
                   (permission_timeout, command_max_age)) or not expected_frame:
            raise ValueError("Invalid fire permission limits")
        self.permission_timeout = permission_timeout
        self.command_max_age = command_max_age
        self.expected_frame = expected_frame
        self.command_gate = CommandGate()
        self.highest_decision_ns = 0
        self.command_stamp_ns = 0
        self.revoked_command_ns = 0
        self.decision_ns = 0
        self.received = None
        self.last_ros_ns = None
        self.reason = "waiting_for_permission"

    @property
    def has_command(self):
        return self.received is not None

    def invalidate(self, reason="revoked"):
        self.revoked_command_ns = max(self.revoked_command_ns, self.command_stamp_ns)
        self.command_gate.invalidate()
        self.received = None
        self.reason = reason

    def _clocks_valid(self, now, ros_ns):
        if not math.isfinite(now) or not isinstance(ros_ns, int) or ros_ns <= 0:
            self.invalidate("invalid_clock")
            return False
        if self.last_ros_ns is not None and ros_ns < self.last_ros_ns:
            self.invalidate("clock_reset")
            self.highest_decision_ns = self.revoked_command_ns = self.command_stamp_ns = 0
            self.last_ros_ns = ros_ns
            return False
        self.last_ros_ns = ros_ns
        return True

    @staticmethod
    def _integer(value):
        return type(value) is int and value > 0

    def update(self, data, now, ros_ns):
        if not self._clocks_valid(now, ros_ns):
            return False
        try:
            d = json.loads(data)
            if not isinstance(d, dict):
                raise ValueError("invalid_schema")
            if d.get("schema") != 1 or d.get("preview_only") is not False:
                raise ValueError("preview_or_unknown_schema")
            decision_ns = d.get("stamp_ns")
            if not self._integer(decision_ns):
                raise ValueError("invalid_decision_stamp")
            if not 0 <= (ros_ns-decision_ns)*1e-9 < self.permission_timeout:
                raise ValueError("expired_or_future_permission")
            if decision_ns <= self.highest_decision_ns:
                raise ValueError("replayed_permission")
            self.highest_decision_ns = decision_ns
            if d.get("allowed") is not True or d.get("reason") != "ready":
                raise ValueError("permission_denied")
            stamp = d.get("command_stamp_ns")
            if not self._integer(stamp) or not 0 <= (ros_ns-stamp)*1e-9 < self.command_max_age:
                raise ValueError("expired_or_future_command")
            if stamp <= self.revoked_command_ns or stamp < self.command_stamp_ns:
                raise ValueError("revoked_or_reordered_command")
            if d.get("command_frame") != self.expected_frame:
                raise ValueError("command_frame_mismatch")
            yaw, pitch = d.get("command_yaw"), d.get("command_pitch")
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in (yaw,pitch)):
                raise ValueError("invalid_angles")
            if not self.command_gate.update(True, yaw, pitch, now):
                raise ValueError("invalid_angles")
        except (ValueError, TypeError, OverflowError) as exc:
            self.invalidate(str(exc))
            return False
        self.command_stamp_ns, self.decision_ns = stamp, decision_ns
        self.received = now
        self.reason = "ready"
        return True

    def packet(self, now, ros_ns):
        if not self._clocks_valid(now, ros_ns) or self.received is None:
            return None
        if (not 0 <= now-self.received < self.permission_timeout
                or not 0 <= (ros_ns-self.decision_ns)*1e-9 < self.permission_timeout
                or not 0 <= (ros_ns-self.command_stamp_ns)*1e-9 < self.command_max_age):
            self.invalidate("permission_or_command_timeout")
            return None
        return self.command_gate.packet(now, self.permission_timeout)
