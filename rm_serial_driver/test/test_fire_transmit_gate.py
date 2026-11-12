import json
import math
import struct
import pytest
from fire_transmit_gate import FireTransmitGate

BASE = 100_000_000_000


def permit(t=0, **kwargs):
    d = dict(schema=1, preview_only=False, allowed=True, reason="ready",
             stamp_ns=BASE+int(t*1e9), command_stamp_ns=BASE+int(t*1e9),
             command_frame="controller_joint_targets_preview", command_yaw=.25, command_pitch=-.125)
    d.update(kwargs)
    return json.dumps(d)


def grant(g, t=0, **kwargs):
    return g.update(permit(t, **kwargs), t, BASE+int(t*1e9))


def packet(g, t=0, ros_ns=None):
    return g.packet(t, BASE+int(t*1e9) if ros_ns is None else ros_ns)


def test_legacy_wire_and_repetition():
    g=FireTransmitGate()
    assert packet(g) is None
    assert grant(g)
    expected=struct.pack("<BffBB",0xa5,.25,-.125,0x5a,0x5b)
    assert packet(g)==expected
    assert packet(g,.05)==expected


def test_denial_clears_cache_and_old_command_cannot_restart():
    g=FireTransmitGate();assert grant(g)
    assert not grant(g,.01,allowed=False,reason="not_aligned")
    assert packet(g,.02) is None
    assert not grant(g,.03,command_stamp_ns=BASE)
    assert grant(g,.04)
    assert packet(g,.04)


def test_expired_lease_clears_cache_even_when_ros_clock_pauses():
    g=FireTransmitGate();assert grant(g)
    assert packet(g,.11,ros_ns=BASE) is None
    assert not g.has_command
    assert not grant(g,.12,command_stamp_ns=BASE)


def test_republishing_old_source_does_not_extend_source_lifetime():
    g=FireTransmitGate();assert grant(g)
    assert grant(g,.09,command_stamp_ns=BASE)
    assert grant(g,.18,command_stamp_ns=BASE)
    assert packet(g,.201) is None


@pytest.mark.parametrize("change", [
    {"preview_only": True}, {"allowed": False}, {"allowed": 1},
    {"schema": 2}, {"reason": "settling"}, {"command_frame": "pitch_joint"},
    {"command_yaw": float("nan")}, {"command_pitch": float("inf")},
    {"command_yaw": True}, {"command_yaw": 1e100},
    {"command_stamp_ns": 0}, {"command_stamp_ns": BASE-1_000_000_000},
    {"command_stamp_ns": BASE+1_000_000_000}, {"stamp_ns": BASE+1_000_000_000},
    {"stamp_ns": BASE-1_000_000_000},
])
def test_invalid_permit_clears_existing_packet(change):
    g=FireTransmitGate();assert grant(g)
    assert not grant(g,.01,**change)
    assert packet(g,.02) is None


@pytest.mark.parametrize("data", ["garbage", "[]", "null", "{}"])
def test_malformed_json_revokes(data):
    g=FireTransmitGate();assert grant(g)
    assert not g.update(data,.01,BASE+10_000_000)
    assert packet(g,.02) is None


def test_replay_and_reconnect_do_not_restore_packet():
    g=FireTransmitGate();assert grant(g)
    assert not g.update(permit(),.01,BASE+10_000_000)
    assert packet(g,.02) is None
    assert grant(g,.03)
    g.invalidate("connection_reset")
    assert packet(g,.04) is None
    assert not grant(g,.05,command_stamp_ns=BASE+30_000_000)


def test_clock_rollback_requires_new_permit():
    g=FireTransmitGate();assert grant(g,.1)
    assert packet(g,.11,ros_ns=BASE) is None
    assert grant(g,.12)


@pytest.mark.parametrize("kwargs", [{"permission_timeout":0},{"command_max_age":math.nan},{"expected_frame":""}])
def test_invalid_limits(kwargs):
    with pytest.raises(ValueError):FireTransmitGate(**kwargs)
