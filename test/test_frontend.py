from dataclasses import replace
import math
from pathlib import Path
import struct

import numpy as np
import pytest

from hc_teleop_recv.config import ConfigError, load_config, parse_config, validate_motion_channels
from hc_teleop_recv.frontend import Frontend, InputSession
from hc_teleop_recv.pose_math import quaternion_from_axis_angle, quaternion_to_matrix, relative_target
from hc_teleop_recv.protocol import (
    PACKET_FORMAT, LEGACY_PACKET_FORMAT, PacketError, Pose, PosePacket, ControllerInput,
    decode_pose_packet, decode_vrdata,
)


IDENTITY = Pose((0., 0., 0.), (0., 0., 0., 1.))
FK = Pose((.2, .3, .4), (0., 0., 0., 1.))


def document():
    return {
        "schema_version": 1,
        "control": {"enabled_on_start": True},
        "channels": [{
            "id": "arm", "controller": "right", "target_pose_topic": "/teleop/arm/servo_p",
            "fk_pose_topic": "/teleop/arm/fk_pose", "base_frame": "base", "tool_frame": "tool",
            "axis_mapping": [[0, 0, -1], [-1, 0, 0], [0, 1, 0]], "filter_alpha": 1.,
        }],
    }


def packet(seq=0, grip=0., position=(0., 0., 0.), *, flags=7, quat=(0., 0., 0., 1.)):
    return PosePacket(2, seq, 1. + seq * .01, flags, IDENTITY, IDENTITY,
                      Pose(position, quat), ControllerInput(), ControllerInput(grip=grip))


def wire(frame):
    values = [b"PICO", frame.protocol_version, frame.sequence, frame.vr_timestamp, frame.flags]
    for p in (frame.head, frame.left, frame.right):
        values.extend((*p.position, *p.quaternion))
    if frame.protocol_version == 2:
        for inp in (frame.left_input, frame.right_input):
            values.extend((inp.held_mask, inp.pressed_mask, inp.released_mask, inp.trigger, inp.grip,
                           *inp.primary_axis, *inp.secondary_axis))
    return struct.pack(PACKET_FORMAT if frame.protocol_version == 2 else LEGACY_PACKET_FORMAT, *values)


def bound_frontend(config=None):
    f = Frontend(config or parse_config(document()))
    for ident in f.channels:
        f.update_fk(ident, FK, 0.)
    f.ingest(packet(0, 0.), 0.)
    f.ingest(packet(1, 1.), .01)
    return f


def test_clutch_starts_at_measured_fk_and_rebinds_after_release():
    f = bound_frontend()
    assert f.tick(.01)["arm"] == FK
    f.ingest(packet(2, 1., (0., 0., -.1)), .02)
    np.testing.assert_allclose(f.tick(.02)["arm"].position, [.28, .3, .4])
    f.ingest(packet(3, 0., (0., 0., -.1)), .03)
    assert f.tick(.03) == {}
    new_fk = Pose((.5, .4, .3), FK.quaternion)
    f.update_fk("arm", new_fk, .04)
    f.ingest(packet(4, 1., (0., 0., -.5)), .04)
    assert f.tick(.04)["arm"] == new_fk


def test_starting_with_grip_held_never_binds_or_uses_zero_fk():
    f = Frontend(parse_config(document()))
    f.ingest(packet(0, 1.), 0.)
    assert f.tick(0.) == {}
    f.ingest(packet(1, 0.), .01)
    f.ingest(packet(2, 1.), .02)
    assert f.tick(.02) == {}
    f.update_fk("arm", FK, .03)
    f.ingest(packet(3, 1.), .03)
    assert f.tick(.03) == {}
    f.ingest(packet(4, 0.), .04)
    f.ingest(packet(5, 1.), .05)
    assert f.tick(.05)["arm"] == FK


@pytest.mark.parametrize("failure", ["input", "fk", "tracking", "clutch_tracking"])
def test_failure_stops_output_and_requires_grip_release(failure):
    f = bound_frontend()
    if failure == "input":
        f.update_fk("arm", FK, .4)
        assert f.tick(.4) == {}
    elif failure == "fk":
        f.ingest(packet(2, 1.), .2)
        f.ingest(packet(3, 1.), .3)
        assert f.tick(.3) == {}
    else:
        if failure == "clutch_tracking":
            cfg = replace(f.config.channels[0], controller="left")
            f.channels["arm"].config = cfg
        f.ingest(packet(2, 1., flags=3), .02)
        assert f.tick(.02) == {}
    f.update_fk("arm", FK, .41)
    f.ingest(packet(50, 1.), .41)
    assert f.tick(.41) == {}
    f.ingest(packet(51, 0.), .42)
    f.ingest(packet(52, 1.), .43)
    assert f.tick(.43)["arm"] == FK


def test_explicit_disable_and_a_resume_require_release():
    f = bound_frontend()
    f.set_enabled(False)
    assert f.tick(.02) == {}
    p = packet(2, 1.)
    p = replace(p, right_input=ControllerInput(grip=1., held_mask=1))
    f.ingest(p, .02)
    assert f.enabled
    assert f.tick(.02) == {}
    f.ingest(packet(3, 0.), .03)
    f.ingest(packet(4, 1.), .04)
    assert f.tick(.04)["arm"] == FK


def test_world_increment_rotation_left_multiplies_nontrivial_reference():
    cfg = parse_config(document()).channels[0]
    q0 = quaternion_from_axis_angle([1, 0, 0], .6)
    qh = quaternion_from_axis_angle([0, 1, 0], .2)
    _, q = relative_target([0, 0, 0], qh, [0, 0, 0], [0, 0, 0, 1],
                           [0, 0, 0], q0, cfg.axis_mapping, .8, .8, True)
    expected = quaternion_to_matrix(quaternion_from_axis_angle([0, 0, 1], .2)) @ quaternion_to_matrix(q0)
    np.testing.assert_allclose(quaternion_to_matrix(q), expected, atol=1e-12)


def test_shoulder_mapping_matches_original_body_to_shoulder_chain():
    body_map = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    body_fk_p = np.array([.2, .1, .6])
    body_fk_q = quaternion_from_axis_angle([1, 2, 3], .7)
    hand_q = quaternion_from_axis_angle([2, 1, 0], .3)
    body_target = relative_target([.1, .05, -.2], hand_q, [0, 0, 0], [0, 0, 0, 1],
                                  body_fk_p, body_fk_q, body_map, .8, .8, True)
    from hc_teleop_recv.pose_math import matrix_to_quaternion
    for angle, y in ((1.5708, -.031), (-1.5708, .031)):
        r = quaternion_to_matrix(quaternion_from_axis_angle([1, 0, 0], angle))
        offset = np.array([0, y, .735])
        shoulder_fk = (r.T @ (body_fk_p - offset), matrix_to_quaternion(r.T @ quaternion_to_matrix(body_fk_q)))
        target = relative_target([.1, .05, -.2], hand_q, [0, 0, 0], [0, 0, 0, 1],
                                 *shoulder_fk, r.T @ body_map, .8, .8, True)
        np.testing.assert_allclose(target[0], r.T @ (body_target[0] - offset), atol=1e-12)
        np.testing.assert_allclose(quaternion_to_matrix(target[1]), r.T @ quaternion_to_matrix(body_target[1]), atol=1e-12)


def test_configurable_bimanual_binding_has_independent_references_and_axes():
    doc = document()
    doc["channels"].append(dict(doc["channels"][0], id="another_arm", controller="left", clutch_controller="right",
                                target_pose_topic="/other/servo_p", fk_pose_topic="/other/fk_pose",
                                base_frame="other_base", tool_frame="other_tool", axis_mapping=np.eye(3).tolist()))
    f = bound_frontend(parse_config(doc))
    moved = replace(packet(2, 1., (0, 0, -.1)), left=Pose((.1, 0, 0), FK.quaternion))
    f.ingest(moved, .02)
    targets = f.tick(.02)
    np.testing.assert_allclose(targets["arm"].position, [.28, .3, .4])
    np.testing.assert_allclose(targets["another_arm"].position, [.28, .3, .4])


def test_displacement_workspace_and_filtering_are_configurable():
    doc = document()
    doc["channels"][0].update(max_displacement=.1, filter_alpha=.5,
                              workspace={"min": [0, 0, 0], "max": [.25, 1, 1]})
    f = bound_frontend(parse_config(doc))
    f.ingest(packet(2, 1., (0, 0, -1.)), .02)
    np.testing.assert_allclose(f.tick(.02)["arm"].position, [.225, .3, .4])


def test_workspace_outside_fk_does_not_jump_on_bind():
    doc = document()
    doc["channels"][0]["workspace"] = {"min": [1, 1, 1], "max": [2, 2, 2]}
    f = bound_frontend(parse_config(doc))
    assert f.tick(.01) == {}


@pytest.mark.parametrize("version", [1, 2])
def test_wire_and_json_round_trip(version):
    p = replace(packet(), protocol_version=version)
    decoded = decode_pose_packet(wire(p))
    assert decoded == p
    assert decode_vrdata(decoded.as_dict()) == p


@pytest.mark.parametrize("change", [
    {"flags": 8}, {"vr_timestamp": float("nan")}, {"right": Pose((6., 0, 0), IDENTITY.quaternion)},
    {"right": Pose((0, 0, 0), (0, 0, 0, 0))}, {"right_input": ControllerInput(grip=1.5)},
    {"right_input": ControllerInput(primary_axis=(2., 0))}, {"right_input": ControllerInput(held_mask=32768)},
])
def test_bad_wire_and_json_packets_are_rejected(change):
    p = replace(packet(), **change)
    with pytest.raises(PacketError):
        decode_pose_packet(wire(p))
    data = p.as_dict()
    if "flags" in change:
        data["tracking"]["right"] = "invalid"
    with pytest.raises(PacketError):
        decode_vrdata(data)


@pytest.mark.parametrize("payload", [b"", b"PICO", b"X" * 162, wire(packet()) + b"x"])
def test_malformed_datagrams(payload):
    with pytest.raises(PacketError):
        decode_pose_packet(payload)


def test_session_rejects_replay_frozen_timestamp_and_second_sender_until_expiry():
    s = InputSession(.25)
    p = replace(packet(), sequence=2**32-1)
    assert s.accept(p, "pico1", 0)
    assert not s.accept(p, "pico1", .01)
    newer = replace(p, sequence=0, vr_timestamp=2.)
    assert not s.accept(newer, "pico2", .02)
    assert s.accept(newer, "pico1", .02)
    assert not s.accept(replace(newer, sequence=1), "pico1", .03)
    assert s.accept(packet(), "pico2", .4)
    assert s.restarted


@pytest.mark.parametrize("field,value", [
    ("axis_mapping", [[1, 0, 0], [0, 1, 0], [0, 0, -1]]),
    ("axis_mapping", [[2, 0, 0], [0, 1, 0], [0, 0, 1]]),
    ("filter_alpha", 0), ("position_scale", float("nan")),
    ("clutch_threshold", "bad"), ("orientation_enabled", "false"),
    ("target_pose_topic", "/teleop/arm/fk_pose"), ("controller", "unknown"),
])
def test_invalid_channel_config(field, value):
    doc = document()
    doc["channels"][0][field] = value
    with pytest.raises(ConfigError):
        parse_config(doc)


def test_example_and_motion_contract_allow_an_unrelated_robot():
    path = Path(__file__).resolve().parents[1] / "config/single_arm.example.yaml"
    cfg = load_config(path)
    c = cfg.channels[0]
    motion = {"kind": "servo_p", "endpoint": c.target_pose_topic, "base_frame": c.base_frame,
              "tip_frame": c.tool_frame, "fk_pose_topic": c.fk_pose_topic}
    validate_motion_channels(cfg, [motion])
    for key in ("kind", "base_frame", "tip_frame", "fk_pose_topic", "endpoint"):
        with pytest.raises(ConfigError):
            validate_motion_channels(cfg, [dict(motion, **{key: "mismatch"})])


def test_adapter_identity_and_button_topic_validation():
    doc = document()
    doc['adapter'] = {'robot_id':'lab_arm','buttons_topic':'/lab/buttons'}
    config = parse_config(doc)
    assert config.robot_id == 'lab_arm'
    assert config.buttons_topic == '/lab/buttons'
    for bad in ('../bad', 'Bad ID'):
        doc['adapter']['robot_id'] = bad
        with pytest.raises(ConfigError):
            parse_config(doc)
    doc['adapter'] = {'buttons_topic':'/teleop/arm/servo_p'}
    with pytest.raises(ConfigError):
        parse_config(doc)
