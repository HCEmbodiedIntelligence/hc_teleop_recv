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


def test_each_packet_produces_at_most_one_target_and_backlog_keeps_timeouts():
    f = bound_frontend()
    assert f.tick(.01)
    assert f.tick(.02) == {}
    f.ingest(packet(2, 1., (0., 0., -.1)), .03)
    assert f.tick(.03, input_ready=False) == {}
    assert f.tick(.04)["arm"].position[0] == pytest.approx(.28)
    assert f.tick(.05) == {}
    assert f.tick(.5, input_ready=False) == {}
    assert f.channels['arm'].state != 'active'


def test_starting_with_grip_held_binds_when_fresh_fk_arrives():
    f = Frontend(parse_config(document()))
    f.ingest(packet(0, 1.), 0.)
    assert f.tick(0.) == {}
    f.ingest(packet(1, 0.), .01)
    f.ingest(packet(2, 1.), .02)
    assert f.tick(.02) == {}
    f.update_fk("arm", FK, .03)
    f.ingest(packet(3, 1.), .03)
    assert f.tick(.03)["arm"] == FK
    f.ingest(packet(4, 0.), .04)
    f.ingest(packet(5, 1.), .05)
    assert f.tick(.05)["arm"] == FK


@pytest.mark.parametrize("failure", ["input", "fk", "tracking", "clutch_tracking"])
def test_failure_stops_output_and_rebinds_with_grip_held(failure):
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
    assert f.tick(.41)["arm"] == FK
    f.ingest(packet(51, 0.), .42)
    f.ingest(packet(52, 1.), .43)
    assert f.tick(.43)["arm"] == FK


def test_explicit_disable_and_a_resume_bind_with_grip_held():
    f = bound_frontend()
    f.set_enabled(False)
    assert f.tick(.02) == {}
    p = packet(2, 1.)
    p = replace(p, right_input=ControllerInput(grip=1., held_mask=1))
    f.ingest(p, .02)
    assert f.enabled
    assert f.tick(.02)["arm"] == FK
    f.ingest(packet(3, 0.), .03)
    f.ingest(packet(4, 1.), .04)
    assert f.tick(.04)["arm"] == FK


@pytest.mark.parametrize('resume', ['startup', 'service', 'a_button'])
def test_arm_switch_preserves_both_channels_and_blocks_all_resume_paths(resume):
    doc = document()
    doc['channels'].append(dict(doc['channels'][0], id='left_arm', controller='left',
        clutch_controller='right', target_pose_topic='/teleop/left/servo_p', fk_pose_topic='/teleop/left/fk_pose'))
    doc['control'].update(arm_control_enabled=False, enabled_on_start=resume == 'startup')
    cfg = parse_config(doc)
    assert len(cfg.channels) == 2
    f = Frontend(cfg)
    if resume == 'service':
        f.set_enabled(True)
    for seq, grip in enumerate([0., 1., 0., 1.]):
        now = seq * .01
        for ident in f.channels:
            f.update_fk(ident, FK, now)
        p = packet(seq, grip, (0., 0., -.1))
        if resume == 'a_button':
            p = replace(p, right_input=ControllerInput(grip=grip, held_mask=1))
        f.ingest(p, now)
        assert f.enabled
        assert f.tick(now) == {}
        assert all(c.state == 'disabled' and c.reference_fk is None for c in f.channels.values())
    assert f.status()['arm_control_enabled'] is False
    assert all(c['reason'] == '机械臂遥操作已关闭' for c in f.status()['channels'].values())


@pytest.mark.parametrize('invalid', [0, 1, 'false', None])
def test_arm_switch_requires_boolean_and_defaults_to_existing_behavior(invalid):
    doc = document()
    assert parse_config(doc).arm_control_enabled is True
    doc['control']['arm_control_enabled'] = invalid
    with pytest.raises(ConfigError, match='arm_control_enabled'):
        parse_config(doc)


def test_world_increment_rotation_left_multiplies_nontrivial_reference():
    cfg = parse_config(document()).channels[0]
    q0 = quaternion_from_axis_angle([1, 0, 0], .6)
    qh = quaternion_from_axis_angle([0, 1, 0], .2)
    _, q = relative_target([0, 0, 0], qh, [0, 0, 0], [0, 0, 0, 1],
                           [0, 0, 0], q0, cfg.axis_mapping, .8, .8, True)
    expected = quaternion_to_matrix(quaternion_from_axis_angle([0, 0, 1], .2)) @ quaternion_to_matrix(q0)
    np.testing.assert_allclose(quaternion_to_matrix(q), expected, atol=1e-12)


@pytest.mark.parametrize('controller', ['left', 'right'])
def test_openarmx_downward_controller_lifts_tool_forward_without_a_bind_jump(controller):
    doc = document()
    doc['channels'][0].update(controller=controller, clutch_controller='right',
        axis_mapping=[[0, 0, -1], [1, 0, 0], [0, -1, 0]], position_axis_signs=[-1, -1, -1])
    frontend = Frontend(parse_config(doc))
    tool_down = Pose(FK.position, tuple(quaternion_from_axis_angle([0, 1, 0], math.pi)))
    frontend.update_fk('arm', tool_down, 0.)
    for step, lifted in enumerate((0., math.pi / 6, math.pi / 3, math.pi / 2)):
        # Controller +Z initially points down (-Y); lifting toward its trigger
        # brings +Z forward. Tool +Z must move from body -Z toward body +X.
        hand = Pose(IDENTITY.position, tuple(quaternion_from_axis_angle([1, 0, 0], math.pi / 2 - lifted)))
        frame = replace(packet(step, 1.), **{controller: hand})
        frontend.ingest(frame, step * .01)
        target = frontend.tick(step * .01)['arm']
        assert target.position == tool_down.position
        expected_forward = [math.sin(lifted), 0., -math.cos(lifted)]
        np.testing.assert_allclose(quaternion_to_matrix(target.quaternion) @ [0, 0, 1],
                                   expected_forward, atol=1e-12)
        if step == 0:
            np.testing.assert_allclose(quaternion_to_matrix(target.quaternion),
                                       quaternion_to_matrix(tool_down.quaternion), atol=1e-12)
    # Releasing and re-engaging retains the measured pose, with no automatic alignment.
    frontend.ingest(packet(5, 0.), .05)
    assert frontend.tick(.05) == {}
    frontend.update_fk('arm', target, .06)
    frontend.ingest(replace(packet(6, 1.), **{controller: hand}), .06)
    np.testing.assert_allclose(quaternion_to_matrix(frontend.tick(.06)['arm'].quaternion),
                               quaternion_to_matrix(target.quaternion), atol=1e-12)


@pytest.mark.parametrize('controller', ['left', 'right'])
@pytest.mark.parametrize('angle', [-math.pi / 3, math.pi / 3])
def test_openarmx_forward_trigger_axis_roll_follows_hand_side(controller, angle):
    from hc_teleop_recv.pose_math import quaternion_multiply
    doc = document()
    doc['channels'][0].update(controller=controller, clutch_controller='right',
        axis_mapping=[[0, 0, -1], [1, 0, 0], [0, -1, 0]], position_axis_signs=[-1, -1, -1])
    frontend = Frontend(parse_config(doc))
    tool_down = Pose(FK.position, tuple(quaternion_from_axis_angle([0, 1, 0], math.pi)))
    hand_down = quaternion_from_axis_angle([1, 0, 0], math.pi / 2)
    frontend.update_fk('arm', tool_down, 0.)
    frontend.ingest(replace(packet(0, 1.), **{controller: Pose(IDENTITY.position, tuple(hand_down))}), 0.)
    np.testing.assert_allclose(quaternion_to_matrix(frontend.tick(0.)['arm'].quaternion),
                               quaternion_to_matrix(tool_down.quaternion), atol=1e-12)
    rolled = quaternion_multiply(quaternion_from_axis_angle([0, 0, 1], angle), hand_down)
    # Trigger (+Y local) remains forward (+Z VR); the downward pointer rolls
    # right (+X VR). Robot +Y is left, so its pointer must roll toward -Y.
    np.testing.assert_allclose(quaternion_to_matrix(rolled) @ [0, 1, 0], [0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(quaternion_to_matrix(rolled) @ [0, 0, 1],
                               [math.sin(angle), -math.cos(angle), 0], atol=1e-12)
    frontend.ingest(replace(packet(1, 1.), **{controller: Pose(IDENTITY.position, tuple(rolled))}), .01)
    target = frontend.tick(.01)['arm']
    assert target.position == tool_down.position
    np.testing.assert_allclose(quaternion_to_matrix(target.quaternion) @ [0, 0, 1],
                               [0, -math.sin(angle), -math.cos(angle)], atol=1e-12)


def test_openarmx_mapping_preserves_translation_and_converts_all_rotation_axes_consistently():
    old = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    new = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]])
    np.testing.assert_array_equal(np.diag([-1, 1, 1]) @ old, np.diag([-1, -1, -1]) @ new)
    assert np.linalg.det(new) == pytest.approx(1.)
    # Unity right/up/forward to robot forward/left/up is a reflection B.
    # C=-B is a proper rotation and C*R*C.T == B*R*B.T for every rotation.
    basis = np.array([[0, 0, 1], [-1, 0, 0], [0, 1, 0]])
    for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 2, 3]):
        hand_delta = quaternion_to_matrix(quaternion_from_axis_angle(axis, .4))
        robot_delta = new @ hand_delta @ new.T
        np.testing.assert_allclose(robot_delta @ basis, basis @ hand_delta, atol=1e-12)
    # Turning the forward-pointing hand right must turn the tool toward -Y.
    yaw = quaternion_to_matrix(quaternion_from_axis_angle([0, 1, 0], .4))
    np.testing.assert_allclose(new @ yaw @ new.T @ [1, 0, 0],
                               [math.cos(.4), -math.sin(.4), 0], atol=1e-12)


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
    {"flags": 8}, {"vr_timestamp": float("nan")}, {"right": Pose((float("inf"), 0, 0), IDENTITY.quaternion)},
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
    ("position_axis_signs", [0, 1, 1]),
    ("position_axis_signs", [-1, 1]),
    ("position_axis_signs", [float("nan"), 1, 1]),
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


@pytest.mark.parametrize('position', [(0., 0., .1), (.1, 0., 0.), (0., .1, 0.), (.1, .1, .1)])
def test_position_axis_signs_reverse_only_base_x_and_preserve_orientation(position):
    doc = document()
    original = bound_frontend(parse_config(doc))
    doc['channels'][0]['position_axis_signs'] = [-1, 1, 1]
    corrected = bound_frontend(parse_config(doc))
    sample = packet(2, 1., position, quat=tuple(quaternion_from_axis_angle([0., 1., 0.], .2)))
    for frontend in (original, corrected):
        frontend.ingest(sample, .02)
    before = original.tick(.02)['arm']
    after = corrected.tick(.02)['arm']
    expected = np.asarray(FK.position) + (np.asarray(before.position)-FK.position)*[-1, 1, 1]
    assert after.position == pytest.approx(expected)
    assert after.quaternion == pytest.approx(before.quaternion)


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


@pytest.mark.parametrize('device,flags', [('head', 7), ('left', 7), ('right', 7), ('left', 5)])
def test_world_origin_offset_does_not_discard_controller_inputs(device, flags):
    inputs = ControllerInput(held_mask=12, trigger=.75, grip=.9)
    sample = replace(packet(flags=flags), left_input=inputs, right_input=inputs,
                     **{device: Pose((-4.182662, .494823, 6.897388), IDENTITY.quaternion)})
    decoded = decode_pose_packet(wire(sample))
    assert decoded.left_input.held_mask == 12
    assert decoded.right_input.held_mask == 12
    assert decoded.right_input.trigger == pytest.approx(.75)
    assert decoded.right_input.grip == pytest.approx(.9)
    assert decode_vrdata(sample.as_dict()) == sample


def test_world_origin_translation_preserves_clutch_motion_and_displacement_limit():
    def run(origin):
        frontend = Frontend(parse_config(document()))
        frontend.update_fk('arm', FK, 0.)
        for seq, grip, delta in [(0, 0., 0.), (1, 1., 0.), (2, 1., -.1), (3, 1., -100.)]:
            position = (origin[0], origin[1], origin[2] + delta)
            frame = replace(packet(seq, grip, position), head=Pose(origin, IDENTITY.quaternion))
            frontend.ingest(decode_pose_packet(wire(frame)), seq * .01)
            result = frontend.tick(seq * .01)
            if seq == 1:
                assert result['arm'] == FK
            elif seq >= 2:
                yield result['arm']
    ordinary = list(run((0., 0., 0.)))
    shifted = list(run((-4.182662, .494823, 6.897388)))
    for a, b in zip(ordinary, shifted):
        np.testing.assert_allclose(a.position, b.position, atol=1e-6)
    bound = parse_config(document()).channels[0].max_displacement
    assert np.linalg.norm(np.asarray(shifted[-1].position) - FK.position) <= bound + 1e-6


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_world_positions_are_still_rejected(invalid):
    sample = replace(packet(), head=Pose((invalid, 0., 0.), IDENTITY.quaternion))
    with pytest.raises(PacketError):
        decode_pose_packet(wire(sample))
    with pytest.raises(PacketError):
        decode_vrdata(sample.as_dict())


def test_relative_displacement_overflow_stops_output_instead_of_publishing_nan():
    frontend = Frontend(parse_config(document()))
    frontend.update_fk('arm', FK, 0.)
    frontend.ingest(packet(0, 1., (-1e308, 0., 0.)), 0.)
    frontend.ingest(packet(1, 1., (1e308, 0., 0.)), .01)
    assert frontend.tick(.01) == {}
    assert 'invalid pose mapping' in frontend.channels['arm'].reason


def test_diagnostic_summary_is_bounded_under_flood_and_flushes_after_silence():
    from hc_teleop_recv.log_summary import LogSummary
    summary = LogSummary(('input_rejected',))
    summary.record('input_rejected', 'first')
    assert len(summary.poll(0.)) == 1
    for i in range(10000):
        summary.record('input_rejected', f'packet {i}: ' + 'x'*2000)
        assert summary.poll(i / 1000.) == []
    assert len(summary.buckets) == 1
    bucket = summary.buckets['input_rejected']
    assert len(bucket.latest) == 1024
    assert bucket.pending == 10000
    messages = summary.poll(30.)
    assert len(messages) == 1 and 'count_since_last_log=10000' in messages[0][1]
    assert 'packet 9999' in messages[0][1]
    assert summary.poll(60.) == []
