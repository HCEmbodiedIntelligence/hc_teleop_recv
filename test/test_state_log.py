from dataclasses import replace

from hc_teleop_recv.config import parse_config
from hc_teleop_recv.frontend import Frontend
from hc_teleop_recv.protocol import Pose
from hc_teleop_recv.state_log import PoseActivity, StateLog, TeleopStateLog
from test_frontend import document, packet, FK


def test_state_changes_emit_once_and_rapid_flapping_is_bounded_but_visible():
    log = StateLog(['udp'])
    log.observe('udp', 'receiving', 'packets=1')
    assert 'unknown -> receiving' in log.poll(0)[0]
    for i in range(1, 100):
        log.observe('udp', 'stopped' if i % 2 else 'receiving', f'packets={i}')
        assert log.poll(i / 100) == []
    messages = log.poll(1)
    assert len(messages) == 1 and 'changes=99' in messages[0]
    for i in range(2, 100):
        log.observe('udp', 'stopped', f'packets={i}')
        assert log.poll(i) == []


def test_short_start_stop_cycle_is_retained_even_when_final_state_is_unchanged():
    log = StateLog(['udp'])
    log.observe('udp', 'stopped')
    log.poll(0)
    log.observe('udp', 'receiving')
    assert log.poll(.1) == []
    log.observe('udp', 'stopped')
    assert 'stopped -> receiving -> stopped' in log.poll(1)[0]
    assert log.poll(2) == []


def test_pose_activity_ignores_jitter_and_quaternion_sign_detects_slow_motion_and_rotation():
    activity = PoseActivity()
    activity.update(FK, 0)
    for i in range(1, 50):
        pose = replace(FK, position=(FK.position[0] + (-1)**i * .0002, *FK.position[1:]),
                       quaternion=tuple(-x for x in FK.quaternion))
        activity.update(pose, i / 100)
        assert activity.state(i / 100) == 'stationary'
    for i in range(1, 5):
        activity.update(replace(FK, position=(FK.position[0] + i * .001, *FK.position[1:])), .5+i/100)
    assert activity.state(.55) == 'moving'
    assert activity.state(1.1) == 'stationary'
    activity.reset()
    # A resumed input must establish a new baseline, not count its discontinuity as motion.
    activity.update(Pose((1., 2., 3.), FK.quaternion), 2)
    assert activity.state(2) == 'stationary'
    activity.update(Pose((1., 2., 3.), (0., 0., .05, .9987492178)), 2.1)
    assert activity.state(2.1) == 'moving'


def test_transport_acceptance_output_and_timeout_are_distinct_and_no_periodic_spam():
    cfg = parse_config(document())
    log, frontend = TeleopStateLog(cfg), Frontend(cfg)

    def poll(now, accepted=0, rejected=0):
        return log.poll(now, frontend, frontend.tick(now), {}, accepted, rejected)

    assert any('input.transport: unknown -> stopped' in m for m in poll(0))
    # Bytes arrive but are rejected: never claim valid hand input or target publication.
    log.transport(1, ('127.0.0.1', 5005))
    messages = poll(1, rejected=1)
    assert any('input.transport: stopped -> receiving' in m for m in messages)
    assert not any('input.accepted' in m or 'publishing' in m for m in messages)
    frontend.update_fk('arm', FK, 2)
    p = packet(grip=1)
    log.transport(2, ('127.0.0.1', 5005))
    log.accepted(p, 2, True)
    frontend.ingest(p, 2)
    messages = poll(2, accepted=1, rejected=1)
    assert any('input.accepted: stopped -> receiving' in m for m in messages)
    assert any('arm.arm.output:' in m and '-> publishing' in m for m in messages)
    # Hold the exact same pose and a live stream for several seconds: no repeated logs.
    for i in range(1, 301):
        now = 2+i/100
        frontend.update_fk('arm', FK, now)
        frontend.ingest(p, now)
        log.transport(now, 'peer')
        log.accepted(p, now, False)
        assert poll(now, accepted=i+1, rejected=1) == []
    messages = poll(6, accepted=301, rejected=1)
    assert any('input.transport: receiving -> stopped' in m for m in messages)
    assert any('input.accepted: receiving -> stopped' in m for m in messages)
    assert any('publishing -> waiting: controller input timed out' in m for m in messages)
    assert any('controller.right.pose: stationary -> input_stopped' in m for m in messages)
    assert poll(60) == []


def test_grip_tracking_fk_failure_and_recovery_have_actionable_reasons():
    cfg = parse_config(document())
    log, frontend = TeleopStateLog(cfg), Frontend(cfg)

    def step(now, p, fk=True):
        if fk:
            frontend.update_fk('arm', FK, now)
        log.accepted(p, now, now-frontend.input_at > cfg.input_timeout)
        frontend.ingest(p, now)
        return log.poll(now, frontend, frontend.tick(now), {}, 1, 0)

    messages = step(0, packet(grip=.1))
    assert any('Grip released' in m and 'grip=0.100' in m and 'threshold=0.55' in m for m in messages)
    messages = step(1, packet(grip=1), fk=False)
    assert any('fresh measured FK required' in m for m in messages)
    messages = step(2, packet(grip=1))
    assert any('-> publishing' in m for m in messages)
    messages = step(3, packet(grip=1, flags=3))
    assert any('-> tracking_lost' in m for m in messages)
    assert any('controller tracking/input unavailable' in m for m in messages)
    messages = step(4, packet(grip=1))
    assert any('-> publishing' in m for m in messages)
