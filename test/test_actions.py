from dataclasses import replace

import pytest

from hc_teleop_recv.actions import InputActions, parse_actions
from hc_teleop_recv.config import ConfigError, parse_config
from hc_teleop_recv.frontend import Frontend
from hc_teleop_recv.protocol import ControllerInput
from test_frontend import document, packet, FK


def frame(left=0, right=0, lx=0, rx=0):
    return replace(packet(), left_input=ControllerInput(held_mask=left, primary_axis=(lx, 0)),
                   right_input=ControllerInput(held_mask=right, primary_axis=(rx, 0)))


def test_record_edges_and_different_combination_gestures():
    actions = InputActions(parse_actions({'recording_buttons_enabled': True, 'mark_gesture_enabled': True,
        'home_pose_id': 'right_home', 'home_gesture_enabled': True}))
    assert actions.update(frame()) == []
    assert actions.update(frame(left=1)) == ['record_start']
    assert actions.update(frame(left=1)) == []
    assert actions.update(frame(left=2)) == ['record_stop']
    assert actions.update(frame(left=32)) == []
    assert actions.update(frame(left=32, right=32)) == ['record_mark']
    assert actions.update(frame(left=32, right=32)) == []
    assert actions.update(frame(lx=-1, rx=1)) == ['home']
    assert actions.update(frame(lx=-1, rx=1)) == []
    actions.update(frame())
    assert actions.update(frame(lx=-1, rx=1)) == ['home']


def test_reconnection_does_not_repeat_motion_or_recording():
    actions = InputActions(parse_actions({'home_pose_id': 'right_home', 'home_gesture_enabled': True,
                                         'recording_buttons_enabled': True}))
    assert actions.update(frame(left=1, lx=-1, rx=1), restarted=True) == []
    assert actions.update(frame(left=1, lx=-1, rx=1)) == []
    actions.update(frame())
    assert actions.update(frame(left=1)) == ['record_start']


def test_short_press_released_before_packet_keeps_explicit_edge():
    actions = InputActions(parse_actions({'recording_buttons_enabled': True}))
    actions.update(frame())
    pulse = replace(frame(), left_input=ControllerInput(pressed_mask=1, released_mask=1))
    assert actions.update(pulse) == ['record_start']


def test_pose_selection_and_button_conflicts():
    with pytest.raises(ConfigError):
        parse_actions({'home_gesture_enabled': True})
    with pytest.raises(ConfigError):
        parse_actions({'pause_button': 'right_secondary', 'reset_reference_button': 'right_secondary'})


def test_a_cannot_reenable_during_motion_and_held_grip_rebinds_afterward():
    frontend = Frontend(parse_config(document()))
    frontend.inhibited = True
    frontend.set_enabled(False)
    a = replace(packet(grip=1), right_input=ControllerInput(held_mask=1, grip=1))
    frontend.update_fk('arm', FK, 1)
    frontend.ingest(a, 1)
    assert not frontend.enabled and not frontend.tick(1)
    frontend.inhibited = False
    frontend.set_enabled(True)
    frontend.ingest(packet(grip=1), 1.01)
    assert frontend.tick(1.01)['arm'] == FK
