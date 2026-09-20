from dataclasses import replace

from hc_teleop_recv.config import parse_config
from hc_teleop_recv.frontend import Frontend
from hc_teleop_recv.vr_status import control_status
from test_frontend import document, packet, FK


def test_enable_state_is_not_inferred_from_connection_or_a_button():
    doc = document()
    doc['control']['enabled_on_start'] = False
    frontend = Frontend(parse_config(doc))
    frontend.ingest(packet(), 1.)
    status = control_status(frontend, 1.)
    assert status['input_fresh'] and status['state'] == 'disabled'
    assert not status['enabled']
    frontend.inhibited = True
    assert control_status(frontend, 1.)['state'] == 'motion_active'
    frontend.set_enabled(True)
    assert not control_status(frontend, 1.)['enabled']


def test_ready_active_and_stale_feedback_are_distinct():
    frontend = Frontend(parse_config(document()))
    frontend.ingest(packet(), 1.)
    assert control_status(frontend, 1.)['state'] == 'feedback_timeout'
    frontend.update_fk('arm', FK, 1.)
    assert control_status(frontend, 1.)['state'] == 'ready'
    frontend.ingest(packet(1, grip=1.), 1.01)
    frontend.tick(1.01)
    assert control_status(frontend, 1.01)['state'] == 'active'
    assert control_status(frontend, 1.01, input_ready=False)['state'] == 'waiting'
    assert control_status(frontend, 2.)['state'] == 'input_timeout'
    frontend.ingest(packet(2, grip=1.), 2.)
    assert control_status(frontend, 2.)['state'] == 'feedback_timeout'
    frontend.update_fk('arm', FK, 2.)
    frontend.ingest(packet(3, grip=1., flags=3), 2.01)
    assert control_status(frontend, 2.01)['state'] == 'tracking_lost'


def test_one_arm_blocked_is_not_reported_as_both_arms_active():
    doc = document()
    doc['channels'].append(dict(doc['channels'][0], id='left', controller='left',
                               fk_pose_topic='/left/fk', target_pose_topic='/left/target'))
    frontend = Frontend(parse_config(doc))
    frontend.ingest(packet(), 1.)
    frontend.update_fk('arm', FK, 1.)
    status = control_status(frontend, 1.)
    assert status['state'] == 'partial'
    assert status['channels']['left']['state'] == 'feedback_timeout'
    assert status['channels']['arm']['state'] == 'ready'
    frontend.config = replace(frontend.config, arm_control_enabled=False)
    assert control_status(frontend, 1.)['state'] == 'enabled'
