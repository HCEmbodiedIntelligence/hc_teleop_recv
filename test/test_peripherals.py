from dataclasses import replace

import pytest

from hc_teleop_recv.config import parse_config, ConfigError
from hc_teleop_recv.frontend import Frontend
from hc_teleop_recv.peripherals import Peripherals, BUTTON_MASKS
from hc_teleop_recv.protocol import ControllerInput, PosePacket, Pose

POSE=Pose((0.,0.,0.),(0.,0.,0.,1.))


def test_gripper_trigger_without_grip_still_obeys_pause_and_feedback_timeout():
    cfg = parse_config({'schema_version': 1, 'channels': [],
        'control': {'enabled_on_start': True},
        'grippers': [{'id': 'tool', 'enabled': True, 'require_enable_button': False}]})
    p, f = Peripherals(cfg), Frontend(cfg)
    p.feedback('tool', .03, 0.)
    feed(f, 0., False)
    assert 'tool' in p.tick(f, 0.)['grippers']
    feed(f, .05, False)
    assert p.tick(f, .05)['grippers']['tool'] < .03
    f.set_enabled(False)
    assert not p.tick(f, .06)['grippers']
    f.set_enabled(True)
    feed(f, 1., False)
    assert not p.tick(f, 1.)['grippers']


def config(enabled=True):
    return parse_config({'schema_version':1,'control':{'enabled_on_start':True},'channels':[],
        'chassis':{'enabled':enabled},'grippers':[{'id':'tool','enabled':enabled}]})


def feed(frontend,now,held=False,trigger=1.,ready=True):
    packet=PosePacket(2,round(now*10000),now+1,7,POSE,POSE,POSE,
        ControllerInput(held_mask=BUTTON_MASKS['primary_axis_click'] if held else 0,primary_axis=(.5,1.)),
        ControllerInput(held_mask=BUTTON_MASKS['grip_button'] if held else 0,trigger=trigger))
    frontend.ingest(packet,now)


def test_disabled_configuration_never_publishes_and_can_be_peripheral_only():
    cfg=config(False)
    p=Peripherals(cfg)
    f=Frontend(cfg)
    feed(f,0,True)
    result=p.tick(f,0)
    assert result['chassis'] is None and result['grippers']=={}
    assert not cfg.channels


def test_chassis_deadman_limits_timeout_and_interface_recovery():
    cfg=config()
    p,f=Peripherals(cfg),Frontend(cfg)
    feed(f,0,True)
    assert p.tick(f,0)['chassis']==(0,0,0)
    feed(f,.01,False)
    p.tick(f,.01)
    feed(f,.05,True)
    output=p.tick(f,.05)['chassis']
    assert 0<output[0]<=cfg.chassis.linear_acceleration*.04+1e-12
    assert abs(output[2])<=cfg.chassis.angular_acceleration*.04+1e-12
    assert p.tick(f,.4)['chassis']==(0,0,0)
    feed(f,.41,True)
    p.tick(f,.41)
    assert p.base_velocity==(0,0,0)
    feed(f,.42,False)
    p.tick(f,.42,{'chassis':False})
    feed(f,.43,True)
    p.tick(f,.43,{'chassis':True})
    assert p.base_velocity==(0,0,0)  # Release while disconnected cannot rearm.
    feed(f,.44,False)
    p.tick(f,.44,{'chassis':True})
    feed(f,.48,True)
    assert p.tick(f,.48,{'chassis':True})['chassis'][0]>0


def test_backlog_stops_peripherals_even_while_fresh_packets_are_ingested():
    cfg = config()
    p, f = Peripherals(cfg), Frontend(cfg)
    p.feedback('tool', .03, 0.)
    feed(f, 0., False)
    p.tick(f, 0.)
    feed(f, .05, True)
    active = p.tick(f, .05)
    assert active['chassis'][0] > 0 and active['grippers']
    feed(f, .06, True)
    stopped = p.tick(f, .06, input_ready=False)
    assert stopped['chassis'] == (0., 0., 0.)
    assert stopped['grippers'] == {} and stopped['stop_grippers'] == ['tool']


def test_gripper_measured_seed_bounded_speed_and_fresh_feedback():
    cfg=config()
    p,f=Peripherals(cfg),Frontend(cfg)
    feed(f,0,False)
    p.tick(f,0)
    feed(f,.05,True)
    assert not p.tick(f,.05)['grippers']
    assert p.feedback('tool',.03,.06)
    feed(f,.07,True)
    assert p.tick(f,.07)['grippers']['tool']==pytest.approx(.03-cfg.grippers[0].max_speed*.02)
    feed(f,.08,False)
    p.tick(f,.08)
    feed(f,.13,True)
    command=p.tick(f,.13)['grippers']['tool']
    assert command==pytest.approx(.03-cfg.grippers[0].max_speed*.05)
    feed(f,.7,True)
    result=p.tick(f,.7)
    assert result['stop_grippers']==['tool'] and not result['grippers']
    assert not p.feedback('tool',float('nan'),.71)
    assert not p.feedback('tool',.5,.71)


@pytest.mark.parametrize('interruption', ['enable', 'input_timeout', 'tracking', 'feedback', 'interface', 'session'])
def test_gripper_resumes_with_grip_held_without_release(interruption):
    cfg = config()
    p, f = Peripherals(cfg), Frontend(cfg)
    p.feedback('tool', .03, 0.)
    feed(f, 0., True)
    assert 'tool' in p.tick(f, 0.)['grippers']
    feed(f, .1, True)
    ready = {'tool': True}
    if interruption == 'enable':
        f.set_enabled(False)
    elif interruption == 'input_timeout':
        f.input_at = -1.
    elif interruption == 'tracking':
        f.packet = replace(f.packet, flags=3)
    elif interruption == 'feedback':
        p.feedback('tool', float('nan'), .1)
    elif interruption == 'interface':
        ready['tool'] = False
    else:
        p.reset()
    if interruption != 'session':
        assert not p.tick(f, .1, ready)['grippers']
    f.set_enabled(True)
    p.feedback('tool', .025, .2)
    feed(f, .2, True)
    result = p.tick(f, .2, {'tool': True})
    assert 'tool' in result['grippers']
    assert p.grippers['tool'].state == 'active'
    feed(f, .3, False)
    assert not p.tick(f, .3)['grippers']
    assert p.grippers['tool'].state == 'ready'


def test_gripper_and_chassis_work_with_saved_arm_channels_switched_off():
    from test_frontend import document
    doc = document()
    doc['control']['arm_control_enabled'] = False
    doc['chassis'] = {'enabled': True}
    doc['grippers'] = [{'id': 'tool', 'enabled': True}]
    cfg = parse_config(doc)
    p, f = Peripherals(cfg), Frontend(cfg)
    p.feedback('tool', .03, 0.)
    feed(f, 0., False)
    p.tick(f, 0.)
    feed(f, .05, True)
    result = p.tick(f, .05)
    assert result['grippers']['tool'] < .03
    assert result['chassis'][0] > 0
    assert f.tick(.05) == {}
    assert len(f.channels) == 1


def test_conflicting_outputs_and_wrong_gripper_action_units_rejected():
    with pytest.raises(ConfigError):
        parse_config({'schema_version':1,'channels':[],'chassis':{'command_topic':'/hc_teleop/joint_cmd'}})
    with pytest.raises(ConfigError):
        parse_config({'schema_version':1,'channels':[],'grippers':[{'id':'a','command_type':'gripper_action','position_unit':'rad'}]})
    shared = parse_config({'schema_version':1,'channels':[],'grippers':[{'id':'a'},{'id':'b'}]})
    assert shared.grippers[0].command_topic == shared.grippers[1].command_topic
    with pytest.raises(ConfigError, match='joint_state'):
        parse_config({'schema_version':1,'channels':[],'grippers':[
            {'id':'a'}, {'id':'b','command_type':'float64'}]})
