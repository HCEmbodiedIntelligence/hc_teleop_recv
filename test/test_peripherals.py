from dataclasses import replace

import pytest

from hc_teleop_recv.config import parse_config, ConfigError
from hc_teleop_recv.frontend import Frontend
from hc_teleop_recv.peripherals import Peripherals, BUTTON_MASKS
from hc_teleop_recv.protocol import ControllerInput, PosePacket, Pose

POSE=Pose((0.,0.,0.),(0.,0.,0.,1.))


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


def test_gripper_measured_seed_bounded_speed_and_fresh_feedback():
    cfg=config()
    p,f=Peripherals(cfg),Frontend(cfg)
    feed(f,0,False)
    p.tick(f,0)
    feed(f,.05,True)
    assert not p.tick(f,.05)['grippers']
    assert p.feedback('tool',.03,.06)
    feed(f,.07,True)
    assert not p.tick(f,.07)['grippers']
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


def test_conflicting_outputs_and_wrong_gripper_action_units_rejected():
    with pytest.raises(ConfigError):
        parse_config({'schema_version':1,'channels':[],'chassis':{'command_topic':'/hc_teleop/joint_cmd'}})
    with pytest.raises(ConfigError):
        parse_config({'schema_version':1,'channels':[],'grippers':[{'id':'a','command_type':'gripper_action','position_unit':'rad'}]})
    with pytest.raises(ConfigError):
        parse_config({'schema_version':1,'channels':[],'grippers':[{'id':'a'},{'id':'b'}]})
