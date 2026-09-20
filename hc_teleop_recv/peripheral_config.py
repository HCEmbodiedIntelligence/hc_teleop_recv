"""Optional chassis and gripper configuration; independent of any vendor driver."""
from dataclasses import dataclass

from .config import ConfigError, boolean, mapping, number, text, topic

BUTTONS = ('grip_button', 'trigger_button', 'primary', 'secondary', 'menu', 'primary_axis_click', 'secondary_axis_click')
AXES = ('none', 'primary_x', 'primary_y', 'secondary_x', 'secondary_y')


def choice(value, choices, label):
    if value not in choices:
        raise ConfigError(f'{label} must be one of {", ".join(choices)}')
    return value


@dataclass(frozen=True)
class ChassisConfig:
    enabled: bool = False
    command_topic: str = '/cmd_vel'
    message_type: str = 'twist'
    frame_id: str = 'base_link'
    controller: str = 'left'
    enable_button: str = 'primary_axis_click'
    forward_axis: str = 'primary_y'
    lateral_axis: str = 'none'
    turn_axis: str = 'primary_x'
    invert_forward: bool = False
    invert_lateral: bool = False
    invert_turn: bool = True
    max_forward_speed: float = .3
    max_lateral_speed: float = .3
    max_turn_speed: float = .6
    linear_acceleration: float = .5
    angular_acceleration: float = 1.
    deadband: float = .12
    rate_hz: float = 30.
    command_timeout: float = .25


@dataclass(frozen=True)
class GripperConfig:
    id: str
    enabled: bool = False
    controller: str = 'right'
    input_axis: str = 'trigger'
    require_enable_button: bool = True
    enable_button: str = 'grip_button'
    command_type: str = 'joint_state'
    command_topic: str = '/gripper/command'
    feedback_type: str = 'joint_state'
    feedback_topic: str = '/joint_states'
    joint_name: str = 'finger_joint'
    position_unit: str = 'm'
    open_position: float = .04
    closed_position: float = 0.
    max_speed: float = .05
    max_effort: float = 10.
    deadband: float = .01
    rate_hz: float = 20.
    feedback_timeout: float = .5


def parse_chassis(document):
    defaults = ChassisConfig().__dict__
    entry = mapping(document, set(), set(defaults), 'chassis')
    values = {**defaults, **entry}
    for key in ('enabled', 'invert_forward', 'invert_lateral', 'invert_turn'):
        values[key] = boolean(values[key], f'chassis.{key}')
    values['command_topic'] = topic(values['command_topic'], 'chassis.command_topic')
    values['frame_id'] = text(values['frame_id'], 'chassis.frame_id')
    choice(values['message_type'], ('twist', 'twist_stamped'), 'chassis.message_type')
    choice(values['controller'], ('left', 'right'), 'chassis.controller')
    choice(values['enable_button'], BUTTONS, 'chassis.enable_button')
    for key in ('forward_axis', 'lateral_axis', 'turn_axis'):
        choice(values[key], AXES, f'chassis.{key}')
    for key, low, high in [('max_forward_speed',0,5), ('max_lateral_speed',0,5), ('max_turn_speed',0,5),
                           ('linear_acceleration',.01,10), ('angular_acceleration',.01,20),
                           ('deadband',0,.95), ('rate_hz',1,100), ('command_timeout',.02,2)]:
        values[key] = number(values[key], f'chassis.{key}', low, high)
    return ChassisConfig(**values)


def parse_grippers(document):
    import re
    if not isinstance(document, list) or len(document) > 16:
        raise ConfigError('grippers must be a list with at most 16 entries')
    result, ids, outputs = [], set(), {}
    for item in document:
        defaults = GripperConfig(id='gripper').__dict__
        item = mapping(item, {'id'}, set(defaults)-{'id'}, 'gripper')
        values = {**defaults, **item}
        ident = values['id']
        if not isinstance(ident, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]{0,63}', ident) or ident in ids:
            raise ConfigError('gripper.id must be unique and use letters, digits or underscores')
        ids.add(ident)
        values['enabled'] = boolean(values['enabled'], 'gripper.enabled')
        values['require_enable_button'] = boolean(values['require_enable_button'], 'gripper.require_enable_button')
        for key in ('command_topic', 'feedback_topic'):
            values[key] = topic(values[key], f'gripper.{key}')
        if values['command_topic'] == values['feedback_topic']:
            raise ConfigError('gripper command endpoints must be separate from feedback')
        values['joint_name'] = text(values['joint_name'], 'gripper.joint_name')
        for key, options in [('controller',('left','right')), ('input_axis',('trigger','grip')),
                             ('enable_button',BUTTONS), ('command_type',('joint_state','float64','gripper_action')),
                             ('feedback_type',('joint_state','float64')), ('position_unit',('m','rad'))]:
            choice(values[key], options, f'gripper.{key}')
        previous_command_type = outputs.get(values['command_topic'])
        if previous_command_type is not None and (
                previous_command_type != 'joint_state' or values['command_type'] != 'joint_state'):
            raise ConfigError(
                'shared gripper command topics are only supported with joint_state messages')
        outputs[values['command_topic']] = values['command_type']
        for key, low, high in [('open_position',-10,10), ('closed_position',-10,10), ('max_speed',.0001,10),
                               ('max_effort',0,1000), ('deadband',0,.95), ('rate_hz',1,100), ('feedback_timeout',.02,5)]:
            values[key] = number(values[key], f'gripper.{key}', low, high)
        if abs(values['open_position']-values['closed_position']) < 1e-9:
            raise ConfigError('gripper open_position and closed_position must differ')
        if values['command_type'] == 'gripper_action' and values['position_unit'] != 'm':
            raise ConfigError('GripperCommand uses metres; select position_unit=m')
        result.append(GripperConfig(**values))
    return tuple(result)


def validate_peripheral_endpoints(chassis, grippers, reserved, feedback_types=None):
    feedback_types = dict(feedback_types or {})
    outputs = {g.command_topic for g in grippers}
    if chassis:
        if chassis.command_topic in outputs:
            raise ConfigError('chassis and gripper command endpoints must differ')
        outputs.add(chassis.command_topic)
    if outputs & set(reserved):
        raise ConfigError('peripheral command endpoints overlap arm, input or status topics')
    for gripper in grippers:
        if gripper.feedback_topic in outputs:
            raise ConfigError('gripper feedback must not overlap command endpoints')
        previous = feedback_types.get(gripper.feedback_topic)
        if previous is not None and previous != gripper.feedback_type:
            raise ConfigError('shared gripper feedback topics must have the same message type')
        feedback_types[gripper.feedback_topic] = gripper.feedback_type
