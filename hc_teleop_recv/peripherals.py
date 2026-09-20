"""Vendor-neutral joystick/base and measured-feedback gripper target generation."""
import math
from dataclasses import dataclass

from .protocol import BUTTON_NAMES

BUTTON_MASKS = {name: mask for mask, name in BUTTON_NAMES.items()}


def held(packet, controller, button):
    return bool(getattr(packet, controller+'_input').held_mask & BUTTON_MASKS[button])


def axis_value(packet, controller, axis, deadband):
    if axis == 'none':
        return 0.
    group, component = axis.split('_')
    value = getattr(getattr(packet, controller+'_input'), group+'_axis')[0 if component == 'x' else 1]
    return math.copysign((abs(value)-deadband)/(1-deadband), value) if abs(value) > deadband else 0.


def approach(current, target, delta):
    return current + max(-delta, min(delta, target-current))


@dataclass
class GripperState:
    feedback: float | None = None
    feedback_at: float = float('-inf')
    target: float | None = None
    state: str = 'disabled'
    reason: str = 'disabled'
    last_publish: float = float('-inf')
    last_input: float | None = None


class Peripherals:
    def __init__(self, config):
        self.config = config
        self.grippers = {g.id: GripperState() for g in config.grippers}
        self.base_armed = False
        self.base_velocity = (0.,0.,0.)
        self.base_status = {'state':'disabled','reason':'未配置或未启用'}
        self.last_tick = None
        self.last_base_publish = float('-inf')
        self.was_enabled = False

    def reset(self):
        self.base_armed = False
        for state in self.grippers.values():
            state.last_input = None

    def feedback(self, ident, position, now):
        cfg = next(g for g in self.config.grippers if g.id == ident)
        if not math.isfinite(position) or not min(cfg.open_position,cfg.closed_position)-1e-6 <= position <= max(cfg.open_position,cfg.closed_position)+1e-6:
            self.grippers[ident].feedback_at = float('-inf')
            return False
        self.grippers[ident].feedback = float(position)
        self.grippers[ident].feedback_at = now
        return True

    def tick(self, frontend, now, ready=None, input_ready=True):
        ready = ready or {}
        dt = min(.1, max(0, now-self.last_tick)) if self.last_tick is not None else 0.
        self.last_tick = now
        if frontend.enabled != self.was_enabled:
            self.reset()
        self.was_enabled = frontend.enabled
        packet = frontend.packet
        fresh = input_ready and packet is not None and packet.protocol_version == 2 and now-frontend.input_at <= self.config.input_timeout
        output = {'chassis':None, 'grippers':{}, 'stop_grippers':[]}
        cfg = self.config.chassis
        if cfg and cfg.enabled:
            reason = ''
            active = False
            if not frontend.enabled:
                reason = '接收端未启用'
            elif not ready.get('chassis',True):
                reason = '等待底盘控制接口'
            elif not fresh or now-frontend.input_at > cfg.command_timeout or not packet.tracked(cfg.controller):
                reason = '手柄输入超时或跟踪丢失'
            if reason:
                self.base_armed = False
            elif not held(packet,cfg.controller,cfg.enable_button):
                self.base_armed = True
                reason = '使能按键已松开'
            elif not self.base_armed:
                reason = '请先松开使能按键，再按下'
            else:
                active = True
            previous = self.base_velocity
            if active:
                x = axis_value(packet,cfg.controller,cfg.forward_axis,cfg.deadband)*(-1 if cfg.invert_forward else 1)
                y = axis_value(packet,cfg.controller,cfg.lateral_axis,cfg.deadband)*(-1 if cfg.invert_lateral else 1)
                z = axis_value(packet,cfg.controller,cfg.turn_axis,cfg.deadband)*(-1 if cfg.invert_turn else 1)
                norm = max(1.,math.hypot(x,y))
                targets = (x/norm*cfg.max_forward_speed,y/norm*cfg.max_lateral_speed,z*cfg.max_turn_speed)
                self.base_velocity = tuple(approach(a,b,limit*dt) for a,b,limit in zip(previous,targets,
                    (cfg.linear_acceleration,cfg.linear_acceleration,cfg.angular_acceleration)))
            else:
                self.base_velocity = (0.,0.,0.)
            self.base_status = {'state':'active' if active else 'stopped','reason':reason or '摇杆控制中',
                'command_topic':cfg.command_topic,'velocity':list(self.base_velocity),'interface_ready':ready.get('chassis',True)}
            if now-self.last_base_publish >= 1/cfg.rate_hz or (previous != (0.,0.,0.) and not active):
                output['chassis'] = self.base_velocity
                self.last_base_publish = now
        for cfg in self.config.grippers:
            state = self.grippers[cfg.id]
            previous = state.state
            reason = ''
            if not cfg.enabled or not frontend.enabled:
                reason = '夹爪或接收端未启用'
            elif not ready.get(cfg.id,True):
                reason = '等待夹爪控制接口'
            elif not fresh or not packet.tracked(cfg.controller):
                reason = '手柄输入超时或跟踪丢失'
            elif state.feedback is None or now-state.feedback_at > cfg.feedback_timeout:
                reason = '等待新鲜且位于开合范围内的夹爪反馈'
            if reason:
                state.state = 'disabled' if not cfg.enabled else 'waiting'
            elif cfg.require_enable_button and not held(packet,cfg.controller,cfg.enable_button):
                state.state = 'ready'
                reason = '使能按键已松开'
            else:
                state.state = 'active'
            state.reason = reason or '夹爪控制中'
            if state.state != 'active':
                state.target = state.feedback
                state.last_input = None
                if previous == 'active':
                    output['stop_grippers'].append(cfg.id)
                continue
            value = getattr(getattr(packet,cfg.controller+'_input'),cfg.input_axis)
            if state.last_input is None or abs(value-state.last_input) >= cfg.deadband:
                state.last_input = value
            target = cfg.open_position+(cfg.closed_position-cfg.open_position)*state.last_input
            state.target = approach(state.target if state.target is not None else state.feedback,target,cfg.max_speed*dt)
            if now-state.last_publish >= 1/cfg.rate_hz:
                output['grippers'][cfg.id] = state.target
                state.last_publish = now
        return output

    def status(self):
        return {'chassis':self.base_status, 'grippers':{cfg.id:{
            'enabled':cfg.enabled,'state':self.grippers[cfg.id].state,'reason':self.grippers[cfg.id].reason,
            'feedback':self.grippers[cfg.id].feedback,'target':self.grippers[cfg.id].target,
            'command_topic':cfg.command_topic,'command_type':cfg.command_type,'position_unit':cfg.position_unit}
            for cfg in self.config.grippers}}
