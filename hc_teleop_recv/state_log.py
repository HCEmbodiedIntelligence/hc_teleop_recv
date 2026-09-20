"""Bounded state-change diagnostics, independent of control decisions and ROS."""
from dataclasses import dataclass, field
import math


@dataclass
class _Transition:
    state: object = None
    logged: object = None
    detail: str = ''
    changes: int = 0
    last_emit: float = float('-inf')
    path: list = field(default_factory=list)


class StateLog:
    """One bounded slot per configured source; coalesce rapid changes, never repeat steady state."""
    def __init__(self, keys, interval=1.0):
        self.interval = interval
        self.entries = {key: _Transition() for key in keys}

    def observe(self, key, state, detail=''):
        entry = self.entries[key]
        if state != entry.state:
            entry.changes += 1
            entry.state = state
            if len(entry.path) < 8:
                entry.path.append(state)
            else:
                entry.path[-1] = state
        entry.detail = detail[:1024]

    def poll(self, now):
        messages = []
        for key, entry in self.entries.items():
            if entry.changes and now - entry.last_emit >= self.interval:
                path = list(entry.path)
                if entry.changes > len(path):
                    path.insert(-1, f'[{entry.changes-len(path)} changes omitted]')
                messages.append(f'{key}: {entry.logged or "unknown"} -> {" -> ".join(path)}; '
                                f'changes={entry.changes}; {entry.detail}')
                entry.logged, entry.changes, entry.last_emit = entry.state, 0, now
                entry.path.clear()
        return messages


class PoseActivity:
    """Use accumulated pose change, quaternion sign invariance and a quiet period for noise."""
    def __init__(self, position_threshold=.002, rotation_threshold=.01, quiet_period=.5):
        self.position_threshold = position_threshold
        self.rotation_threshold = rotation_threshold
        self.quiet_period = quiet_period
        self.anchor = None
        self.last_change = float('-inf')

    def reset(self):
        self.anchor = None
        self.last_change = float('-inf')

    def update(self, pose, now):
        if self.anchor is not None:
            distance = math.dist(pose.position, self.anchor.position)
            a, b = pose.quaternion, self.anchor.quaternion
            norm = math.sqrt(sum(x*x for x in a) * sum(x*x for x in b))
            dot = min(1., abs(sum(x*y for x, y in zip(a, b))) / norm)
            angle = 2 * math.acos(dot)
            if distance < self.position_threshold and angle < self.rotation_threshold:
                return
            self.last_change = now
        self.anchor = pose

    def state(self, now):
        return 'moving' if now - self.last_change < self.quiet_period else 'stationary'


class TeleopStateLog:
    def __init__(self, config):
        self.config = config
        self.controllers = sorted({c.controller for c in config.channels} |
                                  {c.clutch_controller for c in config.channels} |
                                  {g.controller for g in config.grippers})
        keys = ['input.transport', 'input.accepted', 'control']
        keys += [f'controller.{hand}.pose' for hand in self.controllers]
        keys += [f'arm.{c.id}.{kind}' for c in config.channels for kind in ('fk', 'output')]
        keys += [f'gripper.{g.id}' for g in config.grippers]
        self.log = StateLog(keys)
        self.transport_at = float('-inf')
        self.transport_count = 0
        self.peer = ''
        self.poses = {hand: PoseActivity() for hand in self.controllers}
        self.fk_errors = {c.id: '' for c in config.channels}
        self.fk_error_kinds = {c.id: '' for c in config.channels}

    def transport(self, now, peer):
        self.transport_at = now
        self.transport_count += 1
        self.peer = str(peer)[:128]

    def accepted(self, packet, now, restarted):
        for hand, activity in self.poses.items():
            if restarted or not packet.tracked(hand):
                activity.reset()
            if packet.tracked(hand):
                activity.update(getattr(packet, hand), now)

    def poll(self, now, frontend, targets, peripherals, accepted_count, rejected_count):
        cfg, packet = self.config, frontend.packet
        fresh = packet is not None and now - frontend.input_at <= cfg.input_timeout
        receiving = now - self.transport_at <= cfg.input_timeout
        self.log.observe('input.transport', 'receiving' if receiving else 'stopped',
                         f'mode={cfg.input_mode}; peer={self.peer}; packets={self.transport_count}; '
                         f'timeout_s={cfg.input_timeout:g}')
        self.log.observe('input.accepted', 'receiving' if fresh else 'stopped',
                         f'accepted={accepted_count}; rejected={rejected_count}; '
                         f'sequence={packet.sequence if packet else "none"}')
        control = 'motion_active' if frontend.inhibited else (
            'enabled' if frontend.enabled else 'disabled')
        self.log.observe('control', control, f'arm_control_enabled={cfg.arm_control_enabled}')
        for hand, activity in self.poses.items():
            state = 'input_stopped' if not fresh else (
                activity.state(now) if packet.tracked(hand) else 'tracking_lost')
            self.log.observe(f'controller.{hand}.pose', state)
        for ident, channel in frontend.channels.items():
            c = channel.config
            fk_fresh = channel.fk is not None and now - channel.fk_at <= cfg.fk_timeout
            fk_reason = self.fk_error_kinds[ident] or (
                'timeout' if channel.fk is not None else 'waiting_for_first_feedback')
            fk_state = 'receiving' if fk_fresh else f'unavailable: {fk_reason}'
            self.log.observe(f'arm.{ident}.fk', fk_state,
                             f'topic={c.fk_pose_topic}; timeout_s={cfg.fk_timeout:g}; '
                             f'last_rejection={self.fk_errors[ident] or "none"}')
            # Derive live input timeout even if an inactive frontend retained its last reason.
            reason = channel.reason
            if frontend.enabled and cfg.arm_control_enabled and not fresh:
                reason = 'controller input timed out'
            state = 'publishing' if ident in targets else f'{channel.state}: {reason}'
            grip = getattr(packet, c.clutch_controller + '_input').grip if packet else 0.
            self.log.observe(f'arm.{ident}.output', state,
                             f'topic={c.target_pose_topic}; pose_controller={c.controller}; '
                             f'clutch_controller={c.clutch_controller}; grip={grip:.3f}; '
                             f'threshold={c.clutch_threshold:g}; fk_fresh={fk_fresh}')
        for ident, state in peripherals.get('grippers', {}).items():
            self.log.observe(f'gripper.{ident}', f'{state["state"]}: {state["reason"]}')
        return self.log.poll(now)
