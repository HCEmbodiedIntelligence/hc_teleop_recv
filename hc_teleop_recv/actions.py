"""Bounded input-edge actions. No ROS, motion execution or file IO."""
from dataclasses import dataclass

from .config import boolean, mapping, number, ConfigError


@dataclass(frozen=True)
class ActionConfig:
    home_pose_id: str = ''
    home_gesture_enabled: bool = False
    home_threshold: float = .95
    home_release_threshold: float = .35
    recording_buttons_enabled: bool = False
    mark_gesture_enabled: bool = False
    pause_button: str = 'none'
    reset_reference_button: str = 'none'
    posture_pose_id: str = ''
    posture_weight: float = .001


BUTTONS = {'none': None, 'right_secondary': ('right', 2),
           'left_menu': ('left', 16), 'right_menu': ('right', 16)}


def parse_actions(raw):
    defaults = ActionConfig().__dict__
    values = {**defaults, **mapping(raw, set(), set(defaults), 'actions')}
    import re
    for key in ('home_pose_id', 'posture_pose_id'):
        value = values[key]
        if not isinstance(value, str) or (value and not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', value)):
            raise ConfigError(f'actions.{key}: invalid pose ID')
    for key in ('home_gesture_enabled', 'recording_buttons_enabled', 'mark_gesture_enabled'):
        values[key] = boolean(values[key], 'actions.' + key)
    if values['home_gesture_enabled'] and not values['home_pose_id']:
        raise ConfigError('请先选择手柄回位姿态，再启用回位手势')
    for key in ('pause_button', 'reset_reference_button'):
        if values[key] not in BUTTONS:
            raise ConfigError(f'actions.{key}: unknown button')
    if values['pause_button'] != 'none' and values['pause_button'] == values['reset_reference_button']:
        raise ConfigError('暂停与重新对齐不能使用同一个按键')
    values['home_threshold'] = number(values['home_threshold'], 'home_threshold', .5, 1)
    values['home_release_threshold'] = number(values['home_release_threshold'], 'home_release_threshold', 0, .49)
    values['posture_weight'] = number(values['posture_weight'], 'posture_weight', .000001, 1)
    return ActionConfig(**values)


class InputActions:
    def __init__(self, config):
        self.config = config
        self.previous = None
        self.home_latched = False

    def update(self, packet, restarted=False):
        masks = {side: getattr(packet, side + '_input').held_mask for side in ('left', 'right')}
        left_x, right_x = packet.left_input.primary_axis[0], packet.right_input.primary_axis[0]
        outside = left_x <= -self.config.home_threshold and right_x >= self.config.home_threshold
        if restarted or self.previous is None:
            self.previous = masks
            self.home_latched = outside
            return []
        previous, self.previous = self.previous, masks
        def pressed(side, bit):
            value = getattr(packet, side + '_input')
            return bool((masks[side] | value.pressed_mask) & bit and not previous[side] & bit)
        result = []
        for action, key in (('pause', 'pause_button'), ('reset_reference', 'reset_reference_button')):
            binding = BUTTONS[getattr(self.config, key)]
            if binding:
                side, bit = binding
                if pressed(side, bit):
                    result.append(action)
        if self.config.recording_buttons_enabled:
            if pressed('left', 1):
                result.append('record_start')
            elif pressed('left', 2):
                result.append('record_stop')
        if self.config.mark_gesture_enabled:
            if masks['left'] & 32 and masks['right'] & 32 and not (previous['left'] & 32 and previous['right'] & 32):
                result.append('record_mark')
        if abs(left_x) <= self.config.home_release_threshold and abs(right_x) <= self.config.home_release_threshold:
            self.home_latched = False
        if self.config.home_gesture_enabled and outside and not self.home_latched:
            self.home_latched = True
            result.append('home')
        return result
