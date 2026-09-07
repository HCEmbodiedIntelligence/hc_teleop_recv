"""Relative clutch and target generation. FK is supplied by the motion server."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .config import ChannelConfig, ReceiverConfig
from .pose_math import normalize_quaternion, relative_target, stabilize_pose
from .protocol import Pose, PosePacket, is_sequence_newer


def pose_arrays(pose: Pose):
    p = np.asarray(pose.position, dtype=float)
    if p.shape != (3,) or not np.all(np.isfinite(p)):
        raise ValueError("pose position must be a finite 3-vector")
    return p.copy(), normalize_quaternion(pose.quaternion)


def as_pose(arrays) -> Pose:
    return Pose(tuple(float(x) for x in arrays[0]), tuple(float(x) for x in arrays[1]))


class InputSession:
    """One active sender, monotonic device time/sequence, and explicit session expiry."""
    def __init__(self, timeout: float):
        self.timeout = timeout
        self.peer = None
        self.sequence = None
        self.timestamp = None
        self.received_at = None
        self.restarted = False

    def accept(self, packet: PosePacket, peer, now: float) -> bool:
        expired = self.received_at is None or now - self.received_at > self.timeout
        self.restarted = False
        if not expired:
            if peer != self.peer or not is_sequence_newer(packet.sequence, self.sequence):
                return False
            if packet.vr_timestamp <= self.timestamp:
                return False
        else:
            self.restarted = True
        self.peer, self.sequence, self.timestamp, self.received_at = peer, packet.sequence, packet.vr_timestamp, now
        return True


@dataclass
class ChannelState:
    config: ChannelConfig
    state: str = "disabled"
    reason: str = "disabled"
    fk: Pose | None = None
    fk_at: float = float("-inf")
    reference_vr: Pose | None = None
    reference_fk: Pose | None = None
    target: Pose | None = None


class Frontend:
    def __init__(self, config: ReceiverConfig):
        self.config = config
        self.enabled = config.enabled_on_start
        self.channels = {c.id: ChannelState(c) for c in config.channels}
        self.packet = None
        self.input_at = float("-inf")
        self.a_down = False
        self.reset_bindings("startup")

    def _release(self, channel: ChannelState, reason: str, state="wait_release"):
        channel.state = state if self.enabled else "disabled"
        channel.reason = reason
        channel.reference_vr = channel.reference_fk = channel.target = None

    def reset_bindings(self, reason: str):
        for channel in self.channels.values():
            self._release(channel, reason)

    def set_enabled(self, enabled: bool):
        self.enabled = enabled
        self.reset_bindings("enabled; release Grip before engaging" if enabled else "disabled")

    def update_fk(self, ident: str, pose: Pose, now: float):
        normalized = as_pose(pose_arrays(pose))
        self.channels[ident].fk = normalized
        self.channels[ident].fk_at = now

    def ingest(self, packet: PosePacket, now: float):
        if now - self.input_at > self.config.input_timeout:
            self.reset_bindings("input session restarted")
        self.packet, self.input_at = packet, now
        a_down = bool((packet.right_input.held_mask | packet.right_input.pressed_mask) & 1)
        if self.config.resume_on_a and a_down and not self.a_down and not self.enabled:
            self.set_enabled(True)
        self.a_down = a_down
        for channel in self.channels.values():
            cfg = channel.config
            if not self.enabled:
                continue
            if packet.protocol_version != 2 or not packet.tracked(cfg.controller) or not packet.tracked(cfg.clutch_controller):
                self._release(channel, "controller tracking/input unavailable")
                continue
            clutch = getattr(packet, cfg.clutch_controller + "_input").grip >= cfg.clutch_threshold
            if not clutch:
                self._release(channel, "Grip released", "ready")
                continue
            if channel.state == "ready":
                if channel.fk is None or now - channel.fk_at > self.config.fk_timeout:
                    self._release(channel, "fresh measured FK required")
                    continue
                p = np.asarray(channel.fk.position)
                if cfg.workspace_min is not None and (np.any(p < cfg.workspace_min) or np.any(p > cfg.workspace_max)):
                    self._release(channel, "measured FK is outside configured workspace")
                    continue
                channel.reference_fk = channel.fk
                channel.reference_vr = getattr(packet, cfg.controller)
                channel.target = channel.fk
                channel.state, channel.reason = "active", "Grip engaged"

    def tick(self, now: float) -> dict[str, Pose]:
        targets = {}
        for ident, channel in self.channels.items():
            if not self.enabled or channel.state != "active":
                continue
            if now - self.input_at > self.config.input_timeout or now - channel.fk_at > self.config.fk_timeout:
                self._release(channel, "input or measured FK timed out")
                continue
            cfg = channel.config
            try:
                raw = relative_target(
                    *pose_arrays(getattr(self.packet, cfg.controller)),
                    *pose_arrays(channel.reference_vr), *pose_arrays(channel.reference_fk),
                    cfg.axis_mapping, cfg.position_scale, cfg.max_displacement, cfg.orientation_enabled,
                )
                if cfg.workspace_min is not None:
                    raw = (np.clip(raw[0], cfg.workspace_min, cfg.workspace_max), raw[1])
                target = stabilize_pose(*raw, *pose_arrays(channel.target), cfg.position_deadband,
                                        cfg.orientation_deadband, cfg.filter_alpha)
                channel.target = as_pose(target)
                targets[ident] = channel.target
            except (ValueError, FloatingPointError) as error:
                self._release(channel, f"invalid pose mapping: {error}")
        return targets

    def status(self):
        return {"enabled": self.enabled, "channels": {
            ident: {"state": c.state, "reason": c.reason, "base_frame": c.config.base_frame,
                    "tool_frame": c.config.tool_frame, "target_pose_topic": c.config.target_pose_topic}
            for ident, c in self.channels.items()
        }}
