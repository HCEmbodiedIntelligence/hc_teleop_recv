"""Shared, ROS-free configuration validation for the receiver and deployment manager."""
from __future__ import annotations

from dataclasses import dataclass
import ipaddress
from pathlib import Path
import re
from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:
    from .peripheral_config import ChassisConfig, GripperConfig

import numpy as np
import yaml


class ConfigError(ValueError):
    pass


def mapping(value, required, optional, label):
    if not isinstance(value, dict) or not all(isinstance(k, str) for k in value):
        raise ConfigError(f"{label} must be a mapping with string keys")
    missing = set(required) - value.keys()
    unknown = value.keys() - set(required) - set(optional)
    if missing or unknown:
        raise ConfigError(f"{label}: missing {sorted(missing)}, unknown {sorted(unknown)}")
    return value


def number(value, label, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{label} must be a finite number")
    if not np.isfinite(value) or not low <= value <= high:
        raise ConfigError(f"{label} must be in [{low}, {high}]")
    return float(value)


def boolean(value, label):
    if not isinstance(value, bool):
        raise ConfigError(f"{label} must be true or false")
    return value


def text(value, label):
    if not isinstance(value, str) or not value or value.strip() != value:
        raise ConfigError(f"{label} must be a non-empty string without surrounding whitespace")
    return value


def topic(value, label):
    value = text(value, label)
    if not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*", value):
        raise ConfigError(f"{label} must be an absolute ROS topic name")
    return value


def vector(value, size, label):
    if not isinstance(value, list) or len(value) != size:
        raise ConfigError(f"{label} must have {size} elements")
    return tuple(number(x, label, -1e6, 1e6) for x in value)


@dataclass(frozen=True)
class ChannelConfig:
    id: str
    controller: str
    clutch_controller: str
    clutch_threshold: float
    target_pose_topic: str
    fk_pose_topic: str
    base_frame: str
    tool_frame: str
    axis_mapping: tuple
    position_scale: float
    max_displacement: float
    orientation_enabled: bool
    filter_alpha: float
    position_deadband: float
    orientation_deadband: float
    workspace_min: tuple | None
    workspace_max: tuple | None


@dataclass(frozen=True)
class ReceiverConfig:
    input_mode: str
    bind_host: str
    source_ip: str
    pose_port: int
    discovery_port: int
    vr_data_topic: str
    publish_vrdata: bool
    rate_hz: float
    input_timeout: float
    fk_timeout: float
    enabled_on_start: bool
    resume_on_a: bool
    emergency_stop_topic: str
    channels: tuple[ChannelConfig, ...]
    robot_id: str = ""
    buttons_topic: str = "/hc_teleop_recv/buttons"
    chassis: ChassisConfig | None = None
    grippers: tuple[GripperConfig, ...] = ()


def parse_config(document: Any) -> ReceiverConfig:
    root = mapping(document, {"schema_version", "channels"}, {"input", "control", "adapter", "chassis", "grippers"}, "config")
    from .peripheral_config import parse_chassis, parse_grippers, validate_peripheral_endpoints
    chassis = parse_chassis(root['chassis']) if 'chassis' in root else None
    grippers = parse_grippers(root.get('grippers', []))
    adapter = mapping(root.get("adapter", {}), set(), {"robot_id", "buttons_topic"}, "adapter")
    robot_id = adapter.get("robot_id", "")
    if not isinstance(robot_id, str) or (robot_id and not re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,63}", robot_id)):
        raise ConfigError("adapter.robot_id must be a robot composition ID")
    buttons_topic = topic(adapter.get("buttons_topic", "/hc_teleop_recv/buttons"), "adapter.buttons_topic")
    if type(root["schema_version"]) is not int or root["schema_version"] != 1:
        raise ConfigError("schema_version must be 1")
    inp = mapping(root.get("input", {}), set(), {
        "mode", "bind_host", "source_ip", "pose_port", "discovery_port", "vr_data_topic", "publish_vrdata"
    }, "input")
    mode = inp.get("mode", "udp")
    if mode not in ("udp", "vrdata"):
        raise ConfigError("input.mode must be udp or vrdata")
    host = inp.get("bind_host", "0.0.0.0")
    source = inp.get("source_ip", "")
    try:
        ipaddress.IPv4Address(host)
        if source:
            ipaddress.IPv4Address(source)
        elif not isinstance(source, str):
            raise ValueError("source_ip must be a string")
    except (ValueError, TypeError) as error:
        raise ConfigError("bind_host/source_ip must be IPv4 addresses (source_ip may be empty)") from error
    ports = []
    for field, default in (("pose_port", 5005), ("discovery_port", 5006)):
        port = inp.get(field, default)
        if type(port) is not int or not 1 <= port <= 65535:
            raise ConfigError(f"input.{field} must be an integer port in [1, 65535]")
        ports.append(port)
    if ports[0] == ports[1]:
        raise ConfigError("pose_port and discovery_port must differ")
    ctrl = mapping(root.get("control", {}), set(), {
        "rate_hz", "input_timeout", "fk_timeout", "enabled_on_start", "resume_on_a", "emergency_stop_topic"
    }, "control")
    entries = root["channels"]
    if not isinstance(entries, list) or len(entries) > 32 or (not entries and chassis is None and not grippers):
        raise ConfigError("channels must contain 1 to 32 entries, or configure chassis/grippers")
    channels = []
    ids, outputs, feedback = set(), set(), set()
    required = {"id", "controller", "target_pose_topic", "fk_pose_topic", "base_frame", "tool_frame", "axis_mapping"}
    optional = {"clutch_controller", "clutch_threshold", "position_scale", "max_displacement",
                "orientation_enabled", "filter_alpha", "position_deadband", "orientation_deadband", "workspace"}
    for entry in entries:
        entry = mapping(entry, required, optional, "channel")
        ident = text(entry["id"], "channel.id")
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", ident) or ident in ids:
            raise ConfigError("channel.id must be unique and use letters, digits or underscores")
        ids.add(ident)
        controller = entry["controller"]
        clutch = entry.get("clutch_controller", controller)
        if controller not in ("left", "right", "head") or clutch not in ("left", "right"):
            raise ConfigError("controller must be left/right/head; clutch_controller must be left/right")
        out = topic(entry["target_pose_topic"], "target_pose_topic")
        fk = topic(entry["fk_pose_topic"], "fk_pose_topic")
        if out in outputs or fk in feedback:
            raise ConfigError("target and FK topics must be unique per channel")
        outputs.add(out)
        feedback.add(fk)
        axes = entry["axis_mapping"]
        if not isinstance(axes, list) or len(axes) != 3:
            raise ConfigError("axis_mapping must be 3x3")
        axes = tuple(vector(row, 3, "axis_mapping row") for row in axes)
        matrix = np.asarray(axes)
        if not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-6, rtol=0) or not np.isclose(np.linalg.det(matrix), 1, atol=1e-6, rtol=0):
            raise ConfigError("axis_mapping must be an orthogonal rotation with determinant +1")
        lo = hi = None
        if "workspace" in entry:
            bounds = mapping(entry["workspace"], {"min", "max"}, set(), "workspace")
            lo, hi = vector(bounds["min"], 3, "workspace.min"), vector(bounds["max"], 3, "workspace.max")
            if any(a >= b for a, b in zip(lo, hi)):
                raise ConfigError("workspace.min must be below workspace.max")
        channels.append(ChannelConfig(
            id=ident, controller=controller, clutch_controller=clutch,
            clutch_threshold=number(entry.get("clutch_threshold", .55), "clutch_threshold", .01, 1),
            target_pose_topic=out, fk_pose_topic=fk,
            base_frame=text(entry["base_frame"], "base_frame"), tool_frame=text(entry["tool_frame"], "tool_frame"),
            axis_mapping=axes,
            position_scale=number(entry.get("position_scale", .8), "position_scale", .001, 10),
            max_displacement=number(entry.get("max_displacement", .8), "max_displacement", .001, 10),
            orientation_enabled=boolean(entry.get("orientation_enabled", True), "orientation_enabled"),
            filter_alpha=number(entry.get("filter_alpha", .75), "filter_alpha", .001, 1),
            position_deadband=number(entry.get("position_deadband", 0), "position_deadband", 0, 1),
            orientation_deadband=number(entry.get("orientation_deadband", 0), "orientation_deadband", 0, np.pi),
            workspace_min=lo, workspace_max=hi,
        ))
    data_topic = topic(inp.get("vr_data_topic", "/vrdata"), "vr_data_topic")
    stop_topic = topic(ctrl.get("emergency_stop_topic", "/teleop/emergency_stop"), "emergency_stop_topic")
    if outputs & feedback or (outputs | feedback) & {data_topic, stop_topic} or data_topic == stop_topic:
        raise ConfigError("input, output, FK and stop topics must not overlap")
    if buttons_topic in outputs | feedback | {data_topic, stop_topic, "/hc_teleop_recv/status"}:
        raise ConfigError("adapter.buttons_topic must not overlap control or status topics")
    reserved = outputs | feedback | {data_topic, stop_topic, buttons_topic, '/hc_teleop_recv/status', '/hc_teleop/joint_cmd'}
    feedback_types = {name: 'reserved' for name in reserved}
    feedback_types['/hc_teleop/joint_states'] = 'joint_state'
    validate_peripheral_endpoints(chassis, grippers, reserved, feedback_types)
    return ReceiverConfig(
        input_mode=mode, bind_host=host, source_ip=source, pose_port=ports[0], discovery_port=ports[1],
        vr_data_topic=data_topic, publish_vrdata=boolean(inp.get("publish_vrdata", True), "publish_vrdata"),
        rate_hz=number(ctrl.get("rate_hz", 100), "rate_hz", 20, 250),
        input_timeout=number(ctrl.get("input_timeout", .25), "input_timeout", .02, 2),
        fk_timeout=number(ctrl.get("fk_timeout", .25), "fk_timeout", .02, 2),
        enabled_on_start=boolean(ctrl.get("enabled_on_start", False), "enabled_on_start"),
        resume_on_a=boolean(ctrl.get("resume_on_a", True), "resume_on_a"),
        emergency_stop_topic=stop_topic, channels=tuple(channels),
        robot_id=robot_id, buttons_topic=buttons_topic,
        chassis=chassis, grippers=grippers,
    )


def load_config(path: str | Path) -> ReceiverConfig:
    try:
        path = Path(path)
        if path.stat().st_size > 1024 * 1024:
            raise ConfigError("receiver configuration exceeds 1 MiB")
        return parse_config(yaml.safe_load(path.read_text(encoding="utf-8")))
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError(f"cannot read receiver configuration {path}: {error}") from error


def validate_motion_channels(config: ReceiverConfig, channels: list[dict]) -> None:
    """Require each receiver output to address the same frame/tool as its measured FK."""
    by_endpoint = {channel.get("endpoint"): channel for channel in channels}
    from .peripheral_config import validate_peripheral_endpoints
    validate_peripheral_endpoints(config.chassis, config.grippers, by_endpoint)
    for receiver in config.channels:
        motion = by_endpoint.get(receiver.target_pose_topic)
        if motion is None or motion.get("kind") != "servo_p":
            raise ConfigError(f"{receiver.id}: target_pose_topic must match a motion servo_p endpoint")
        for key, expected in (("base_frame", receiver.base_frame), ("tip_frame", receiver.tool_frame), ("fk_pose_topic", receiver.fk_pose_topic)):
            if motion.get(key) != expected:
                raise ConfigError(f"{receiver.id}: {key} differs from the motion channel")
