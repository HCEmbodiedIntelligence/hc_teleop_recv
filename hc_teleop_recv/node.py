"""ROS/UDP boundary for the HC Cartesian frontend; no robot kinematics."""
from __future__ import annotations

import json
import socket
import struct
import time
import hashlib
import uuid
from dataclasses import asdict
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .config import load_config
from .actions import ActionConfig, InputActions
from .frontend import Frontend, InputSession
from .log_summary import LogSummary
from .state_log import TeleopStateLog
from .vr_status import control_status
from .peripheral_ros import PeripheralROS
from .protocol import (
    DISCOVERY_REQUEST, PacketError, Pose, decode_pose_packet, decode_vrdata,
    envelope, encode_json_packet,
)


class TeleopRecvNode(Node):
    def __init__(self, *, config=None, **kwargs):
        super().__init__("hc_teleop_recv", **kwargs)
        config_file = self.declare_parameter("config_file", "").value
        if config is None and not config_file:
            raise ValueError("config_file must point to a robot-model hc_teleop YAML")
        self.config = config or load_config(config_file)
        self.configuration_document = asdict(self.config)
        content = Path(config_file).read_bytes() if config is None else json.dumps(self.configuration_document, sort_keys=True).encode()
        self.configuration_identity = {"robot_id": self.config.robot_id,
            "path": config_file, "sha256": hashlib.sha256(content).hexdigest()}
        self.button_masks = None
        self.last_buttons = None
        self.frontend = Frontend(self.config)
        self.input_actions = InputActions(self.config.actions or ActionConfig())
        self.last_action = None
        self.last_action_request = {}
        self.peripherals = PeripheralROS(self, self.config)
        self.session = InputSession(self.config.input_timeout)
        self.received_packets = 0
        self.rejected_packets = 0
        self.last_packet_error = None
        self.log_summary = LogSummary(("input_rejected", "vr_send_failed", "safety_resume", "safety_stop"))
        self.state_log = TeleopStateLog(self.config)
        self.last_status = float("-inf")
        self.vr_session_id = uuid.uuid4().hex
        self.vr_event_sequence = 0
        self.last_vr_status = float('-inf')
        self.last_vr_check = float('-inf')
        self.last_vr_compatibility = float('-inf')
        self.last_vr_state = None
        self.fk_stamps = {}
        self.fk_ros_now_ns = 0
        self.input_stamp_ns = 0
        self.udp_backlog = False
        self.last_target_times = {}
        self.sockets = []
        self.target_publishers = {}
        self.fk_subscriptions = []
        self.raw_publisher = None
        self.raw_subscription = None
        arm_channels = self.config.channels if self.config.arm_control_enabled else ()
        for channel in arm_channels:
            self.target_publishers[channel.id] = self.create_publisher(
                PoseStamped, channel.target_pose_topic, qos_profile_sensor_data)
            self.fk_subscriptions.append(self.create_subscription(
                PoseStamped, channel.fk_pose_topic,
                lambda msg, channel=channel: self._fk_callback(channel, msg), qos_profile_sensor_data))
        self.status_publisher = self.create_publisher(String, "~/status", 10)
        self.buttons_publisher = self.create_publisher(String, self.config.buttons_topic, 100)
        self.configuration_service = self.create_service(Trigger, "~/get_configuration", self._get_configuration)
        self.enable_service = self.create_service(SetBool, "~/set_enabled", self._set_enabled)
        self.motion_service = self.create_service(SetBool, '~/set_motion_active', self._set_motion_active)
        self.reference_service = self.create_service(Trigger, '~/reset_reference', self._reset_reference)
        self.home_service = self.create_service(Trigger, '~/home', self._home)
        self.action_publisher = self.create_publisher(String, '~/actions', 10)
        self.event_subscription = self.create_subscription(String, '~/events', self._action_event, 10)
        self.stop_publisher = self.create_publisher(
            Bool, self.config.emergency_stop_topic, 10)
        self.stop_subscription = self.create_subscription(
            Bool, self.config.emergency_stop_topic, self._stop_callback, 10)
        self.last_a_held = False
        self.last_peer_host = self.config.source_ip if self.config.source_ip else None
        self.outbound_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.outbound_socket.setblocking(False)
        self.outbound_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        if self.config.input_mode == "udp":
            try:
                for port in (self.config.pose_port, self.config.discovery_port):
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    # Linux timestamps each datagram on arrival, before it can
                    # sit in the socket queue during an executor stall.
                    sock.setsockopt(socket.SOL_SOCKET, getattr(socket, 'SO_TIMESTAMPNS', 35), 1)
                    self.sockets.append(sock)
                    sock.bind((self.config.bind_host, port))
                    sock.setblocking(False)
            except OSError:
                for sock in self.sockets:
                    sock.close()
                raise
            if self.config.publish_vrdata:
                self.raw_publisher = self.create_publisher(String, self.config.vr_data_topic, 10)
        else:
            self.raw_subscription = self.create_subscription(
                String, self.config.vr_data_topic, self._vrdata_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / self.config.rate_hz, self._tick)
        self.get_logger().info(
            f"HC Cartesian frontend: {len(arm_channels)} enabled arm channels, input={self.config.input_mode}, "
            f"enabled={self.frontend.enabled}; FK comes from humanoid_motion_server")

    def _get_configuration(self, request, response):
        response.success = True
        response.message = json.dumps({"identity": self.configuration_identity, "configuration": self.configuration_document})
        return response

    def _send_vr_event(self, kind: str, reason: str, **payload_kwargs):
        host = self.last_peer_host or self.config.source_ip
        if not host:
            return
        payload = {"reason": reason, "message": reason}
        payload.update(payload_kwargs)
        self.vr_event_sequence += 1
        packet = envelope(kind, "hc_teleop_recv", payload, session_id=self.vr_session_id,
                          sequence=self.vr_event_sequence)
        raw = encode_json_packet(packet)
        try:
            for _ in range(2):
                self.outbound_socket.sendto(raw, (host, self.config.event_port))
        except OSError as exc:
            self.log_summary.record("vr_send_failed", f"{kind} to {host}:{self.config.event_port}: {exc}")

    def _sync_vr_status(self, now, *, force=False):
        if not force and now - self.last_vr_check < .05:
            return
        self.last_vr_check = now
        host = self.last_peer_host or self.config.source_ip
        if not host:
            return
        status = control_status(self.frontend, now, input_ready=not self.udp_backlog)
        identity = (host, json.dumps(status, sort_keys=True))
        changed = identity != self.last_vr_state
        if force or changed or now - self.last_vr_status >= .2:
            self._send_vr_event('teleop_status', status['reason'], **{
                key: value for key, value in status.items() if key != 'reason'},
                robot_id=self.config.robot_id, state_sync=True, valid_for_ms=1000)
            self.last_vr_status = now
            self.last_vr_state = identity
        if force or changed or now - self.last_vr_compatibility >= 1.0:
            # Keep compatibility with the existing safety event protocol. Send the actual
            # enable state after reconnect/restart/loss, without changing ROS
            # control state or generating a local resume/stop action.
            enabled = status['enabled'] and not status['motion_active'] and status['input_fresh']
            self._send_vr_event('safety_resume' if enabled else 'safety_stop', status['reason'],
                                enabled=status['enabled'], motion_active=status['motion_active'],
                                input_fresh=status['input_fresh'], state=status['state'], state_sync=True)
            self.last_vr_compatibility = now

    def _handle_safety_resume(self, reason: str) -> None:
        if self.frontend.inhibited:
            self._sync_vr_status(time.monotonic(), force=True)
            return
        if not self.frontend.enabled:
            self.frontend.set_enabled(True)
            self.peripherals.reset()
            self.peripherals.tick(self.frontend, time.monotonic())
        if self.stop_publisher is not None:
            self.stop_publisher.publish(Bool(data=False))
        self._send_vr_event("safety_resume", reason)
        self.log_summary.record("safety_resume", reason)

    def _handle_safety_stop(self, reason: str) -> None:
        if self.frontend.enabled:
            self.frontend.set_enabled(False)
            self.peripherals.reset()
            self.peripherals.tick(self.frontend, time.monotonic())
        self._send_vr_event("safety_stop", reason)
        self.log_summary.record("safety_stop", reason)

    def _set_enabled(self, request, response):
        if request.data and self.frontend.inhibited:
            response.success, response.message = False, '回位执行中，请等待结束或先取消回位'
            return response
        if request.data:
            self._handle_safety_resume("Service set_enabled True")
            response.success = True
            response.message = "Enabled; hold Grip to bind measured FK"
        else:
            self._handle_safety_stop("Service set_enabled False")
            response.success = True
            response.message = "Disabled"
        return response

    def _stop_callback(self, message):
        if message.data:
            self._handle_safety_stop("Emergency stop topic asserted")

    def _set_motion_active(self, request, response):
        if request.data and self.frontend.inhibited:
            response.success, response.message = False, '已有回位动作执行中'
            return response
        self.frontend.inhibited = request.data
        self._handle_safety_stop('回位执行中' if request.data else '回位结束，参考已清除')
        self.frontend.reset_bindings('waiting for fresh FK after motion')
        for channel in self.frontend.channels.values():
            channel.fk_at = float('-inf')
        response.success, response.message = True, 'Motion active' if request.data else 'Motion released'
        return response

    def _reset_reference(self, request, response):
        if self.frontend.inhibited:
            response.success, response.message = False, '回位执行中，暂不能重新对齐'
            return response
        self.frontend.reset_bindings('reference reset; waiting for fresh FK')
        for channel in self.frontend.channels.values():
            channel.fk_at = float('-inf')
        response.success, response.message = True, '参考已清除，收到新反馈后自动对齐'
        self._send_vr_event('teleop_reference_reset', response.message)
        return response

    def _home(self, request, response):
        response.success = self._request_action('home')
        response.message = '已提交回位请求，结果见遥操状态' if response.success else '未提交：检查回位姿态、管理器连接或当前动作'
        return response

    def _request_action(self, action):
        now = time.monotonic()
        if now - self.last_action_request.get(action, float('-inf')) < .2:
            return False
        if action == 'home' and (self.frontend.inhibited or not self.input_actions.config.home_pose_id):
            return False
        if not self.action_publisher.get_subscription_count():
            self._send_vr_event('teleop_action_error', '管理器未连接，操作未执行')
            return False
        self.last_action_request[action] = now
        payload = {'id': uuid.uuid4().hex, 'action': action, 'robot_id': self.config.robot_id,
                   'stamp_ns': time.time_ns(), 'configuration_sha256': self.configuration_identity['sha256']}
        if action == 'home':
            payload['pose_id'] = self.input_actions.config.home_pose_id
        self.action_publisher.publish(String(data=json.dumps(payload)))
        return True

    def _action_event(self, message):
        if len(message.data) > 8192:
            return
        try:
            event = json.loads(message.data)
            if event.get('robot_id') != self.config.robot_id or event.get('kind') not in {
                    'teleop_action', 'recording_started', 'recording_stopped', 'recording_marked',
                    'recording_error', 'replay_status'}:
                return
            if event['kind'] != 'replay_status':
                self.last_action = event
            payload = {key: event[key] for key in ('action', 'ok', 'filename', 'recording',
                'status', 'is_active', 'paused', 'position', 'duration', 'speed', 'error') if key in event}
            self._send_vr_event(event['kind'], str(event.get('message', ''))[:1024],
                                request_id=event.get('id', ''), **payload)
        except (ValueError, TypeError, AttributeError):
            return

    def _fk_callback(self, channel, message):
        if message.header.frame_id != channel.base_frame:
            self.state_log.fk_error_kinds[channel.id] = 'frame_mismatch'
            self.state_log.fk_errors[channel.id] = (
                f'frame mismatch: expected={channel.base_frame}, received={message.header.frame_id}')[:256]
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        ros_now = self.get_clock().now().nanoseconds
        if ros_now < self.fk_ros_now_ns:
            self.fk_stamps.clear()
            self.frontend.reset_bindings('ROS clock reset; waiting for fresh FK')
            for state in self.frontend.channels.values():
                state.fk_at = float('-inf')
        self.fk_ros_now_ns = ros_now
        age = (ros_now - stamp) / 1e9
        # Reject frozen, delayed and future feedback without refreshing its local age.
        if stamp <= 0 or stamp <= self.fk_stamps.get(channel.id, 0) or not -.05 <= age <= self.config.fk_timeout:
            self.state_log.fk_error_kinds[channel.id] = 'timestamp_invalid'
            self.state_log.fk_errors[channel.id] = f'invalid/stale stamp: stamp={stamp}, age_s={age:.3f}'
            return
        p, q = message.pose.position, message.pose.orientation
        try:
            self.frontend.update_fk(channel.id, Pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)),
                                    time.monotonic() - max(0., age))
        except ValueError as error:
            self.state_log.fk_error_kinds[channel.id] = 'pose_invalid'
            self.state_log.fk_errors[channel.id] = f'invalid pose: {error}'[:256]
            return
        self.fk_stamps[channel.id] = stamp
        self.state_log.fk_errors[channel.id] = ''
        self.state_log.fk_error_kinds[channel.id] = ''

    def _accept(self, packet, peer, received_at=None):
        now = time.monotonic()
        received_at = now if received_at is None else received_at
        age = now - received_at
        if not 0 <= age < self.config.input_timeout:
            self._reject_packet('input expired in transport queue')
            return
        if not self.session.accept(packet, peer, received_at):
            self._reject_packet(f"session/order check failed: peer={peer}, sequence={packet.sequence}, vr_timestamp={packet.vr_timestamp}")
            return
        if isinstance(peer, (tuple, list)) and peer and peer[0]:
            self.last_peer_host = peer[0]
        if self.session.restarted:
            self.frontend.reset_bindings("input sender/session changed; rebind measured FK")
            self.peripherals.reset()
            self.button_masks = None
        self.state_log.accepted(packet, now, self.session.restarted)
        self.frontend.ingest(packet, received_at)
        self.input_stamp_ns = self.get_clock().now().nanoseconds - int(age * 1e9)
        self.received_packets += 1
        inputs = {"left": packet.left_input, "right": packet.right_input}

        right_input = packet.right_input
        right_held = int(getattr(right_input, "held_mask", 0))
        right_pressed = int(getattr(right_input, "pressed_mask", 0))
        a_held = bool(right_held & 1)
        a_down = bool(right_pressed & 1) or (a_held and not self.last_a_held)
        self.last_a_held = a_held
        if self.config.resume_on_a and a_down:
            self._handle_safety_resume("VR controller A button pressed")

        for action in self.input_actions.update(packet, self.session.restarted):
            if action == 'pause':
                self._handle_safety_stop('手柄暂停遥操')
                self._request_action('pause')
            elif action == 'reset_reference':
                self._reset_reference(None, Trigger.Response())
            else:
                self._request_action(action)

        edges = []
        if self.button_masks is not None:
            for hand, value in inputs.items():
                previous = self.button_masks[hand]
                for action, mask in (("pressed", value.held_mask & ~previous), ("released", previous & ~value.held_mask)):
                    edges.extend({"controller": hand, "button": button, "action": action}
                                 for button in value.decode_buttons(mask))
        self.button_masks = {hand: value.held_mask for hand, value in inputs.items()}
        self.last_buttons = {"stamp_ns": self.get_clock().now().nanoseconds,
            "sequence": packet.sequence, "vr_timestamp": packet.vr_timestamp,
            "inputs": {hand: value.as_dict() for hand, value in inputs.items()}, "edges": edges}
        self.buttons_publisher.publish(String(data=json.dumps(self.last_buttons)))
        if self.raw_publisher is not None:
            raw = packet.as_dict()
            raw['received_stamp_ns'] = self.input_stamp_ns
            self.raw_publisher.publish(String(data=json.dumps(raw, separators=(",", ":"))))

    def _vrdata_callback(self, message):
        self.state_log.transport(time.monotonic(), self.config.vr_data_topic)
        try:
            if len(message.data) > 65536:
                raise PacketError("/vrdata exceeds 64 KiB")
            document = json.loads(message.data)
            if not isinstance(document, dict):
                raise PacketError('/vrdata must be a JSON object')
            stamp = document.get('received_stamp_ns')
            if type(stamp) is not int or stamp <= 0:
                raise PacketError('/vrdata requires received_stamp_ns in the ROS clock domain')
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if not -.05 <= age < self.config.input_timeout:
                raise PacketError('/vrdata source timestamp expired or future-dated')
            self._accept(decode_vrdata(document), "vrdata", time.monotonic() - max(0., age))
        except (PacketError, ValueError) as error:
            self._reject_packet(f"vrdata: {error}")

    def _poll_udp(self):
        self.udp_backlog = False
        for index, sock in enumerate(self.sockets):
            # Bound work per control tick even under a datagram flood.
            for _ in range(64):
                try:
                    if index == 1:
                        data, peer = sock.recvfrom(2048)
                        ancillary = []
                        flags = 0
                    else:
                        data, ancillary, flags, peer = sock.recvmsg(2048, 64)
                except BlockingIOError:
                    break
                except OSError as error:
                    self._reject_packet(f"UDP receive failed: {error}")
                    break
                if index == 0:
                    self.state_log.transport(time.monotonic(), peer)
                if self.config.source_ip and peer[0] != self.config.source_ip:
                    self._reject_packet(f"source IP rejected: {peer[0]}")
                    continue
                if index == 1:
                    if data == DISCOVERY_REQUEST:
                        if (self.session.received_at is None or
                                time.monotonic() - self.session.received_at > self.config.input_timeout):
                            self.last_peer_host = peer[0]
                        try:
                            sock.sendto(f"PICO_RECEIVER_V1|{self.config.pose_port}".encode("ascii"), peer)
                        except OSError:
                            pass
                    continue
                try:
                    packet = decode_pose_packet(data)
                    stamps = [struct.unpack('@ll', value[:16]) for level, kind, value in ancillary
                              if level == socket.SOL_SOCKET and
                              kind == getattr(socket, 'SO_TIMESTAMPNS', 35) and len(value) >= 16]
                    if not stamps or flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
                        raise PacketError('UDP arrival timestamp missing or datagram truncated')
                    sec, nsec = stamps[-1]
                    age_ns = time.time_ns() - (sec * 1_000_000_000 + nsec)
                    if age_ns < -50_000_000:
                        raise PacketError('UDP arrival timestamp is in the future')
                    self._accept(packet, peer, time.monotonic() - max(0, age_ns) / 1e9)
                except PacketError as error:
                    self._reject_packet(f"peer={peer}: {error}")
            else:
                if index == 0:
                    # Finish draining on the next tick; never publish a pose
                    # from the middle of a backlog. Button edges above retain order.
                    self.udp_backlog = True

    def _reject_packet(self, detail):
        self.rejected_packets += 1
        self.last_packet_error = detail[:1024]
        self.log_summary.record("input_rejected", detail)

    def _flush_diagnostics(self, now):
        for category, message in self.log_summary.poll(now):
            if category == "safety_resume":
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)

    def _tick(self):
        self._poll_udp()
        now = time.monotonic()
        self._flush_diagnostics(now)
        targets = self.frontend.tick(now, input_ready=not self.udp_backlog)
        self.peripherals.tick(self.frontend, now, input_ready=not self.udp_backlog)
        stamp = Time(nanoseconds=max(0, self.input_stamp_ns)).to_msg()
        for ident, target in targets.items():
            message = PoseStamped()
            message.header.stamp = stamp
            message.header.frame_id = self.frontend.channels[ident].config.base_frame
            message.pose.position.x, message.pose.position.y, message.pose.position.z = target.position
            q = message.pose.orientation
            q.x, q.y, q.z, q.w = target.quaternion
            self.target_publishers[ident].publish(message)
            self.last_target_times[ident] = now
        self._sync_vr_status(now)
        if now - self.last_status >= .2:
            peripheral_status = self.peripherals.status()
            recent_targets = {ident: True for ident, at in self.last_target_times.items()
                              if now - at <= self.config.input_timeout and
                              self.frontend.channels[ident].state == 'active'}
            for message in self.state_log.poll(now, self.frontend, recent_targets, peripheral_status,
                                               self.received_packets, self.rejected_packets):
                self.get_logger().info(message)
            status = self.frontend.status()
            status.update(peripheral_status)
            status.update(received_packets=self.received_packets, rejected_packets=self.rejected_packets,
                          last_packet_error=self.last_packet_error,
                          configuration=self.configuration_identity, buttons_topic=self.config.buttons_topic,
                          last_buttons=self.last_buttons)
            status.update(motion_active=self.frontend.inhibited, last_action=self.last_action,
                          actions=asdict(self.input_actions.config),
                          control_status=control_status(self.frontend, now, input_ready=not self.udp_backlog))
            self.status_publisher.publish(String(data=json.dumps(status)))
            self.last_status = now

    def destroy_node(self):
        if hasattr(self, 'peripherals'):
            self.peripherals.close()
        for sock in self.sockets:
            sock.close()
        if hasattr(self, 'outbound_socket') and self.outbound_socket is not None:
            try:
                self.outbound_socket.close()
            except OSError:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopRecvNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
