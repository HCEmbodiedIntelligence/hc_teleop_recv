"""ROS/UDP boundary for the HC Cartesian frontend; no robot kinematics."""
from __future__ import annotations

import json
import socket
import time
import hashlib
from dataclasses import asdict
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .config import load_config
from .frontend import Frontend, InputSession
from .peripheral_ros import PeripheralROS
from .protocol import DISCOVERY_REQUEST, PacketError, Pose, decode_pose_packet, decode_vrdata


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
        self.peripherals = PeripheralROS(self, self.config)
        self.session = InputSession(self.config.input_timeout)
        self.received_packets = 0
        self.rejected_packets = 0
        self.last_status = float("-inf")
        self.fk_stamps = {}
        self.sockets = []
        self.target_publishers = {}
        self.fk_subscriptions = []
        self.raw_publisher = None
        self.raw_subscription = None
        for channel in self.config.channels:
            self.target_publishers[channel.id] = self.create_publisher(
                PoseStamped, channel.target_pose_topic, qos_profile_sensor_data)
            self.fk_subscriptions.append(self.create_subscription(
                PoseStamped, channel.fk_pose_topic,
                lambda msg, channel=channel: self._fk_callback(channel, msg), qos_profile_sensor_data))
        self.status_publisher = self.create_publisher(String, "~/status", 10)
        self.buttons_publisher = self.create_publisher(String, self.config.buttons_topic, 100)
        self.configuration_service = self.create_service(Trigger, "~/get_configuration", self._get_configuration)
        self.enable_service = self.create_service(SetBool, "~/set_enabled", self._set_enabled)
        self.stop_subscription = self.create_subscription(
            Bool, self.config.emergency_stop_topic, self._stop_callback, 10)
        if self.config.input_mode == "udp":
            try:
                for port in (self.config.pose_port, self.config.discovery_port):
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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
            f"HC Cartesian frontend: {len(self.config.channels)} channels, input={self.config.input_mode}, "
            f"enabled={self.frontend.enabled}; FK comes from humanoid_motion_server")

    def _get_configuration(self, request, response):
        response.success = True
        response.message = json.dumps({"identity": self.configuration_identity, "configuration": self.configuration_document})
        return response

    def _set_enabled(self, request, response):
        self.frontend.set_enabled(request.data)
        self.peripherals.reset()
        self.peripherals.tick(self.frontend, time.monotonic())
        response.success = True
        response.message = "Enabled; release then press Grip to bind measured FK" if request.data else "Disabled"
        return response

    def _stop_callback(self, message):
        if message.data:
            self.frontend.set_enabled(False)
            self.peripherals.reset()
            self.peripherals.tick(self.frontend, time.monotonic())

    def _fk_callback(self, channel, message):
        if message.header.frame_id != channel.base_frame:
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        # Reject frozen, delayed and future feedback without refreshing its local age.
        if stamp <= 0 or stamp <= self.fk_stamps.get(channel.id, 0) or not -.05 <= age <= self.config.fk_timeout:
            return
        p, q = message.pose.position, message.pose.orientation
        try:
            self.frontend.update_fk(channel.id, Pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)), time.monotonic())
        except ValueError:
            return
        self.fk_stamps[channel.id] = stamp

    def _accept(self, packet, peer):
        now = time.monotonic()
        if not self.session.accept(packet, peer, now):
            self.rejected_packets += 1
            return
        if self.session.restarted:
            self.frontend.reset_bindings("input sender/session changed; release Grip")
            self.peripherals.reset()
            self.button_masks = None
        self.frontend.ingest(packet, now)
        self.received_packets += 1
        inputs = {"left": packet.left_input, "right": packet.right_input}
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
            self.raw_publisher.publish(String(data=json.dumps(packet.as_dict(), separators=(",", ":"))))

    def _vrdata_callback(self, message):
        try:
            if len(message.data) > 65536:
                raise PacketError("/vrdata exceeds 64 KiB")
            self._accept(decode_vrdata(json.loads(message.data)), "vrdata")
        except (PacketError, ValueError):
            self.rejected_packets += 1

    def _poll_udp(self):
        for index, sock in enumerate(self.sockets):
            # Bound work per control tick even under a datagram flood.
            for _ in range(64):
                try:
                    data, peer = sock.recvfrom(2048)
                except BlockingIOError:
                    break
                except OSError:
                    self.rejected_packets += 1
                    break
                if self.config.source_ip and peer[0] != self.config.source_ip:
                    self.rejected_packets += 1
                    continue
                if index == 1:
                    if data == DISCOVERY_REQUEST:
                        try:
                            sock.sendto(f"PICO_RECEIVER_V1|{self.config.pose_port}".encode("ascii"), peer)
                        except OSError:
                            pass
                    continue
                try:
                    self._accept(decode_pose_packet(data), peer)
                except PacketError:
                    self.rejected_packets += 1

    def _tick(self):
        self._poll_udp()
        now = time.monotonic()
        targets = self.frontend.tick(now)
        self.peripherals.tick(self.frontend, now)
        stamp = self.get_clock().now().to_msg()
        for ident, target in targets.items():
            message = PoseStamped()
            message.header.stamp = stamp
            message.header.frame_id = self.frontend.channels[ident].config.base_frame
            message.pose.position.x, message.pose.position.y, message.pose.position.z = target.position
            q = message.pose.orientation
            q.x, q.y, q.z, q.w = target.quaternion
            self.target_publishers[ident].publish(message)
        if now - self.last_status >= .2:
            status = self.frontend.status()
            status.update(self.peripherals.status())
            status.update(received_packets=self.received_packets, rejected_packets=self.rejected_packets,
                          configuration=self.configuration_identity, buttons_topic=self.config.buttons_topic,
                          last_buttons=self.last_buttons)
            self.status_publisher.publish(String(data=json.dumps(status)))
            self.last_status = now

    def destroy_node(self):
        if hasattr(self, 'peripherals'):
            self.peripherals.close()
        for sock in self.sockets:
            sock.close()
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
