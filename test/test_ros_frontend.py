"""Exercise actual ROS messages, QoS, UDP discovery and timeout handling without hardware."""
from dataclasses import replace
import os
import socket
import time

import numpy as np
import pytest
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from hc_teleop_recv.config import parse_config
from hc_teleop_recv.node import TeleopRecvNode
from test_frontend import document, packet, wire


def unused_ports():
    sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2)]
    try:
        for sock in sockets:
            sock.bind(("127.0.0.1", 0))
        return [sock.getsockname()[1] for sock in sockets]
    finally:
        for sock in sockets:
            sock.close()


@pytest.mark.parametrize('arm_enabled', [False, True])
def test_arm_switch_controls_owned_ros_publishers_without_removing_grippers(arm_enabled):
    # Only construct nodes in an isolated test domain; never run the control timer.
    context = Context()
    context.init(args=[], domain_id=210 + os.getpid() % 10)
    recv = None
    try:
        doc = document()
        doc['input'] = {'mode': 'vrdata'}
        doc['control']['arm_control_enabled'] = arm_enabled
        doc['channels'].append(dict(doc['channels'][0], id='left_arm', controller='left',
            target_pose_topic='/teleop/left/servo_p', fk_pose_topic='/teleop/left/fk_pose'))
        doc['grippers'] = [{'id': 'tool', 'enabled': True}]
        recv = TeleopRecvNode(config=parse_config(doc), context=context)
        owned_topics = {publisher.topic_name for publisher in recv.publishers}
        for channel in recv.config.channels:
            assert (channel.target_pose_topic in owned_topics) is arm_enabled
        assert len(recv.fk_subscriptions) == (2 if arm_enabled else 0)
        assert len(recv.config.channels) == 2
        assert recv.config.grippers[0].command_topic in owned_topics
    finally:
        if recv is not None:
            recv.destroy_node()
        context.shutdown()


@pytest.mark.parametrize("input_mode", ["udp", "vrdata"])
def test_input_to_servo_p_with_measured_fk_and_stops(input_mode):
    from std_msgs.msg import String
    import json

    context = Context()
    context.init(args=[], domain_id=210 + os.getpid() % 10)
    executor = SingleThreadedExecutor(context=context)
    pose_port, discovery_port = unused_ports()
    doc = document()
    doc["input"] = {"mode": input_mode, "bind_host": "127.0.0.1", "pose_port": pose_port,
                    "discovery_port": discovery_port, "publish_vrdata": False}
    doc["control"].update(enabled_on_start=False, input_timeout=.15, fk_timeout=.15)
    recv = TeleopRecvNode(config=parse_config(doc), context=context)
    peer = Node("mock_motion_feedback", context=context)
    executor.add_node(recv)
    executor.add_node(peer)
    fk_pub = peer.create_publisher(PoseStamped, "/teleop/arm/fk_pose", qos_profile_sensor_data)
    stop_pub = peer.create_publisher(Bool, "/teleop/emergency_stop", 10)
    raw_pub = peer.create_publisher(String, "/vrdata", qos_profile_sensor_data)
    targets = []
    sub = peer.create_subscription(PoseStamped, "/teleop/arm/servo_p", targets.append, qos_profile_sensor_data)
    client = peer.create_client(SetBool, "/hc_teleop_recv/set_enabled")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(.2)
    seq = 0

    def send(grip, position=(0., 0., 0.), flags=7):
        nonlocal seq
        p = packet(seq, grip, position, flags=flags)
        seq += 1
        if input_mode == "udp":
            sock.sendto(wire(p), ("127.0.0.1", pose_port))
        else:
            raw_pub.publish(String(data=json.dumps({**p.as_dict(), 'received_stamp_ns': peer.get_clock().now().nanoseconds})))

    def feedback(frame="base", stamp=None):
        msg = PoseStamped()
        msg.header.stamp = stamp or peer.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = .2, .3, .4
        msg.pose.orientation.w = 1.
        fk_pub.publish(msg)

    def pump(duration, *, grip=None, position=(0., 0., 0.), fk=True, frame="base", stamp=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if fk:
                feedback(frame, stamp)
            if grip is not None:
                send(grip, position)
            for _ in range(8):
                executor.spin_once(timeout_sec=.0002)
            time.sleep(.005)

    try:
        pump(.3)
        if input_mode == "udp":
            sock.sendto(b"PICO_DISCOVER_V1", ("127.0.0.1", discovery_port))
            pump(.03)
            assert sock.recvfrom(256)[0] == f"PICO_RECEIVER_V1|{pose_port}".encode()
        assert client.wait_for_service(timeout_sec=1)
        future = client.call_async(SetBool.Request(data=True))
        executor.spin_until_future_complete(future, timeout_sec=1)
        assert future.result().success
        pump(.04, grip=1.)
        assert targets
        pump(.04, grip=0.)
        pump(.04, grip=1.)
        assert targets
        assert targets[-1].header.frame_id == "base"
        np.testing.assert_allclose([targets[0].pose.position.x, targets[0].pose.position.y, targets[0].pose.position.z], [.2, .3, .4])
        pump(.05, grip=1., position=(0., 0., -.1))
        assert targets[-1].pose.position.x == pytest.approx(.28, abs=1e-6)
        pump(.04, grip=0.)
        targets.clear()
        pump(.04, grip=0.)
        assert targets == []

        # Fresh packets keep arriving, but FK stops: no stale feedback may sustain motion.
        pump(.03, grip=1.)
        pump(.24, grip=1., fk=False)
        targets.clear()
        pump(.03, grip=1., fk=False)
        assert targets == []
        pump(.04, grip=1.)  # Fresh FK permits rebinding while Grip stays held.
        assert targets
        pump(.03, grip=0.)
        pump(.03, grip=1.)
        assert targets

        # A frozen header or wrong FK frame must not refresh the measured pose's age.
        frozen_stamp = peer.get_clock().now().to_msg()
        pump(.23, grip=1., stamp=frozen_stamp)
        targets.clear()
        pump(.04, grip=1., frame="wrong_base")
        assert targets == []
        pump(.03, grip=0.)
        pump(.03, grip=1.)
        assert targets

        # Input loss expires publication even when measured FK continues.
        pump(.25)
        targets.clear()
        pump(.04)
        assert targets == []
        pump(.04, grip=1.)
        assert targets
        pump(.03, grip=0.)
        pump(.03, grip=1.)
        assert targets
        stop_pub.publish(Bool(data=True))
        pump(.04, grip=1.)
        targets.clear()
        pump(.04, grip=1.)
        assert targets == []

        publisher_topics = {name for name, _ in peer.get_topic_names_and_types()
                            if peer.count_publishers(name)}
        assert "/hc_teleop/joint_cmd" not in publisher_topics
    finally:
        sock.close()
        executor.remove_node(recv)
        executor.remove_node(peer)
        recv.destroy_node()
        peer.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_buttons_are_observable_and_configuration_is_queryable():
    import json
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from hc_teleop_recv.protocol import ControllerInput
    context = Context(); context.init(args=[], domain_id=190 + os.getpid() % 10)
    executor = SingleThreadedExecutor(context=context)
    doc = document(); doc['input'] = {'mode':'vrdata'}
    doc['adapter'] = {'robot_id':'lab'}
    recv = TeleopRecvNode(config=parse_config(doc),context=context)
    peer = Node('button_test',context=context)
    executor.add_node(recv);executor.add_node(peer)
    received=[]
    peer.create_subscription(String,'/hc_teleop_recv/buttons',lambda m:received.append(json.loads(m.data)),10)
    client = peer.create_client(Trigger,'/hc_teleop_recv/get_configuration')
    try:
        end=time.monotonic()+.4
        while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
        for seq,mask in enumerate([0,2,2,0]):
            frame=replace(packet(seq),right_input=ControllerInput(held_mask=mask))
            recv._accept(frame,'test')
            end=time.monotonic()+.04
            while time.monotonic()<end:executor.spin_once(timeout_sec=.002)
        edges=[edge for value in received for edge in value['edges']]
        assert edges==[{'controller':'right','button':'secondary','action':'pressed'},
                       {'controller':'right','button':'secondary','action':'released'}]
        assert client.wait_for_service(timeout_sec=1)
        future=client.call_async(Trigger.Request());executor.spin_until_future_complete(future,timeout_sec=1)
        result=json.loads(future.result().message)
        assert result['identity']['robot_id']=='lab'
        assert len(result['identity']['sha256'])==64
        assert result['configuration']['channels'][0]['target_pose_topic']=='/teleop/arm/servo_p'
    finally:
        executor.shutdown();recv.destroy_node();peer.destroy_node();context.shutdown()

@pytest.mark.parametrize('base_type,gripper_type',[('twist','joint_state'),('twist_stamped','float64')])
def test_optional_peripheral_ros_interfaces_only_use_mock_peers(base_type,gripper_type):
    import json
    from geometry_msgs.msg import Twist,TwistStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64,String
    from hc_teleop_recv.protocol import ControllerInput
    from hc_teleop_recv.peripherals import BUTTON_MASKS

    context=Context()
    context.init(args=[],domain_id=220+os.getpid()%8)
    executor=SingleThreadedExecutor(context=context)
    cfg=parse_config({'schema_version':1,'channels':[],
        'input':{'mode':'vrdata','publish_vrdata':False},'control':{'enabled_on_start':True},
        'chassis':{'enabled':True,'message_type':base_type},
        'grippers':[{'id':'tool','enabled':True,'command_type':gripper_type,'feedback_type':gripper_type}]})
    recv=TeleopRecvNode(config=cfg,context=context)
    peer=Node('mock_peripheral_driver',context=context)
    executor.add_node(recv)
    executor.add_node(peer)
    bases,grippers=[],[]
    base_message=Twist if base_type=='twist' else TwistStamped
    grip_message=JointState if gripper_type=='joint_state' else Float64
    peer.create_subscription(base_message,'/cmd_vel',bases.append,10)
    peer.create_subscription(grip_message,'/gripper/command',grippers.append,10)
    feedback=peer.create_publisher(grip_message,'/joint_states',qos_profile_sensor_data)
    vr=peer.create_publisher(String,'/vrdata',qos_profile_sensor_data)
    stop=peer.create_publisher(Bool,'/teleop/emergency_stop',10)
    seq=0
    def pump(duration,held=False,send_input=True):
        nonlocal seq
        until=time.monotonic()+duration
        while time.monotonic()<until:
            message=grip_message()
            if gripper_type=='joint_state':
                message.header.stamp=peer.get_clock().now().to_msg()
                message.name=['finger_joint']
                message.position=[.03]
            else:
                message.data=.03
            feedback.publish(message)
            if send_input:
                seq+=1
                frame=replace(packet(seq),left_input=ControllerInput(held_mask=BUTTON_MASKS['primary_axis_click'] if held else 0,primary_axis=(0.,1.)),
                    right_input=ControllerInput(held_mask=BUTTON_MASKS['grip_button'] if held else 0,trigger=1.))
                vr.publish(String(data=json.dumps({**frame.as_dict(), 'received_stamp_ns': peer.get_clock().now().nanoseconds})))
            for _ in range(10):
                executor.spin_once(timeout_sec=.0002)
            time.sleep(.005)
    try:
        pump(.35)
        pump(.1,True)
        assert bases and grippers
        twist=bases[-1] if base_type=='twist' else bases[-1].twist
        assert 0<twist.linear.x<=cfg.chassis.max_forward_speed
        position=grippers[-1].position[0] if gripper_type=='joint_state' else grippers[-1].data
        assert 0<=position<.03
        stop.publish(Bool(data=True))
        pump(.07,True)
        twist=bases[-1] if base_type=='twist' else bases[-1].twist
        assert twist.linear.x==0 and not recv.frontend.enabled
    finally:
        executor.remove_node(recv)
        executor.remove_node(peer)
        recv.destroy_node()
        peer.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_vr_safety_resume_and_stop_events():
    import json
    from std_msgs.msg import Bool
    from hc_teleop_recv.protocol import ControllerInput

    context = Context()
    context.init(args=[], domain_id=150 + os.getpid() % 10)
    executor = SingleThreadedExecutor(context=context)

    pose_port, discovery_port, event_port = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(3)]
    for s in (pose_port, discovery_port, event_port):
        s.bind(("127.0.0.1", 0))
    p_port, d_port, e_port = [s.getsockname()[1] for s in (pose_port, discovery_port, event_port)]
    for s in (pose_port, discovery_port):
        s.close()
    event_sock = event_port
    event_sock.setblocking(False)

    doc = document()
    doc["input"] = {
        "mode": "udp",
        "bind_host": "127.0.0.1",
        "pose_port": p_port,
        "discovery_port": d_port,
        "event_port": e_port,
        "publish_vrdata": False,
    }
    doc["control"].update(enabled_on_start=False, resume_on_a=True)
    recv = TeleopRecvNode(config=parse_config(doc), context=context)
    peer = Node("vr_test_peer", context=context)
    executor.add_node(recv)
    executor.add_node(peer)

    stop_msgs = []
    peer.create_subscription(Bool, "/teleop/emergency_stop", stop_msgs.append, 10)
    stop_pub = peer.create_publisher(Bool, "/teleop/emergency_stop", 10)

    sender_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender_sock.settimeout(0.2)

    seq = 0
    def send_pose(held_mask=0, pressed_mask=0):
        nonlocal seq
        seq += 1
        p = replace(packet(seq), right_input=ControllerInput(held_mask=held_mask, pressed_mask=pressed_mask))
        sender_sock.sendto(wire(p), ("127.0.0.1", p_port))

    def collect_vr_events(timeout=0.1, held_mask=None):
        events = []
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if held_mask is not None:
                send_pose(held_mask=held_mask)
            for _ in range(5):
                executor.spin_once(timeout_sec=0.002)
            while True:
                try:
                    data, _ = event_sock.recvfrom(65535)
                    events.append(json.loads(data.decode("utf-8")))
                except BlockingIOError:
                    break
            time.sleep(0.005)
        return events

    try:
        # 1. Send normal packet without A button
        send_pose(held_mask=0)
        events = collect_vr_events(0.05)
        assert any(e['kind'] == 'safety_stop' for e in events)
        assert any(e['kind'] == 'teleop_status' and e['payload']['state'] == 'disabled' for e in events)
        assert not recv.frontend.enabled

        # 2. Press A button (bit 0 = 1) -> triggers safety_resume
        send_pose(held_mask=1, pressed_mask=1)
        events = collect_vr_events(0.08)
        assert any(e.get("kind") == "safety_resume" for e in events)
        assert recv.frontend.enabled

        # Restart the receiver while the VR UI previously showed enabled.
        old_session = recv.vr_session_id
        executor.remove_node(recv)
        recv.destroy_node()
        recv = TeleopRecvNode(config=parse_config(doc), context=context)
        executor.add_node(recv)
        events = collect_vr_events(.3, held_mask=0)
        assert recv.vr_session_id != old_session and not recv.frontend.enabled
        assert any(e['kind'] == 'safety_stop' and e['session_id'] == recv.vr_session_id for e in events)
        assert any(e['kind'] == 'teleop_status' and e['payload']['state'] == 'disabled'
                   and e['session_id'] == recv.vr_session_id for e in events)

        send_pose(held_mask=1, pressed_mask=1)
        collect_vr_events(.08)

        # Drop a notification / reopen the VR UI while still connected. The
        # current state must arrive again without another A edge.
        events = collect_vr_events(1.15, held_mask=1)
        assert any(e['kind'] == 'safety_resume' and e['payload'].get('state_sync') for e in events)
        snapshots = [e for e in events if e['kind'] == 'teleop_status']
        assert len(snapshots) >= 6 and all(e['payload']['enabled'] for e in snapshots)
        assert all(e['session_id'] == recv.vr_session_id for e in snapshots)
        sequences = sorted(set(e['sequence'] for e in snapshots))
        assert len(sequences) >= 3

        # A rejected sender must not redirect status away from the real PICO.
        recv._accept(packet(999), ('127.0.0.2', 7777))
        assert recv.last_peer_host == '127.0.0.1'

        # While homing, even pressing A must report inhibited/disabled.
        recv._set_motion_active(SetBool.Request(data=True), SetBool.Response())
        collect_vr_events(.03, held_mask=0)
        events = collect_vr_events(.1, held_mask=1)
        assert not recv.frontend.enabled
        assert any(e['kind'] == 'teleop_status' and e['payload']['motion_active'] for e in events)
        assert not any(e['kind'] == 'safety_resume' for e in events)
        recv._set_motion_active(SetBool.Request(data=False), SetBool.Response())
        events = collect_vr_events(.3, held_mask=1)
        assert any(e['kind'] == 'teleop_status' and e['payload']['state'] == 'disabled' for e in events)

        # Re-enabling then losing input must no longer leave VR at "running".
        collect_vr_events(.03, held_mask=0)
        collect_vr_events(.08, held_mask=1)
        events = collect_vr_events(.5)
        assert any(e['kind'] == 'teleop_status' and e['payload']['state'] == 'input_timeout' for e in events)
        assert any(e['kind'] == 'safety_stop' and e['payload'].get('state') == 'input_timeout' for e in events)
        assert any(m.data is False for m in stop_msgs)

        # 3. Assert emergency stop via ROS topic -> triggers safety_stop
        stop_msgs.clear()
        stop_pub.publish(Bool(data=True))
        events = collect_vr_events(0.08)
        assert any(e.get("kind") == "safety_stop" for e in events)
        assert not recv.frontend.enabled

        # 4. Release A then press A again -> triggers safety_resume
        send_pose(held_mask=0)
        collect_vr_events(0.03)
        send_pose(held_mask=1, pressed_mask=1)
        events = collect_vr_events(0.08)
        assert any(e.get("kind") == "safety_resume" for e in events)
        assert recv.frontend.enabled
    finally:
        event_sock.close()
        sender_sock.close()
        executor.remove_node(recv)
        executor.remove_node(peer)
        recv.destroy_node()
        peer.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_udp_rejection_records_reason_without_accepting_invalid_input():
    from types import SimpleNamespace

    class BadPacketSocket:
        pending = True

        def recvmsg(self, size, ancillary_size):
            if not self.pending:
                raise BlockingIOError
            self.pending = False
            return b'bad', [], 0, ('127.0.0.1', 5005)

    from hc_teleop_recv.log_summary import LogSummary
    from hc_teleop_recv.state_log import TeleopStateLog
    warnings = []
    accepted = []
    recv = SimpleNamespace(sockets=[BadPacketSocket()], config=SimpleNamespace(source_ip=''),
        state_log=TeleopStateLog(parse_config(document())),
        rejected_packets=0, last_packet_error=None, log_summary=LogSummary(('input_rejected',)),
        _accept=lambda *args: accepted.append(args),
        get_logger=lambda: SimpleNamespace(warn=warnings.append))
    recv._reject_packet = lambda detail: TeleopRecvNode._reject_packet(recv, detail)
    TeleopRecvNode._poll_udp(recv)
    TeleopRecvNode._flush_diagnostics(recv, 0.)
    assert recv.rejected_packets == 1
    assert 'received 3' in recv.last_packet_error
    assert len(warnings) == 1
    recv.sockets[0].pending = True
    TeleopRecvNode._poll_udp(recv)
    assert recv.rejected_packets == 2
    TeleopRecvNode._flush_diagnostics(recv, 1.)
    assert len(warnings) == 1
    TeleopRecvNode._flush_diagnostics(recv, 30.)
    assert len(warnings) == 2
    assert not accepted



def test_motion_services_rebind_and_action_feedback_with_mock_peer():
    import json
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger
    from hc_teleop_recv.protocol import ControllerInput
    from test_frontend import FK
    context = Context(); context.init(args=[], domain_id=210 + os.getpid() % 10)
    executor = SingleThreadedExecutor(context=context)
    doc = document(); doc['input'] = {'mode': 'vrdata'}
    doc['adapter'] = {'robot_id': 'lab'}
    doc['actions'] = {'home_pose_id': 'right_home', 'home_gesture_enabled': True,
                      'recording_buttons_enabled': True}
    recv = TeleopRecvNode(config=parse_config(doc), context=context)
    peer = Node('fake_action_runtime', context=context)
    executor.add_node(recv); executor.add_node(peer)
    actions = []
    peer.create_subscription(String, '/hc_teleop_recv/actions', lambda m: actions.append(json.loads(m.data)), 10)
    events = peer.create_publisher(String, '/hc_teleop_recv/events', 10)
    pose_status = peer.create_publisher(String, '/motion/pose_status', 10)
    def pump():
        pose_status.publish(String(data=json.dumps({
            'robot_id': 'lab', 'configuration_sha256': recv.configuration_identity['sha256'],
            'home_pose_id': 'right_home', 'ready': True, 'state': 'idle', 'stamp_ns': time.time_ns()})))
        end = time.monotonic() + .06
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.002)
    def call(name, value=None):
        kind = Trigger if value is None else SetBool
        client = peer.create_client(kind, '/hc_teleop_recv/' + name)
        assert client.wait_for_service(timeout_sec=1)
        request = kind.Request()
        if value is not None: request.data = value
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=1)
        peer.destroy_client(client)
        return future.result()
    try:
        for _ in range(4): pump()
        recv._accept(packet(1), 'fake')
        frame = replace(packet(2, grip=1), left_input=ControllerInput(held_mask=1))
        recv._accept(frame, 'fake'); pump()
        recv._accept(replace(frame, sequence=3, vr_timestamp=1.03), 'fake'); pump()
        assert [a['action'] for a in actions] == ['record_start']
        assert actions[0]['configuration_sha256'] == recv.configuration_identity['sha256']
        assert call('set_motion_active', True).success
        assert not call('set_enabled', True).success
        assert not call('reset_reference').success
        assert not call('home').success
        recv._accept(replace(packet(4, grip=1), right_input=ControllerInput(held_mask=1, grip=1)), 'fake')
        assert not recv.frontend.enabled
        assert call('set_motion_active', False).success
        assert call('set_enabled', True).success
        assert recv.frontend.channels['arm'].fk_at == float('-inf')
        now = time.monotonic()
        recv.frontend.update_fk('arm', FK, now)
        recv._accept(packet(5, grip=1), 'fake')
        assert recv.frontend.tick(time.monotonic())['arm'] == FK
        assert call('reset_reference').success
        assert recv.frontend.enabled and not recv.frontend.tick(time.monotonic())
        assert call('home').success; pump()
        assert actions[-1]['action'] == 'home' and actions[-1]['pose_id'] == 'right_home'
        event = {'kind': 'recording_started', 'robot_id': 'lab', 'filename': 'test.mcap', 'recording': True}
        events.publish(String(data=json.dumps(event))); pump()
        assert recv.last_action == event
    finally:
        executor.shutdown(); recv.destroy_node(); peer.destroy_node(); context.shutdown()


def test_ros_callbacks_log_input_output_and_feedback_transitions_without_repetition(monkeypatch):
    import json
    from types import SimpleNamespace
    from std_msgs.msg import String
    from hc_teleop_recv import node as node_module

    context = Context()
    context.init(args=[], domain_id=210 + os.getpid() % 10)
    doc = document()
    doc['input'] = {'mode': 'vrdata'}
    recv = TeleopRecvNode(config=parse_config(doc), context=context)
    clock = [0.]
    logs = []
    monkeypatch.setattr(node_module, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    monkeypatch.setattr(recv, 'get_logger', lambda: SimpleNamespace(info=logs.append, warn=lambda _: None))

    def step(now, seq=None, frame='base'):
        clock[0] = now
        if seq is not None:
            fk = PoseStamped()
            fk.header.frame_id = frame
            fk.header.stamp = recv.get_clock().now().to_msg()
            fk.pose.orientation.w = 1.
            recv._fk_callback(recv.config.channels[0], fk)
            recv._vrdata_callback(String(data=json.dumps({**packet(seq, grip=1).as_dict(), 'received_stamp_ns': recv.get_clock().now().nanoseconds})))
        recv._tick()

    try:
        step(0)
        logs.clear()
        clock[0] = 1.
        recv._vrdata_callback(String(data='invalid json'))
        recv._tick()
        assert any('input.transport: stopped -> receiving' in m for m in logs)
        assert not any('-> publishing' in m for m in logs)
        logs.clear()
        step(2, 1)
        assert any('input.accepted: stopped -> receiving' in m for m in logs)
        assert any('arm.arm.output:' in m and '-> publishing' in m for m in logs)
        logs.clear()
        for i in range(2, 402):
            step(2 + i/100, i)
        assert logs == []
        step(7, 402, frame='wrong')
        assert any('arm.arm.fk: receiving -> unavailable' in m and 'frame mismatch' in m for m in logs)
        assert any('fresh measured FK required' in m for m in logs)
        logs.clear()
        step(8)
        assert any('input.transport: receiving -> stopped' in m for m in logs)
        assert any('input.accepted: receiving -> stopped' in m for m in logs)
        logs.clear()
        step(80)
        assert logs == []
        step(81, 403)
        assert any('-> publishing' in m for m in logs)
    finally:
        recv.destroy_node()
        context.shutdown()


def test_udp_backlog_keeps_only_last_pose_and_rejects_expired_arrivals():
    from types import SimpleNamespace
    from test_frontend import FK
    context = Context()
    context.init(args=[], domain_id=210 + os.getpid() % 10)
    pose_port, discovery_port = unused_ports()
    doc = document()
    doc['input'] = {'mode': 'udp', 'bind_host': '127.0.0.1', 'pose_port': pose_port,
                    'discovery_port': discovery_port, 'publish_vrdata': False}
    doc['control']['input_timeout'] = .15
    recv = TeleopRecvNode(config=parse_config(doc), context=context)
    recv.timer.cancel()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    targets, buttons = [], []
    recv.target_publishers['arm'] = SimpleNamespace(publish=targets.append)
    recv.buttons_publisher = SimpleNamespace(publish=buttons.append)
    try:
        recv.frontend.update_fk('arm', FK, time.monotonic())
        for seq in (0, 1):
            sock.sendto(wire(packet(seq, float(seq))), ('127.0.0.1', pose_port))
            recv._tick()
        assert len(targets) == 1
        targets.clear()
        # Button transitions in the middle must survive pose coalescing.
        for seq in range(2, 72):
            frame = packet(seq, 1., (0., 0., -.001 * seq))
            frame = replace(frame, right_input=replace(frame.right_input, held_mask=2 if seq == 10 else 0))
            sock.sendto(wire(frame), ('127.0.0.1', pose_port))
        recv._tick()
        assert recv.udp_backlog and targets == []
        recv._tick()
        assert not recv.udp_backlog and len(targets) == 1
        assert targets[0].pose.position.x == pytest.approx(.2 + .8 * .071)
        import json
        edges = [e for msg in buttons for e in json.loads(msg.data)['edges']]
        assert [(e['button'], e['action']) for e in edges] == [
            ('secondary', 'pressed'), ('secondary', 'released')]
        recv._tick()
        assert len(targets) == 1
        sock.sendto(wire(packet(72, 1.)), ('127.0.0.1', pose_port))
        time.sleep(.17)
        before = recv.received_packets
        recv._tick()
        assert recv.received_packets == before and recv.rejected_packets == 1
        assert len(targets) == 1
    finally:
        sock.close()
        recv.destroy_node()
        context.shutdown()


def test_vrdata_timestamp_is_required_preserved_and_cannot_renew_old_input():
    import json
    from std_msgs.msg import String
    from types import SimpleNamespace
    from test_frontend import FK
    context = Context()
    context.init(args=[], domain_id=210 + os.getpid() % 10)
    doc = document()
    doc['input'] = {'mode': 'vrdata', 'publish_vrdata': False}
    recv = TeleopRecvNode(config=parse_config(doc), context=context)
    recv.timer.cancel()
    targets = []
    recv.target_publishers['arm'] = SimpleNamespace(publish=targets.append)
    try:
        recv.frontend.update_fk('arm', FK, time.monotonic())
        for raw in ('[]', json.dumps(packet().as_dict()), json.dumps({
                **packet().as_dict(), 'received_stamp_ns': recv.get_clock().now().nanoseconds - 10**9})):
            recv._vrdata_callback(String(data=raw))
        assert recv.received_packets == 0 and recv.rejected_packets == 3
        stamp = recv.get_clock().now().nanoseconds - 80_000_000
        msg = String(data=json.dumps({**packet(1, 1.).as_dict(), 'received_stamp_ns': stamp}))
        recv._vrdata_callback(msg)
        recv._tick()
        assert len(targets) == 1
        output_stamp = targets[0].header.stamp.sec * 10**9 + targets[0].header.stamp.nanosec
        assert abs(output_stamp - stamp) < 1_000_000
        recv._tick()
        recv._vrdata_callback(msg)
        recv._tick()
        assert len(targets) == 1 and recv.received_packets == 1
    finally:
        recv.destroy_node()
        context.shutdown()
