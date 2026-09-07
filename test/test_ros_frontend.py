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
            raw_pub.publish(String(data=json.dumps(p.as_dict())))

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
        assert targets == []
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
        pump(.04, grip=1.)  # FK recovery with a held Grip must not rebind.
        assert targets == []
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
        assert targets == []
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
                vr.publish(String(data=json.dumps(frame.as_dict())))
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
