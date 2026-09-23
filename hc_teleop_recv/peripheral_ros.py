"""ROS interface adapters for optional chassis and grippers."""
import math
import json
import time
import uuid

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from rclpy.qos import qos_profile_sensor_data

from .peripherals import Peripherals


class GripperActionOutput:
    def __init__(self, node, config):
        from rclpy.action import ActionClient
        from control_msgs.action import GripperCommand
        self.kind = GripperCommand
        self.config = config
        self.client = ActionClient(node, GripperCommand, config.command_topic)
        self.handle = self.pending = None
        self.desired = None
        self.sent = None
        self.canceling = False
        self.error = None
        self.generation = 0

    def ready(self):
        return self.error is None and self.client.server_is_ready()

    def command(self, position):
        self.desired = position
        if self.pending is not None or self.handle is not None:
            if self.handle is not None and self.sent is not None and abs(position-self.sent) > 1e-4:
                self.cancel()
            return
        if self.sent is not None and abs(position-self.sent) < 1e-6:
            return
        goal = self.kind.Goal()
        goal.command.position, goal.command.max_effort = float(position), float(self.config.max_effort)
        self.sent = position
        self.error = None
        self.pending = self.client.send_goal_async(goal)
        generation = self.generation
        self.pending.add_done_callback(lambda future: self.accepted(future, generation))

    def accepted(self, future, generation):
        self.pending = None
        try:
            handle = future.result()
            if not handle.accepted:
                self.error = '夹爪控制器拒绝目标'
                return
            self.handle = handle
            handle.get_result_async().add_done_callback(self.finished)
            if self.desired is None or generation != self.generation:
                self.cancel()
        except Exception as error:
            self.error = str(error)

    def cancel(self):
        if self.handle is not None and not self.canceling:
            self.canceling = True
            self.handle.cancel_goal_async().add_done_callback(self.canceled)

    def canceled(self, future):
        try:
            if not future.result().goals_canceling:
                self.error = '夹爪 Action 拒绝取消，请检查控制器状态'
        except Exception as error:
            self.error = str(error)

    def stop(self):
        self.generation += 1
        self.desired = None
        if self.handle is None and self.pending is None:
            self.sent = None
        self.cancel()

    def finished(self, future):
        successful = False
        try:
            result = future.result()
            successful = result.status == 4
            if result.status not in (4,5):
                self.error = f'夹爪 Action 结束状态：{result.status}'
        except Exception as error:
            self.error = str(error)
        self.handle = None
        self.canceling = False
        if not successful or self.desired is None:
            self.sent = None

    def close(self):
        self.stop()
        self.client.destroy()


class PeripheralROS:
    def __init__(self, node, config):
        self.node, self.config = node, config
        self.controller = Peripherals(config)
        self.base_publisher = None
        self.publishers, self.actions, self.errors = {}, {}, {}
        self.capture_publishers, self.capture_sequences, self.capture_epochs = {}, {}, {}
        self.subscriptions, self.feedback_stamps = [], {}
        if config.chassis and config.chassis.enabled:
            kind = Twist if config.chassis.message_type == 'twist' else TwistStamped
            self.base_publisher = node.create_publisher(kind,config.chassis.command_topic,10)
        for cfg in config.grippers:
            if not cfg.enabled:
                continue
            if cfg.command_type == 'gripper_action':
                try:
                    self.actions[cfg.id] = GripperActionOutput(node,cfg)
                except ImportError:
                    self.errors[cfg.id] = 'GripperCommand 接口需要安装 ROS control_msgs；当前夹爪未连接'
            else:
                kind = JointState if cfg.command_type == 'joint_state' else Float64
                self.publishers[cfg.id] = node.create_publisher(kind,cfg.command_topic,10)
                if cfg.command_type == 'joint_state' and cfg.command_topic not in self.capture_publishers:
                    self.capture_publishers[cfg.command_topic] = node.create_publisher(
                        String,cfg.command_topic+'/capture_evidence',10)
                    self.capture_sequences[cfg.command_topic] = 0
                    self.capture_epochs[cfg.command_topic] = uuid.uuid4().hex
            kind = JointState if cfg.feedback_type == 'joint_state' else Float64
            self.subscriptions.append(node.create_subscription(kind,cfg.feedback_topic,
                lambda message,cfg=cfg:self.feedback(cfg,message),qos_profile_sensor_data))

    def feedback(self, cfg, message):
        if cfg.feedback_type == 'joint_state':
            stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            age = (self.node.get_clock().now().nanoseconds-stamp)/1e9
            if stamp <= self.feedback_stamps.get(cfg.id,0) or not -.05 <= age <= cfg.feedback_timeout:
                return
            if message.name.count(cfg.joint_name) != 1:
                return
            index = message.name.index(cfg.joint_name)
            if index >= len(message.position):
                return
            position = message.position[index]
            self.feedback_stamps[cfg.id] = stamp
        else:
            position = message.data
        self.controller.feedback(cfg.id,position,time.monotonic())

    def reset(self):
        for cfg in self.config.grippers:
            if self.controller.grippers[cfg.id].state == 'active':
                self._capture_event(cfg,'rebind')
        self.controller.reset()
        for action in self.actions.values():
            action.stop()

    def publish_base(self, velocity):
        if self.base_publisher is None:
            return
        twist = Twist()
        twist.linear.x,twist.linear.y,twist.angular.z = (float(v) for v in velocity)
        if self.config.chassis.message_type == 'twist_stamped':
            message = TwistStamped()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = self.config.chassis.frame_id
            message.twist = twist
        else:
            message = twist
        self.base_publisher.publish(message)

    def _capture_emit(self,cfg,kind,stamp,names,**data):
        publisher=self.capture_publishers.get(cfg.command_topic)
        if publisher is None:
            return
        self.capture_sequences[cfg.command_topic]+=1
        payload={'schema':'openarm-action-evidence/v1','kind':kind,
                 'sequence':self.capture_sequences[cfg.command_topic],
                 'source_epoch':self.capture_epochs[cfg.command_topic],
                 'stamp_ns':stamp,'names':names,**data}
        publisher.publish(String(data=json.dumps(payload,separators=(',',':'))))

    def _capture_event(self,cfg,reason):
        if cfg.command_topic not in self.capture_publishers:
            return
        self.capture_epochs[cfg.command_topic]=uuid.uuid4().hex
        self._capture_emit(cfg,'event',self.node.get_clock().now().nanoseconds,
                           [cfg.joint_name],reason=reason)

    def publish_gripper(self, cfg, position, *, capture_valid=True):
        if cfg.id in self.actions:
            self.actions[cfg.id].command(position)
        elif cfg.id in self.publishers:
            if cfg.command_type == 'joint_state':
                message = JointState()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.name = [cfg.joint_name]
                message.position = [float(position)]
                message.effort = [float(cfg.max_effort)]
            else:
                message = Float64(data=float(position))
            self.publishers[cfg.id].publish(message)
            if capture_valid and cfg.command_type == 'joint_state':
                stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
                self._capture_emit(cfg,'command',stamp,message.name,
                                   positions=list(message.position))

    def tick(self, frontend, now, input_ready=True):
        ready = {'chassis':self.base_publisher is not None and self.base_publisher.get_subscription_count()>0}
        ready.update({cfg.id:self.actions[cfg.id].ready() if cfg.id in self.actions else (
            cfg.id in self.publishers and self.publishers[cfg.id].get_subscription_count()>0) for cfg in self.config.grippers})
        output = self.controller.tick(frontend,now,ready,input_ready=input_ready)
        if output['chassis'] is not None:
            self.publish_base(output['chassis'])
        for cfg in self.config.grippers:
            if cfg.id in output['stop_grippers']:
                state = self.controller.grippers[cfg.id]
                reason = ('pause' if not frontend.enabled else
                          'clutch_release' if state.reason == '使能按键已松开' else
                          'input_loss')
                self._capture_event(cfg,reason)
                if cfg.id in self.actions:
                    self.actions[cfg.id].stop()
                else:
                    if state.feedback is not None and now-state.feedback_at <= cfg.feedback_timeout:
                        self.publish_gripper(cfg,state.feedback,capture_valid=False)
            elif cfg.id in output['grippers']:
                self.publish_gripper(cfg,output['grippers'][cfg.id])

    def status(self):
        status = self.controller.status()
        for ident, entry in status['grippers'].items():
            error = self.errors.get(ident) or (self.actions[ident].error if ident in self.actions else None)
            if error:
                entry.update(interface_error=error,state='interface_error',reason=error)
        return status

    def close(self):
        if self.node.context.ok():
            self.publish_base((0.,0.,0.))
        for action in self.actions.values():
            action.close()
