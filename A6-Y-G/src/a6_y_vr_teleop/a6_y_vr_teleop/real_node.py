"""Loopback Quest input and ROS adapter for physical feedback / mock interfaces."""
import json
import os
from pathlib import Path
import socket
import time
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from a6_y_teleop_msgs.msg import VrJointReference, VrControl, VrState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from .a5_kinematics import A5Kinematics, JOINTS
from .geometry_guard import GeometryGuard
from .real_bridge import RealBridge, STATE_NAMES, REASON_NAMES, STOPPING, PAUSED, FAULT
from .input_protocol import MAX_PACKET
from .commissioning import sha256

CONTROL = '/arm6/a6_y_joint_impedance_controller'
QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class RealBridgeNode(Node):
    def __init__(self):
        super().__init__('vr_bridge')
        paths = {n: self.declare_parameter(n, '').value for n in ('config_path', 'urdf_path', 'limits_path', 'record_file')}
        mode = self.declare_parameter('mode', 'readonly').value
        control = self.declare_parameter('controller_prefix', CONTROL).value
        if control not in (CONTROL, '/arm6/a6_y_vr_position_controller'):
            raise RuntimeError('unsupported controller_prefix')
        run_metadata = json.loads(self.declare_parameter('run_metadata', '{}').value)
        port = self.declare_parameter('udp_port', 15153).value
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1': raise RuntimeError('ROS_LOCALHOST_ONLY=1 required')
        if (mode == 'mock') != (port == 15154): raise RuntimeError('mock uses UDP 15154; hardware uses 15153')
        if mode != 'mock' and port != 15153: raise RuntimeError('physical UDP port must be 15153')
        cfg = yaml.safe_load(Path(paths['config_path']).read_text())
        kin = A5Kinematics(paths['urdf_path'], paths['limits_path'], cfg)
        geometry = GeometryGuard(kin, paths['urdf_path'], cfg) if cfg.get('geometry_checking', True) else None
        self.arm = RealBridge(kin, geometry, cfg, mode)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port)); self.sock.setblocking(False)
        self.record = open(paths['record_file'], 'x', buffering=1) if paths['record_file'] else None
        if self.record:
            self.record.write(json.dumps(dict(kind='metadata', mode=mode, hardware_accessed=mode != 'mock',
                clock_id=self.arm.clock, producer_id=self.arm.producer, config=cfg, run_metadata=run_metadata,
                sha256={k: sha256(paths[k]) for k in ('config_path', 'urdf_path', 'limits_path')}))+'\n')
        self.subscription = self.create_subscription(VrState, control+'/vr_state', self.observe, QOS)
        # Readonly has no publishers on either controller input topic.
        self.reference_pub = self.control_pub = None
        if mode != 'readonly':
            self.reference_pub = self.create_publisher(VrJointReference, control+'/vr_reference', QOS)
            self.control_pub = self.create_publisher(VrControl, control+'/vr_control', 10)
        self.joints = self.create_publisher(JointState, '/arm6/vr/reference_joint_states', 10)
        self.poses = self.create_publisher(PoseStamped, '/arm6/vr/target_pose', 10)
        self.markers = self.create_publisher(MarkerArray, '/arm6/vr/markers', 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/arm6/vr/status', 10)
        self.last_ns = time.monotonic_ns(); self.last_diagnostic = 0
        self.last_stop_reason = ''
        # Compute directly after taking the newest depth-one feedback sample.
        # A periodic timer in the same executor can run before a queued state
        # callback after a slow solve, adding a whole previous solve's latency.
        # The timer only diagnoses missing feedback; it never advances a fresh
        # trajectory. C++ independently expires references if this node stalls.
        self.timer = self.create_timer(1/cfg['update_rate'], self.watchdog)
        self.get_logger().info(f'VR mode={mode}, UDP={port}; initial PREPARE and ARM require local operator commands')
        if not self.arm.geometry_checking:
            self.get_logger().warning('Full position mode: CAD self-collision checking DISABLED; '
                                      'joint limits, speed limits and communication watchdogs remain active')

    def observe(self, state):
        previous = self.arm.state
        self.arm.observe(state, time.monotonic_ns())
        if self.arm.state is not state: return
        if previous is None or state.state != previous.state:
            why = REASON_NAMES[state.reason] if state.reason < len(REASON_NAMES) else str(state.reason)
            if state.state in (STOPPING, FAULT):
                self.last_stop_reason = why
                if why != 'RELEASED':
                    self.get_logger().warning(f'VR {STATE_NAMES[state.state]}: {why}; bridge={self.arm.reason}')
            elif state.state not in (PAUSED,):
                self.last_stop_reason = ''
            if why == 'AUTO_REARMED':
                self.get_logger().info('VR ARMED_HOLD: tracking recovered and grip released; press grip to follow')
        if self.record:
            self.record.write(json.dumps(dict(kind='controller', monotonic_ns=state.monotonic_ns,
                state=state.state, reason=state.reason, takeover_id=state.takeover_id,
                controller_epoch=state.controller_epoch, feedback_valid=state.feedback_valid,
                producer_id=state.producer_id, session_id=state.session_id,
                anchor_monotonic_ns=state.anchor_monotonic_ns,
                anchor_positions=list(state.anchor_positions), accepted_sequence=state.accepted_sequence,
                reference_velocities=list(state.reference_velocities),
                prepare_ramp_complete=state.prepare_ramp_complete,
                position_control=state.position_control,
                gripper_control=state.gripper_control,gripper_reference=state.gripper_reference_position,
                gripper_anchor=state.gripper_anchor_position,
                position_age=list(state.position_age), feedback_age=list(state.feedback_age),
                request_id=state.request_id, request_accepted=state.request_accepted,
                output_permitted=state.output_permitted, follow_permitted=state.follow_permitted,
                auto_rearm_after_release=state.auto_rearm_after_release,
                session_reference_travel_limit=state.session_reference_travel_limit,
                session_actual_travel_limit=state.session_actual_travel_limit,
                session_origin=list(state.session_origin_positions), saturated=list(state.saturated),
                actual=list(state.actual_positions), velocity=list(state.actual_velocities),
                reference=list(state.reference_positions), impedance=list(state.impedance_effort)))+'\n')
        self.update()

    def watchdog(self):
        if not self.arm.feedback_ok(time.monotonic_ns()):
            self.update()

    @staticmethod
    def pose(matrix, stamp):
        msg = PoseStamped(); msg.header.stamp = stamp; msg.header.frame_id = 'base_link'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, matrix[:3, 3])
        q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, q)
        return msg

    def update(self):
        start = time.monotonic_ns(); dt = (start-self.last_ns)/1e9; self.last_ns = start
        for _ in range(128):
            try: packet, _ = self.sock.recvfrom(MAX_PACKET+1)
            except BlockingIOError: break
            now = time.monotonic_ns(); self.arm.receive(packet, now)
            if self.record: self.record.write(json.dumps(dict(kind='input', receive_ns=now, packet=packet.decode(errors='replace')))+'\n')
        else: self.arm.pause('udp_queue_overload')
        now = time.monotonic_ns()
        if self.arm.stop_on_compute_overrun and (now-start)/1e9 > self.arm.cfg['compute_budget']:
            self.arm.pause('input_processing_budget_exceeded')
        reference, control = self.arm.tick(now, dt)
        callback_ns=time.monotonic_ns()-start
        if callback_ns > self.arm.cfg['compute_budget']*1e9:
            if self.record:
                self.record.write(json.dumps(dict(kind='callback_overrun',monotonic_ns=now,
                    callback_ms=callback_ns/1e6,solver_ms=self.arm.compute_s*1000,period_ms=dt*1000,
                    stop_requested=self.arm.stop_on_compute_overrun))+'\n')
            # A nominal-cycle overrun is diagnostic in full position mode.
            # Keep original timestamps: the C++ gate still rejects stale input,
            # feedback or references, independently of this Python callback.
            if self.arm.stop_on_compute_overrun:
                self.arm.pause('callback_budget_exceeded'); control = None
                if reference: reference['valid'] = False
        if reference:
            published_ns = time.monotonic_ns()
            self.reference_pub.publish(VrJointReference(**reference))
            if self.record: self.record.write(json.dumps(dict(kind='reference',
                published_monotonic_ns=published_ns, **reference))+'\n')
        if control: self.control_pub.publish(VrControl(**control))
        stamp = self.get_clock().now().to_msg()
        if self.arm.preview is not None and self.arm.feedback_ok(now):
            self.joints.publish(JointState(header=self.pose(self.arm.target, stamp).header,
                                           name=JOINTS, position=self.arm.preview.tolist()))
            self.poses.publish(self.pose(self.arm.target, stamp))
        if now-self.last_diagnostic < 100_000_000: return
        self.last_diagnostic = now
        state_name = STATE_NAMES[self.arm.state.state] if self.arm.state and self.arm.state.state < len(STATE_NAMES) else 'WAITING'
        tolerated_pairs = self.arm.geometry.last_tolerated_pairs if self.arm.geometry else []
        stop = self.last_stop_reason if self.arm.state and self.arm.state.state in (STOPPING, PAUSED, FAULT) else ''
        controller_reason = (REASON_NAMES[self.arm.state.reason]
            if self.arm.state and self.arm.state.reason < len(REASON_NAMES) else '')
        detail = f'controller={stop}; bridge={self.arm.reason}' if stop else self.arm.reason
        if controller_reason == 'TRACKING_LIMITED' and not stop:
            detail = 'controller=TRACKING_LIMITED; bridge='+self.arm.reason
        diagnostic = DiagnosticStatus(name='a5_vr_'+self.arm.mode, hardware_id=self.arm.mode,
            level=DiagnosticStatus.WARN if self.arm.latched or (stop and stop != 'RELEASED') else DiagnosticStatus.OK,
            message=state_name+': '+detail,
            values=[KeyValue(key=k, value=str(v)) for k, v in dict(compute_ms=self.arm.compute_s*1000,
                accepted=self.arm.accepted, rejected=self.arm.rejected, controller_state=state_name,
                last_stop_reason=stop,
                controller_reason=controller_reason,
                geometry_checking=self.arm.geometry_checking,
                zero_tolerance_pairs=tolerated_pairs).items()])
        if self.record:
            self.record.write(json.dumps(dict(kind='diagnostic',monotonic_ns=now,state=state_name,
                reason=self.arm.reason,controller_stop_reason=stop,controller_reason=controller_reason,latched=self.arm.latched,
                geometry_checking=self.arm.geometry_checking,compute_ms=self.arm.compute_s*1000))+'\n')
        if self.record and tolerated_pairs:
            self.record.write(json.dumps(dict(kind='zero_collision_tolerance', monotonic_ns=now,
                pairs=tolerated_pairs,
                anchor=self.arm.geometry.zero_anchor.tolist() if self.arm.geometry.zero_anchor is not None else None))+'\n')
        array = DiagnosticArray(status=[diagnostic]); array.header.stamp = stamp; self.diagnostics.publish(array)
        if self.arm.preview is not None and self.arm.target is not None:
            markers = []
            actual = self.arm.kin.fk(np.asarray(self.arm.state.actual_positions))
            for i, (pose, color) in enumerate(((actual, (0., .9, .3)), (self.arm.target, (1., .6, 0.)))):
                m = Marker(); pm = self.pose(pose, stamp); m.header = pm.header; m.pose = pm.pose
                m.ns = 'vr_tcp'; m.id = i; m.type = Marker.ARROW; m.action = Marker.ADD
                m.scale.x = .06; m.scale.y = .008; m.scale.z = .008
                m.color.r, m.color.g, m.color.b = color; m.color.a = 1.; markers.append(m)
            self.markers.publish(MarkerArray(markers=markers))

    def destroy_node(self):
        self.sock.close()
        if self.record: self.record.close()
        return super().destroy_node()


def main():
    rclpy.init(); node = None
    try:
        node = RealBridgeNode(); rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
