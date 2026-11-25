"""ROS output for virtual kinematics only; never loads a hardware interface."""
import hashlib
import json
import os
from pathlib import Path
import socket
import sys
import time
import numpy as np
import pinocchio as pin
import scipy
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from .input_protocol import MAX_PACKET
from .a5_kinematics import A5Kinematics, JOINTS
from .virtual_arm import VirtualArm

FRAME = 'vr_a5/base_link'


class VirtualArmNode(Node):
    def __init__(self):
        super().__init__('virtual_arm')
        paths = {n:self.declare_parameter(n,'').value for n in ['config_path','urdf_path','limits_path','record_file']}
        cfg = yaml.safe_load(Path(paths['config_path']).read_text())
        if self.get_namespace() != '/arm6_vr' or os.environ.get('ROS_DOMAIN_ID') != str(cfg['domain_id']):
            raise RuntimeError('Use run_vr_rviz.bash: isolated domain and /arm6_vr namespace are required')
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Virtual ROS nodes require ROS_LOCALHOST_ONLY=1')
        self.arm = VirtualArm(A5Kinematics(paths['urdf_path'],paths['limits_path'],cfg),cfg)
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1',int(cfg['udp_port'])))
        self.sock.setblocking(False)
        self.record = None
        if paths['record_file']:
            self.record = open(paths['record_file'],'x',buffering=1)
            metadata={'kind':'metadata','config':cfg,'urdf':paths['urdf_path'],
                'limits':paths['limits_path'],
                'versions':{'python':sys.version.split()[0],'pinocchio':pin.__version__,
                            'numpy':np.__version__,'scipy':scipy.__version__}}
            for name,path in [('urdf',paths['urdf_path']),('limits',paths['limits_path'])]:
                metadata[name+'_sha256']=hashlib.sha256(Path(path).read_bytes()).hexdigest()
            self.record.write(json.dumps(metadata)+'\n')
        self.joints = self.create_publisher(JointState,'joint_states',10)
        self.target_pub = self.create_publisher(PoseStamped,'target_pose',10)
        self.tcp_pub = self.create_publisher(PoseStamped,'tcp_pose',10)
        self.markers = self.create_publisher(MarkerArray,'markers',10)
        self.status = self.create_publisher(DiagnosticArray,'status',10)
        self.last_ns = time.monotonic_ns()
        self.last_diag_ns = 0
        self.tick_count = 0
        self.max_callback_ms = 0.
        self.frame_count = 0
        self.start_ns = self.last_ns
        self.timer = self.create_timer(1./cfg['update_rate'],self.update)
        # Warm linear algebra before operator input.
        self.arm.kin.step(self.arm.q,self.arm.target,1./cfg['update_rate'])
        self.get_logger().info('Virtual A5 only; loopback UDP %s; domain %s; %.0f Hz' %
            (cfg['udp_port'],cfg['domain_id'],cfg['update_rate']))

    def pose_msg(self, matrix, stamp):
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = stamp, FRAME
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z = map(float,matrix[:3,3])
        quat = Rotation.from_matrix(matrix[:3,:3]).as_quat()
        msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w = map(float,quat)
        return msg

    def update(self):
        start = time.monotonic_ns()
        # Bounded drain: process edges in receive order, never block the state loop.
        for _ in range(128):
            try:
                packet,_ = self.sock.recvfrom(MAX_PACKET+1)
            except BlockingIOError:
                break
            now = time.monotonic_ns()
            self.arm.receive(packet,now)
            self.frame_count += 1
            if self.record:
                self.record.write(json.dumps({'kind':'input','receive_ns':now,'packet':packet.decode(errors='replace')})+'\n')
        else:
            self.arm.pause('input_queue_overload')
        now = time.monotonic_ns()
        dt = (start-self.last_ns)/1e9
        self.last_ns = start
        # Excessive input processing must not commit a late command.
        if (now-start)/1e9 > self.arm.cfg['compute_budget']:
            self.arm.overruns += 1
            self.arm.pause('input_processing_budget_exceeded')
        self.arm.tick(now,dt)
        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name, msg.position, msg.velocity = JOINTS, self.arm.q.tolist(),self.arm.velocity.tolist()
        self.joints.publish(msg)
        self.target_pub.publish(self.pose_msg(self.arm.target,stamp))
        tcp = self.arm.kin.fk(self.arm.q)
        self.tcp_pub.publish(self.pose_msg(tcp,stamp))
        if self.record:
            self.record.write(json.dumps({'kind':'state','monotonic_ns':now,'dt':dt,'q':self.arm.q.tolist(),
                'state':self.arm.state,'reason':self.arm.reason})+'\n')
        self.tick_count += 1
        if now-self.last_diag_ns >= 100_000_000:
            self.last_diag_ns = now
            self.publish_diagnostics(now,stamp,tcp)
        self.max_callback_ms = max(self.max_callback_ms,(time.monotonic_ns()-start)/1e6)

    def publish_diagnostics(self, now, stamp, tcp):
        values = self.arm.diagnostics(now)
        values.update(callback_max_ms=self.max_callback_ms,
            observed_input_events_hz=self.frame_count/max((now-self.start_ns)/1e9,1e-3))
        diagnostic = DiagnosticStatus(name='virtual_a5',hardware_id='virtual_only',
            level=DiagnosticStatus.WARN if self.arm.state=='PAUSED' else DiagnosticStatus.OK,
            message=self.arm.state+': '+self.arm.reason,
            values=[KeyValue(key=k,value=str(v)) for k,v in values.items()])
        array = DiagnosticArray(status=[diagnostic]); array.header.stamp=stamp
        self.status.publish(array)
        markers=[]
        for i,(pose,color) in enumerate([(self.arm.raw_target,(1.,.3,.1)),(self.arm.target,(1.,.8,0.)),(tcp,(0.,.8,.4))]):
            marker=Marker(); marker.header.stamp=stamp; marker.header.frame_id=FRAME
            marker.ns='tcp_targets'; marker.id=i; marker.type=Marker.ARROW; marker.action=Marker.ADD
            marker.pose=self.pose_msg(pose,stamp).pose
            marker.scale.x=.06; marker.scale.y=.009; marker.scale.z=.009
            marker.color.r,marker.color.g,marker.color.b=color; marker.color.a=.9
            markers.append(marker)
        label=Marker(); label.header.stamp=stamp; label.header.frame_id=FRAME
        label.ns='status';label.id=0;label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD
        label.pose.position.z=.8;label.pose.orientation.w=1.;label.scale.z=.04
        label.color.r=1.;label.color.g=1.;label.color.b=1.;label.color.a=1.
        label.text='VIRTUAL A5  '+self.arm.state+(' LIMITED' if self.arm.limited else '')+'\n'+self.arm.reason
        markers.append(label);self.markers.publish(MarkerArray(markers=markers))

    def destroy_node(self):
        self.sock.close()
        if self.record: self.record.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node=None
    try:
        node=VirtualArmNode();rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
