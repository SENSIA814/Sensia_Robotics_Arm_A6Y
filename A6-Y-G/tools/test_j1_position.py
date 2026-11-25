#!/usr/bin/env python3
"""Bounded J1-only position-response experiment, with no VR or IK input.

Only this dedicated controller accepts synthetic references (live=False).
The production VR configuration is never changed. Non-J1 motors are read-only.
"""
import argparse
import copy
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from a6_y_teleop_msgs.msg import VrState, VrControl, VrJointReference
from control_msgs.msg import DynamicJointState
import xacro
import yaml


NAME = 'a6_y_vr_position_controller'
PREFIX = '/j1_test/'+NAME
JOINTS = [f'joint{i}' for i in range(1, 7)]+['gripper_finger_joint1']
QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def smooth_position(start, end, elapsed, duration):
    u = min(1., max(0., elapsed/duration))
    return start+(end-start)*(10*u**3-15*u**4+6*u**5)


def validate_motion(delta_degrees, duration):
    if not math.isfinite(delta_degrees) or not 0 < abs(delta_degrees) <= 30.:
        raise ValueError('relative J1 travel must be nonzero and at most 30 degrees')
    if not math.isfinite(duration) or duration < 2.:
        raise ValueError('each move must last at least 2 seconds')
    # Derivatives of the quintic profile; never depend on a later limiter to
    # make a faster requested experiment valid.
    delta = abs(math.radians(delta_degrees))
    if 1.875*delta/duration > 1. or 5.774*delta/duration**2 > 3.:
        raise ValueError('planned trajectory exceeds full-profile speed/acceleration')


def make_configuration(directory, mode, channel, *, namespace='j1_test',
                       commanded_joints=('joint1',), mock_positions=None):
    share = Path(get_package_share_directory('a6_y_controllers'))
    urdf = Path(get_package_share_directory('a5_description'))/'urdf/A5.urdf'
    source = yaml.safe_load((share/'config/a5_vr_position_full_controllers.yaml').read_text())
    params = copy.deepcopy(source['/arm6/'+NAME]['ros__parameters'])
    params.update(urdf_path=str(urdf), vr_allow_prepare=True, vr_allow_arm=True,
                  vr_allow_test_input=True, vr_auto_rearm_after_release=False,
                  vr_gripper_control=False)
    motors = yaml.safe_load((share/'config/a5_vr_position_full_motors.yaml').read_text())
    for name in JOINTS:
        motors['joints'][name]['read_only'] = name not in commanded_joints
    motor_file = directory/'motors.yaml'
    motor_file.write_text(yaml.safe_dump(motors))
    description = xacro.process_file(str(share/'config/a5.urdf.xacro'), mappings=dict(
        model_file=str(urdf), motor_config_file=str(motor_file), control_mode='pos_vel',
        dry_run='true' if mode == 'mock' else 'false', auto_enable='true',
        can_channel='/dev/THIS_IS_A_MOCK_NO_CAN' if mode == 'mock' else channel,
        command_timeout='0.05', feedback_timeout='0.5')).toxml()
    root = ET.fromstring(description)
    # Read-only motors do not return command feedback. Poll them often enough
    # to satisfy the unchanged controller's 50 ms feedback check.
    root.find("ros2_control/hardware/param[@name='status_request_period']").text = '0.016'
    if mode == 'mock':
        for name, value in zip(JOINTS, mock_positions if mock_positions is not None else [0., 1., 1., 0., .4, 0., -.015]):
            root.find(f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param").text = str(value)
    manager = copy.deepcopy(source['/arm6/controller_manager']['ros__parameters'])
    manager['robot_description'] = ET.tostring(root, encoding='unicode')
    data = {f'/{namespace}/controller_manager': {'ros__parameters': manager},
            f'/{namespace}/{NAME}': {'ros__parameters': params}}
    parameter_file = directory/'controller.yaml'
    parameter_file.write_text(yaml.safe_dump(data))
    return parameter_file, params, motors


class Experiment:
    def __init__(self, node, mode, record, params, manager, namespace='j1_test'):
        self.node, self.mode, self.record, self.params, self.manager = node, mode, record, params, manager
        self.state = None
        self.hardware, self.hardware_ns = {}, 0
        self.anchor = self.goal = self.trajectory = None
        self.phase, self.held, self.edge = 'initial', False, 0
        self.producer, self.session = secrets.randbits(63), secrets.randbits(63)
        self.sequence, self.abort, self.active, self.enabled_once = 0, '', False, False
        self.epoch = None
        self.samples, self.last_print = [], 0.
        prefix = f'/{namespace}/{NAME}'
        self.reference_pub = node.create_publisher(VrJointReference, prefix+'/vr_reference', QOS)
        self.control_pub = node.create_publisher(VrControl, prefix+'/vr_control', 10)
        node.create_subscription(VrState, prefix+'/vr_state', self.observe, QOS)
        node.create_subscription(DynamicJointState, f'/{namespace}/dynamic_joint_states', self.observe_hw, QOS)

    def observe_hw(self, msg):
        self.hardware = {n: dict(zip(row.interface_names, row.values))
                         for n, row in zip(msg.joint_names, msg.interface_values)}
        self.hardware_ns = time.monotonic_ns()
        if self.mode == 'real' and any(self.hardware.get(n, {}).get('enabled', 0.) != 0. for n in JOINTS[1:]):
            self.abort = 'a non-J1 motor is enabled'

    def observe(self, s):
        now = time.monotonic_ns()
        if self.state is not None and s.monotonic_ns <= self.state.monotonic_ns:
            return
        self.state = s
        if self.epoch is not None and s.controller_epoch != self.epoch:
            self.abort = 'controller restarted'
        if self.active:
            if s.state in (VrState.STOPPING, VrState.PAUSED, VrState.FAULT, VrState.DISABLED):
                self.abort = f'unexpected controller state={s.state} reason={s.reason}'
            if not s.feedback_valid:
                self.abort = 'feedback invalid'
            if self.anchor is not None:
                if abs(s.actual_positions[0]-self.anchor[0]) > math.radians(30.5):
                    self.abort = 'J1 actual travel exceeded experiment envelope'
                if max(abs(s.actual_positions[i]-self.anchor[i]) for i in range(1, 6)) > math.radians(2.):
                    self.abort = 'a read-only arm joint moved more than 2 degrees'
        if self.anchor is None or self.abort or now-s.monotonic_ns > 50_000_000:
            return
        plan = self.goal
        if self.trajectory:
            start, end, began, duration = self.trajectory
            plan = smooth_position(start, end, time.monotonic()-began, duration)
        positions = list(self.anchor[:6]); positions[0] = plan
        # The first accepted reference must match the controller's captured pose.
        if s.state == VrState.TELEOP and s.accepted_sequence == 0:
            positions = list(s.anchor_positions)
        self.sequence += 1
        self.reference_pub.publish(VrJointReference(
            clock_id=s.clock_id, controller_epoch=s.controller_epoch, takeover_id=s.takeover_id,
            producer_id=self.producer, session_id=self.session, sequence=self.sequence,
            pose_sequence=self.sequence, clutch_epoch=self.edge,
            generated_monotonic_ns=now, pose_monotonic_ns=now, buttons_monotonic_ns=now,
            feedback_monotonic_ns=s.monotonic_ns, valid=True, squeeze=self.held, live=False,
            positions=positions, gripper_control=False, gripper_position=float(self.anchor[6])))
        row = dict(t_ns=s.monotonic_ns, phase=self.phase, plan=plan,
                   reference=s.reference_positions[0], actual=s.actual_positions[0],
                   reference_velocity=s.reference_velocities[0], actual_velocity=s.actual_velocities[0],
                   actual_all=list(s.actual_positions), reference_all=list(s.reference_positions),
                   position_age=list(s.position_age), feedback_age=list(s.feedback_age),
                   state=s.state, reason=s.reason,
                   j1_hardware=dict(self.hardware.get('joint1', {})),
                   enabled={n: self.hardware.get(n, {}).get('enabled') for n in JOINTS})
        self.samples.append(row); self.record.write(json.dumps(row)+'\n')
        if time.monotonic()-self.last_print >= 2.:
            self.last_print = time.monotonic()
            print(f'{self.phase}: plan={math.degrees(plan-self.anchor[0]):.2f} deg, '
                  f'sent={math.degrees(row["reference"]-self.anchor[0]):.2f}, '
                  f'actual={math.degrees(row["actual"]-self.anchor[0]):.2f}, reason={s.reason}', flush=True)

    def check(self):
        if self.abort: raise RuntimeError(self.abort)
        if self.manager.poll() is not None: raise RuntimeError('controller manager exited')
        if self.active and (self.state is None or time.monotonic_ns()-self.state.monotonic_ns > 50_000_000):
            raise RuntimeError('controller feedback expired')
        if self.active and time.monotonic_ns()-self.hardware_ns > 150_000_000:
            raise RuntimeError('hardware diagnostics expired')

    def wait(self, predicate, timeout, description):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.004)
            self.check()
            if predicate(): return
        raise RuntimeError('timeout: '+description)

    def pump(self, duration):
        end = time.monotonic()+duration
        self.wait(lambda: time.monotonic() >= end, duration+1., self.phase)

    def command(self, action):
        s = self.state; now = time.monotonic_ns()
        self.control_pub.publish(VrControl(action=action, controller_epoch=s.controller_epoch,
            request_id=now, generated_monotonic_ns=now, producer_id=self.producer,
            session_id=self.session, clutch_epoch=self.edge))
        self.wait(lambda: self.state.request_id == now, 2., f'command {action} acknowledgement')
        if not self.state.request_accepted: raise RuntimeError(f'command {action} rejected')

    def settle(self, goal, timeout):
        self.goal, self.trajectory = goal, None
        stable = None
        def reached():
            nonlocal stable
            s = self.state
            good = (abs(s.actual_positions[0]-goal) < math.radians(.5) and
                    abs(s.reference_positions[0]-goal) < math.radians(.1) and
                    abs(s.actual_velocities[0]) < .03)
            if not good: stable = None
            elif stable is None: stable = time.monotonic()
            return stable is not None and time.monotonic()-stable >= .3
        self.wait(reached, timeout, 'settle at goal')

    def run(self, delta, duration, settle_timeout):
        self.wait(lambda: self.state is not None and self.state.feedback_valid and
                  len(self.hardware) == 7, 8., 'initial feedback')
        s = self.state
        if s.state != VrState.DISABLED or not s.position_control:
            raise RuntimeError('expected a disabled native position controller')
        if any(self.hardware[n]['dry_run'] != (1. if self.mode == 'mock' else 0.) for n in JOINTS):
            raise RuntimeError('mock/physical hardware mismatch')
        if any(self.hardware[n]['enabled'] != 0. for n in JOINTS):
            raise RuntimeError('motors already enabled')
        self.epoch = s.controller_epoch
        self.anchor = list(s.actual_positions); self.goal = self.anchor[0]
        endpoint = self.goal+delta
        if not self.params['vr_lower'][0] <= endpoint <= self.params['vr_upper'][0]:
            raise RuntimeError('requested relative J1 endpoint outside joint range')
        if max(abs(v) for v in s.actual_velocities[:6]) > .03:
            raise RuntimeError('initial arm is not stationary')
        self.pump(.25)
        if self.node.count_publishers(PREFIX+'/vr_reference') != 1:
            raise RuntimeError('another reference producer is present')
        self.phase = 'prepare'; self.command(VrControl.PREPARE); self.enabled_once = True
        self.wait(lambda: self.state.prepare_ramp_complete, 3., 'prepare ramp')
        if self.mode == 'real':
            self.wait(lambda: self.hardware['joint1']['enabled'] == 1., 2., 'J1 enable')
        self.command(VrControl.ARM)
        self.held = True; self.edge += 1; self.pump(.05)
        self.command(VrControl.CAPTURE)
        self.active = True
        # Match capture exactly, then start the timed experiment.
        self.anchor = list(self.state.anchor_positions)+[self.anchor[6]]
        self.goal = self.anchor[0]; endpoint = self.goal+delta
        if not self.params['vr_lower'][0] <= endpoint <= self.params['vr_upper'][0]:
            raise RuntimeError('captured endpoint outside joint range')
        self.wait(lambda: self.state.state == VrState.TELEOP and self.state.accepted_sequence > 0,
                  2., 'first captured reference')
        self.phase = 'baseline'; self.pump(.5)
        for phase, start, end in [('outbound', self.anchor[0], endpoint),
                                  ('return', endpoint, self.anchor[0])]:
            self.phase = phase
            self.trajectory = (start, end, time.monotonic(), duration)
            self.pump(duration)
            self.phase = phase+'_settle'; self.settle(end, settle_timeout)
            self.phase = phase+'_hold'; self.pump(2.)

    def disarm(self):
        self.active = False; self.abort = ''; self.trajectory = None; self.phase = 'disarm'
        if self.state is None: return False
        self.command(VrControl.DISARM)
        self.wait(lambda: self.state.state == VrState.DISABLED and
                  bool(self.hardware) and all(self.hardware[n]['enabled'] == 0. for n in JOINTS),
                  3., 'motor disable feedback')
        self.record.write(json.dumps(dict(kind='disarm_confirmed', monotonic_ns=time.monotonic_ns(),
                                         hardware=self.hardware))+'\n')
        return True


def write_analysis(directory, samples, anchor):
    if not samples or anchor is None: return {}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    active = [r for r in samples if r['phase'] not in ('initial', 'prepare', 'disarm')]
    if not active: return {}
    t = np.array([(r['t_ns']-active[0]['t_ns'])*1e-9 for r in active])
    plan, reference, actual = [np.array([r[k] for r in active]) for k in ('plan','reference','actual')]
    with (directory/'trace.csv').open('w') as f:
        writer = csv.writer(f); writer.writerow(['time_s','phase','plan_deg','sent_deg','actual_deg','velocity_deg_s','reason'])
        for r, ti in zip(active, t):
            writer.writerow([ti,r['phase'],*[math.degrees(r[k]-anchor[0]) for k in ('plan','reference','actual')],
                             math.degrees(r['actual_velocity']),r['reason']])
    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for label, data in [('Plan',plan),('Sent to motor',reference),('Encoder',actual)]:
        ax[0].plot(t,np.rad2deg(data-anchor[0]),label=label)
    ax[0].set_ylabel('Relative J1 angle (deg)');ax[0].legend();ax[0].grid(alpha=.3)
    ax[1].plot(t,np.rad2deg(plan-reference),label='Plan - sent')
    ax[1].plot(t,np.rad2deg(reference-actual),label='Sent - encoder')
    ax[1].set_ylabel('Difference (deg)');ax[1].set_xlabel('Time (s)');ax[1].legend();ax[1].grid(alpha=.3)
    fig.tight_layout();fig.savefig(directory/'tracking.png',dpi=160);plt.close(fig)
    return dict(samples=len(active),duration_s=float(t[-1]),
        actual_travel_from_start_deg=float(np.max(np.abs(np.rad2deg(actual-anchor[0])))),
        final_error_from_start_deg=float(math.degrees(actual[-1]-anchor[0])),
        max_plan_to_sent_deg=float(np.max(np.abs(np.rad2deg(plan-reference)))),
        max_sent_to_actual_deg=float(np.max(np.abs(np.rad2deg(reference-actual)))),
        tracking_limited_fraction=sum(r['reason']==26 for r in active)/len(active),
        peak_actual_speed_deg_s=max(abs(math.degrees(r['actual_velocity'])) for r in active),
        max_other_joint_motion_deg=max(math.degrees(abs(r['actual_all'][i]-anchor[i])) for r in active for i in range(1,6)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('mock','real'), default='mock')
    parser.add_argument('--degrees',type=float,default=30.)
    parser.add_argument('--move-seconds',type=float,default=12.)
    parser.add_argument('--settle-timeout',type=float,default=30.)
    parser.add_argument('--can-channel',default='/dev/ttyACM0')
    parser.add_argument('--output',type=Path)
    args = parser.parse_args();validate_motion(args.degrees,args.move_seconds)
    if not math.isfinite(args.settle_timeout) or not 1 <= args.settle_timeout <= 60:
        parser.error('settle timeout must be 1..60 seconds')
    domain = '86' if args.mode == 'mock' else os.environ.get('A6Y_REAL_ROS_DOMAIN_ID','0')
    if os.environ.get('ROS_DOMAIN_ID') != domain or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('use scripts/run_j1_position_test.bash to select the ROS domain')
    directory = (args.output or Path('validation/j1_position')/f'{time.time_ns()}_{args.mode}').resolve()
    directory.mkdir(parents=True,exist_ok=False)
    report = dict(mode=args.mode,delta_degrees=args.degrees,move_seconds=args.move_seconds,
                  planned_peak_speed_deg_s=1.875*abs(args.degrees)/args.move_seconds,
                  hardware_accessed=False,passed=False,disarm_confirmed=False)
    manager = node = experiment = None; locks = []
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        lock = open(f'/tmp/a6y-vr-domain-{domain}.lock','a'); locks.append(lock)
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if args.mode == 'real':
            channel = str(Path(args.can_channel).resolve(strict=True))
            key = hashlib.sha256(channel.encode()).hexdigest()[:16]
            lock = open('/tmp/a5-vr-serial-'+key+'.lock','a');locks.append(lock)
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if subprocess.run(['fuser',channel],capture_output=True,timeout=3).returncode == 0:
                raise RuntimeError('CAN serial port is already in use')
        node = rclpy.create_node('j1_position_experiment')
        end = time.monotonic()+1.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if any(name=='controller_manager' for name,_ in node.get_node_names_and_namespaces()):
            raise RuntimeError('existing controller manager in this ROS domain')
        parameters, params, motors = make_configuration(directory,args.mode,args.can_channel)
        report.update(controller_parameters=params,motors=motors)
        with (directory/'controller.log').open('w') as log, (directory/'trace.jsonl').open('w',buffering=1) as record:
            manager = subprocess.Popen(['ros2','run','controller_manager','ros2_control_node','--ros-args',
                '-r','__ns:=/j1_test','--params-file',str(parameters)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            report['hardware_accessed'] = args.mode == 'real'
            for name in ('joint_state_broadcaster',NAME):
                subprocess.run(['ros2','run','controller_manager','spawner',name,'--controller-manager',
                    '/j1_test/controller_manager','--param-file',str(parameters)],stdout=log,stderr=subprocess.STDOUT,
                    timeout=25,check=True)
            experiment = Experiment(node,args.mode,record,params,manager)
            try:
                experiment.run(math.radians(args.degrees),args.move_seconds,args.settle_timeout)
                report['passed'] = True
            finally:
                try:report['disarm_confirmed'] = experiment.disarm()
                except Exception as error:report['disarm_error'] = str(error)
    except (Exception,KeyboardInterrupt) as error:
        report['error'] = str(error) or type(error).__name__
        print('Test stopped:',report['error'],flush=True)
    finally:
        if manager is not None and manager.poll() is None:
            os.killpg(manager.pid,signal.SIGINT)
            try:manager.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                os.killpg(manager.pid,signal.SIGTERM);manager.wait(timeout=5.)
        if manager is not None:report['manager_exit_code'] = manager.returncode
        if experiment is not None:
            report['anchor_positions'] = experiment.anchor
            report['final_hardware'] = experiment.hardware
            if experiment.state is not None:
                report['final_controller_state'] = experiment.state.state
                report['final_controller_reason'] = experiment.state.reason
            try:report['metrics'] = write_analysis(directory,experiment.samples,experiment.anchor)
            except Exception as error:report['analysis_error'] = str(error)
        report['passed'] = report['passed'] and report['disarm_confirmed']
        if node:node.destroy_node()
        rclpy.shutdown()
        for lock in locks:lock.close()
        (directory/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('motors','controller_parameters')},indent=2),flush=True)
        print('Result:',directory,flush=True)
    return 0 if report['passed'] else 2


if __name__ == '__main__':raise SystemExit(main())
