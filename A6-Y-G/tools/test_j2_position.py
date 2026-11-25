#!/usr/bin/env python3
"""Bounded J2 native-position trial with the other five arm joints holding.

The raw poses and sweep are operator-confirmed for this installation. Capture
current feedback, home slowly, test only J2, return home, then disable all motors.
Gripper is always read-only. No parameters or Flash are written by this tool.
"""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from a6_y_teleop_msgs.msg import VrState, VrControl, VrJointReference
import test_j1_position as base

NAME, JOINTS = base.NAME, base.JOINTS
PREFIX = '/j2_test/'+NAME
RAW_HOME = [-.020027, -.004005, .558289, .567826, -.058556, .000191, .011313]
J2_MAX = 1.801137
EFFORT_LIMITS = [10., 20., 20., 3., 4., 4.]


def home_from_motors(motors):
    q = [m['sign']*raw*m.get('position_scale', 1.)+m.get('offset', 0.)
         for raw, m in zip(RAW_HOME, (motors['joints'][n] for n in JOINTS))]
    # Operator's raw -0.23 deg is inside the existing zero tolerance. Command
    # nominal zero without changing the encoder calibration or its offset.
    q[1] = 0.
    return np.array(q[:6])


def validate_motion(start_degrees, end_degrees, duration):
    if not all(math.isfinite(v) for v in (start_degrees, end_degrees, duration)):
        raise ValueError('finite trajectory values required')
    if not 0. <= start_degrees < end_degrees <= math.degrees(J2_MAX):
        raise ValueError('J2 test endpoints must be ordered within operator-confirmed 0..103.19755 degrees')
    if duration < 2. or 1.875*math.radians(end_degrees-start_degrees)/duration > 1.:
        raise ValueError('trajectory speed or duration outside experiment limits')
    if 5.774*math.radians(end_degrees-start_degrees)/duration**2 > 3.:
        raise ValueError('trajectory acceleration outside experiment limits')


def transfer_duration(start, end):
    # Slow homing and moving between test windows: peak <= 2.5 degrees/s.
    return max(3., 1.875*float(np.max(np.abs(end-start)))/math.radians(2.5))


def make_configuration(directory, mode, channel):
    return base.make_configuration(directory, mode, channel, namespace='j2_test',
        commanded_joints=JOINTS[:6], mock_positions=[.02575, -.003624, .00515, -.298505, .034905, .007057, .00027])


class Experiment(base.Experiment):
    def __init__(self, node, mode, record, params, manager, home):
        super().__init__(node, mode, record, params, manager, namespace='j2_test')
        self.home = home
        self.test_started = False
        self.capture = None

    def observe_hw(self, msg):
        self.hardware = {n: dict(zip(row.interface_names, row.values))
                         for n, row in zip(msg.joint_names, msg.interface_values)}
        self.hardware_ns = time.monotonic_ns()
        if self.mode == 'real' and self.hardware.get(JOINTS[6], {}).get('enabled', 0.) != 0.:
            self.abort = 'read-only gripper unexpectedly enabled'
        if self.active:
            for i, n in enumerate(JOINTS[:6]):
                h = self.hardware.get(n, {})
                effort = h.get('effort', float('nan'))
                if not math.isfinite(effort) or abs(effort) > EFFORT_LIMITS[i]:
                    self.abort = f'{n} feedback effort invalid or exceeds {EFFORT_LIMITS[i]} Nm'
                if self.mode == 'real' and h.get('enabled') != 1.:
                    self.abort = f'{n} unexpectedly disabled'

    def observe(self, s):
        now = time.monotonic_ns()
        if self.state is not None and s.monotonic_ns <= self.state.monotonic_ns:
            return
        self.state = s
        if self.epoch is not None and s.controller_epoch != self.epoch:
            self.abort = 'controller restarted'
        if self.active:
            if s.state != VrState.TELEOP or not s.feedback_valid:
                self.abort = f'controller state={s.state} reason={s.reason} feedback={s.feedback_valid}'
            if not -.034906586 <= s.actual_positions[1] <= J2_MAX+math.radians(.5):
                self.abort = 'J2 left the confirmed sweep'
            if self.test_started and max(abs(s.actual_positions[i]-self.home[i]) for i in (0,2,3,4,5)) > math.radians(2.):
                self.abort = 'a holding arm joint moved more than 2 degrees from home'
        if self.anchor is None or self.abort or now-s.monotonic_ns > 50_000_000:
            return
        plan = self.goal.copy()
        if self.trajectory is not None:
            start, end, began, duration = self.trajectory
            plan = base.smooth_position(start, end, time.monotonic()-began, duration)
        positions = list(plan)
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
        row = dict(t_ns=s.monotonic_ns, received_monotonic_ns=now,
            hardware_received_monotonic_ns=self.hardware_ns, phase=self.phase, plan=float(plan[1]),
            plan_all=list(plan), reference=s.reference_positions[1], actual=s.actual_positions[1],
            actual_velocity=s.actual_velocities[1], actual_all=list(s.actual_positions),
            reference_all=list(s.reference_positions), reference_velocity=s.reference_velocities[1],
            position_age=list(s.position_age), feedback_age=list(s.feedback_age),
            state=s.state, reason=s.reason, hardware=self.hardware,
            j2_hardware=dict(self.hardware.get('joint2', {})))
        self.samples.append(row); self.record.write(json.dumps(row)+'\n')
        if time.monotonic()-self.last_print >= 2.:
            self.last_print = time.monotonic()
            print(f'{self.phase}: J2 plan={math.degrees(plan[1]):.2f}, '
                  f'sent={math.degrees(row["reference"]):.2f}, actual={math.degrees(row["actual"]):.2f} deg; '
                  f'effort={row["j2_hardware"].get("effort", 0.):.2f} Nm; reason={s.reason}', flush=True)

    def check(self):
        try:
            super().check()
        except RuntimeError as error:
            now=time.monotonic_ns()
            self.record.write(json.dumps(dict(kind='abort_diagnostics',error=str(error),
                monotonic_ns=now,phase=self.phase,
                state_age_ms=(now-self.state.monotonic_ns)/1e6 if self.state is not None else None,
                hardware_age_ms=(now-self.hardware_ns)/1e6,hardware=self.hardware))+'\n')
            raise

    def settle(self, goal, timeout):
        self.goal, self.trajectory = goal.copy(), None
        stable = None
        def reached():
            nonlocal stable
            s = self.state
            good = (max(abs(s.actual_positions[i]-goal[i]) for i in range(6)) < math.radians(.5) and
                    max(abs(s.reference_positions[i]-goal[i]) for i in range(6)) < math.radians(.1) and
                    max(abs(v) for v in s.actual_velocities[:6]) < .03)
            if not good: stable = None
            elif stable is None: stable = time.monotonic()
            return stable is not None and time.monotonic()-stable >= .3
        self.wait(reached, timeout, 'all six joints settled')

    def move(self, phase, start, end, duration, settle_timeout):
        self.phase = phase
        self.trajectory = (start.copy(), end.copy(), time.monotonic(), duration)
        self.pump(duration)
        self.phase = phase+'_settle'; self.settle(end, settle_timeout)
        self.phase = phase+'_hold'; self.pump(2.)

    def run(self, start_angle, end_angle, duration, settle_timeout):
        self.wait(lambda: self.state is not None and self.state.feedback_valid and len(self.hardware)==7, 8., 'initial feedback')
        s = self.state
        if s.state != VrState.DISABLED or not s.position_control:
            raise RuntimeError('expected disabled position controller')
        if any(self.hardware[n]['dry_run'] != (1. if self.mode=='mock' else 0.) for n in JOINTS):
            raise RuntimeError('mock/physical mismatch')
        if any(self.hardware[n]['enabled'] != 0. for n in JOINTS):
            raise RuntimeError('motors already enabled')
        if max(abs(v) for v in s.actual_velocities[:6]) > .03:
            raise RuntimeError('initial arm is not stationary')
        if not -.034906586 <= s.actual_positions[1] <= J2_MAX+math.radians(.5):
            raise RuntimeError('initial J2 outside operator-confirmed sweep')
        self.epoch = s.controller_epoch
        self.anchor = list(s.actual_positions); self.goal = np.array(self.anchor[:6])
        self.pump(.25)
        if self.node.count_publishers(PREFIX+'/vr_reference') != 1:
            raise RuntimeError('another reference producer present')
        self.phase = 'prepare'; self.command(VrControl.PREPARE); self.enabled_once = True
        self.wait(lambda: self.state.prepare_ramp_complete, 3., 'prepare ramp')
        if self.mode == 'real':
            self.wait(lambda: all(self.hardware[n]['enabled']==1. for n in JOINTS[:6]), 2., 'six arm motors enabled')
        self.command(VrControl.ARM)
        self.held = True; self.edge += 1; self.pump(.05)
        self.command(VrControl.CAPTURE)
        self.anchor = list(self.state.anchor_positions)+[self.anchor[6]]
        self.capture = list(self.anchor)
        self.goal = np.array(self.anchor[:6]); self.active = True
        self.wait(lambda: self.state.state==VrState.TELEOP and self.state.accepted_sequence>0, 2., 'captured reference')
        self.phase='captured_hold'; self.pump(1.)
        self.move('homing', self.goal, self.home, transfer_duration(self.goal,self.home), settle_timeout)
        self.test_started = True
        start, end = self.home.copy(), self.home.copy()
        start[1], end[1] = start_angle, end_angle
        self.move('to_window', self.home, start, transfer_duration(self.home,start), settle_timeout)
        self.move('outbound', start, end, duration, settle_timeout)
        self.move('return', end, start, duration, settle_timeout)
        self.move('home_return', start, self.home, transfer_duration(start,self.home), settle_timeout)


def write_analysis(directory, samples, home):
    if not samples: return {}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = [r for r in samples if r['phase'] in ('outbound','outbound_settle','outbound_hold','return','return_settle','return_hold')]
    if not rows: return {}
    t=np.array([(r['t_ns']-rows[0]['t_ns'])*1e-9 for r in rows])
    plan,sent,actual=[np.array([r[k] for r in rows]) for k in ('plan','reference','actual')]
    moving=np.array([r['phase'] in ('outbound','return') for r in rows])
    vel=(np.interp(t+.1,t,actual)-np.interp(t-.1,t,actual))/.2
    holds={}
    for name in ('outbound_hold','return_hold'):
        idx=np.array([i for i,r in enumerate(rows) if r['phase']==name])
        if len(idx):
            idx=idx[t[idx]>=t[idx[-1]]-1.]
            holds[name]=dict(span_deg=float(np.rad2deg(np.ptp(actual[idx]))),
                mean_error_deg=float(np.rad2deg(np.mean(actual[idx]-plan[idx]))))
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for label, q in [('Plan',plan),('Sent',sent),('Actual',actual)]:axes[0].plot(t,np.rad2deg(q),label=label)
    axes[0].set_ylabel('J2 angle (deg)')
    axes[1].plot(t,np.rad2deg(plan-actual),label='Plan - actual')
    axes[1].plot(t,np.rad2deg(sent-actual),label='Sent - actual');axes[1].set_ylabel('Error (deg)')
    for i in (0,2,3,4,5):axes[2].plot(t,[math.degrees(r['actual_all'][i]-home[i]) for r in rows],label=JOINTS[i])
    axes[2].set_ylabel('Hold error (deg)');axes[2].set_xlabel('Time (s)')
    for ax in axes:ax.grid(alpha=.3);ax.legend(ncol=5)
    fig.tight_layout();fig.savefig(directory/'tracking.png',dpi=150);plt.close(fig)
    return dict(samples=len(rows),plan_error_max_deg=float(np.rad2deg(np.max(np.abs(plan-actual)))),
        plan_error_rms_deg=float(np.rad2deg(np.sqrt(np.mean((plan[moving]-actual[moving])**2)))),
        motor_error_max_deg=float(np.rad2deg(np.max(np.abs(sent-actual)))),
        tracking_limited_fraction=sum(r['reason']==26 for r in rows)/len(rows),
        encoder_peak_speed_deg_s=float(np.rad2deg(np.max(np.abs(vel)))),holds=holds,
        effort_peak_nm=max(abs(r['j2_hardware'].get('effort',0.)) for r in rows),
        max_other_joint_motion_deg=max(math.degrees(abs(r['actual_all'][i]-home[i])) for r in rows for i in (0,2,3,4,5)),
        overshoot_deg=float(np.rad2deg(max(0.,np.max(actual-plan.max()),np.max(plan.min()-actual)))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('mock','real'), default='mock')
    parser.add_argument('--start-degrees',type=float,default=0.)
    parser.add_argument('--end-degrees',type=float,default=10.)
    parser.add_argument('--move-seconds',type=float,default=12.)
    parser.add_argument('--settle-timeout',type=float,default=30.)
    parser.add_argument('--can-channel',default='/dev/ttyACM0')
    parser.add_argument('--output',type=Path)
    args = parser.parse_args();validate_motion(args.start_degrees,args.end_degrees,args.move_seconds)
    if not math.isfinite(args.settle_timeout) or not 1 <= args.settle_timeout <= 60:
        parser.error('settle timeout must be 1..60 seconds')
    domain = '87' if args.mode == 'mock' else os.environ.get('A6Y_REAL_ROS_DOMAIN_ID','0')
    if os.environ.get('ROS_DOMAIN_ID') != domain or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('use scripts/run_j2_position_test.bash to select the ROS domain')
    directory = (args.output or Path('validation/j2_position')/f'{time.time_ns()}_{args.mode}').resolve()
    directory.mkdir(parents=True,exist_ok=False)
    report = dict(mode=args.mode,start_degrees=args.start_degrees,end_degrees=args.end_degrees,move_seconds=args.move_seconds,
                  planned_peak_speed_deg_s=1.875*abs(args.end_degrees-args.start_degrees)/args.move_seconds,
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
        node = rclpy.create_node('j2_position_experiment')
        end = time.monotonic()+1.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if any(name=='controller_manager' for name,_ in node.get_node_names_and_namespaces()):
            raise RuntimeError('existing controller manager in this ROS domain')
        parameters, params, motors = make_configuration(directory,args.mode,args.can_channel)
        home = home_from_motors(motors)
        report.update(controller_parameters=params,motors=motors,home_positions=list(home))
        with (directory/'controller.log').open('w') as log, (directory/'trace.jsonl').open('w',buffering=1) as record:
            manager = subprocess.Popen(['ros2','run','controller_manager','ros2_control_node','--ros-args',
                '-r','__ns:=/j2_test','--params-file',str(parameters)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            report['hardware_accessed'] = args.mode == 'real'
            for name in ('joint_state_broadcaster',NAME):
                subprocess.run(['ros2','run','controller_manager','spawner',name,'--controller-manager',
                    '/j2_test/controller_manager','--param-file',str(parameters)],stdout=log,stderr=subprocess.STDOUT,
                    timeout=25,check=True)
            experiment = Experiment(node,args.mode,record,params,manager,home)
            try:
                experiment.run(math.radians(args.start_degrees),math.radians(args.end_degrees),args.move_seconds,args.settle_timeout)
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
            try:report['metrics'] = write_analysis(directory,experiment.samples,experiment.home)
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
