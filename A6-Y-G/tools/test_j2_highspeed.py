#!/usr/bin/env python3
"""Operator-scoped J2 high-speed position trial; other five arm axes hold.

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
from collections import deque

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from a6_y_teleop_msgs.msg import VrState, VrControl, VrJointReference
import test_j1_position as base
import test_j2_position as legacy
from test_j1_highspeed import validate_motion, transfer_duration, planned_goal

NAME, JOINTS = base.NAME, base.JOINTS
PREFIX = '/j2_fast/'+NAME
MAX_TRAVEL = math.pi/2
EFFORT_LIMITS = [10., 20., 20., 3., 4., 4.]


def load_home(path):
    data = json.loads(Path(path).read_text())
    home = np.asarray(data['home_model'], dtype=float)
    initial = np.asarray(data['initial_model'], dtype=float)
    if home.shape != (6,) or initial.shape != (7,) or not np.isfinite(home).all() or not np.isfinite(initial).all():
        raise ValueError('six finite home and seven initial positions required')
    if home[1] != 0.:
        raise ValueError('J2 starts at the previously confirmed nominal zero')
    return home, initial


def validate_initial_pose(actual, params):
    actual = np.asarray(actual, dtype=float)
    if actual.shape != (7,) or not np.isfinite(actual).all():
        raise ValueError('seven finite feedback positions required')
    if not -math.radians(2.) <= actual[1] <= legacy.J2_MAX+math.radians(.5):
        raise ValueError('initial J2 outside previously confirmed sweep')
    # As in the previous J2 procedure, capture valid feedback then home
    # slowly. Disabled wrist joints can settle away from the held pose;
    # the 2-degree hold check applies after homing, not before it.
    for i in range(6):
        tolerance=params['vr_zero_tolerance'][i]
        if not params['vr_lower'][i]-tolerance <= actual[i] <= params['vr_upper'][i]+tolerance:
            raise ValueError(f'initial joint{i+1} outside configured position limits')


def make_configuration(directory, mode, channel, home, raw_model, peak_speed=90.):
    parameter_file, params, motors = base.make_configuration(directory,mode,channel,
        namespace='j2_fast',commanded_joints=JOINTS[:6],mock_positions=list(raw_model))
    # Only J2 speed ceilings change, in this generated experiment snapshot.
    # Production VR configuration and earlier low-speed entry points stay intact.
    speed_ceiling=min(1.65,max(.1,1.65*peak_speed/90.))
    params['vr_reference_velocity'][1] = speed_ceiling
    params['position_velocity_limit'][1] = speed_ceiling
    params['vr_actual_velocity'][1] = min(1.90,max(.3,1.90*peak_speed/90.))
    motors['joints']['joint2']['max_velocity'] = speed_ceiling
    (directory/'motors.yaml').write_text(base.yaml.safe_dump(motors))
    data = base.yaml.safe_load(parameter_file.read_text())
    manager = data['/j2_fast/controller_manager']['ros__parameters']
    root = base.ET.fromstring(manager['robot_description'])
    root.find("ros2_control/joint[@name='joint2']/param[@name='max_velocity']").text=str(speed_ceiling)
    manager['robot_description']=base.ET.tostring(root,encoding='unicode')
    data['/j2_fast/'+NAME]['ros__parameters']=params
    parameter_file.write_text(base.yaml.safe_dump(data))
    if not np.allclose(home,legacy.home_from_motors(motors),atol=1e-10,rtol=0.):
        raise ValueError('home must match the previously confirmed J2 pose and current motor mapping')
    if not params['vr_lower'][1] <= home[1] < home[1]+MAX_TRAVEL <= min(legacy.J2_MAX, params['vr_upper'][1]):
        raise ValueError('confirmed J2 90-degree envelope exceeds nominal joint range')
    return parameter_file, params, motors


class Experiment(base.Experiment):
    def __init__(self, node, mode, record, params, manager, home, initial):
        super().__init__(node, mode, record, params, manager, namespace='j2_fast')
        self.home = home
        self.initial = initial
        self.test_started = False
        self.capture = None

    def observe_hw(self, msg):
        self.hardware = {n: dict(zip(row.interface_names, row.values))
                         for n, row in zip(msg.joint_names, msg.interface_values)}
        self.hardware_ns = time.monotonic_ns()
        if self.mode == 'real' and self.hardware.get(JOINTS[6], {}).get('enabled', 0.) != 0.:
            self.abort = 'read-only gripper unexpectedly enabled'
        if self.active or self.enabled_once:
            for i, n in enumerate(JOINTS[:6]):
                h = self.hardware.get(n, {})
                effort = h.get('effort', float('nan'))
                if not math.isfinite(effort) or abs(effort) > EFFORT_LIMITS[i]:
                    self.abort = f'{n} feedback effort invalid or exceeds {EFFORT_LIMITS[i]} Nm'
                if self.active and self.mode == 'real' and h.get('enabled') != 1.:
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
            if not -math.radians(2.) <= s.actual_positions[1] <= legacy.J2_MAX+math.radians(.5):
                self.abort = 'J2 left the confirmed sweep'
            if self.test_started and max(abs(s.actual_positions[i]-self.home[i]) for i in (0,2,3,4,5)) > math.radians(2.):
                self.abort = 'a holding arm joint moved more than 2 degrees from home'
        if self.anchor is None or self.abort or now-s.monotonic_ns > 50_000_000:
            return
        plan = self.goal.copy()
        goal = plan.copy()
        if self.trajectory is not None:
            start, end, began, duration = self.trajectory
            plan,goal = planned_goal(start,end,time.monotonic()-began,duration,
                np.array(s.reference_positions),np.array(self.params['vr_reference_acceleration']))
        positions = list(goal)
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
            plan_all=list(plan), goal_all=list(goal), reference=s.reference_positions[1], actual=s.actual_positions[1],
            actual_velocity=s.actual_velocities[1], actual_all=list(s.actual_positions),
            actual_velocities_all=list(s.actual_velocities),
            reference_all=list(s.reference_positions), reference_velocity=s.reference_velocities[1],
            position_age=list(s.position_age), feedback_age=list(s.feedback_age),
            state=s.state, reason=s.reason, hardware=self.hardware,
            j2_hardware=dict(self.hardware.get('joint2', {})))
        self.record.write(json.dumps(row)+'\n')
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

    def command(self, action):
        try:
            super().command(action)
        except RuntimeError:
            s=self.state
            self.record.write(json.dumps(dict(kind='command_diagnostics',action=int(action),
                state=int(s.state),reason=int(s.reason),feedback_valid=s.feedback_valid,
                actual_velocities=list(s.actual_velocities),hardware=self.hardware))+'\n')
            raise

    def settle(self, goal, timeout):
        self.goal, self.trajectory = goal.copy(), None
        window = deque()
        duration = self.params['vr_settle_duration']
        def reached():
            s = self.state
            q = np.array(s.actual_positions[:6])
            good = (np.isfinite(q).all() and max(abs(q-goal)) < math.radians(.5) and
                    max(abs(s.reference_positions[i]-goal[i]) for i in range(6)) < math.radians(.1))
            if not good:
                window.clear()
                return False
            stamp = s.monotonic_ns*1e-9
            if not window or stamp > window[-1][0]:
                window.append((stamp,q))
            while len(window)>1 and window[1][0]<=stamp-duration:
                window.popleft()
            elapsed = stamp-window[0][0]
            # Native wrist velocity estimates jump by protocol bins even
            # when encoders hold still. Assess actual position stability;
            # ARM/CAPTURE below still enforce raw velocity in the controller.
            return (elapsed >= duration and
                    np.max(np.ptp(np.array([x[1] for x in window]),axis=0)) < math.radians(.1) and
                    np.max(np.abs(q-window[0][1]))/elapsed < self.params['vr_settle_speed'])
        self.wait(reached, timeout, 'all six joints settled')

    def command_when_stationary(self, action, expected_state, timeout=5.):
        """Retry only an explicit preparation rejection, never a motion fault.

        DM4310 velocity bins can cross the stationary threshold between the
        observation and the command cycle. The controller checks every retry;
        no velocity threshold, follow gate, or stopping rule is bypassed.
        """
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.wait(lambda: max(abs(v) for v in self.state.actual_velocities[:6]) < self.params['vr_settle_speed'],
                      max(.001,deadline-time.monotonic()), 'stationary preparation command')
            try:
                self.command(action)
                return
            except RuntimeError as error:
                if (str(error) != f'command {action} rejected' or self.active or self.abort or
                        self.state.state != expected_state or not self.state.feedback_valid):
                    raise
                self.pump(.05)
        raise RuntimeError(f'command {action} remained rejected during preparation')

    def move(self, phase, start, end, duration, settle_timeout):
        self.phase = phase
        self.trajectory = (start.copy(), end.copy(), time.monotonic(), duration)
        self.pump(duration)
        self.phase = phase+'_settle'; self.settle(end, settle_timeout)
        self.phase = phase+'_hold'; self.pump(2.)

    def run(self, start_angle, end_angle, duration, settle_timeout, return_home_only=False):
        self.wait(lambda: self.state is not None and self.state.feedback_valid and len(self.hardware)==7 and
                  max(abs(v) for v in self.state.actual_velocities[:6]) <= self.params['vr_settle_speed'],
                  8., 'initial stationary feedback')
        s = self.state
        if s.state != VrState.DISABLED or not s.position_control:
            raise RuntimeError('expected disabled position controller')
        if any(self.hardware[n]['dry_run'] != (1. if self.mode=='mock' else 0.) for n in JOINTS):
            raise RuntimeError('mock/physical mismatch')
        if any(self.hardware[n]['enabled'] != 0. for n in JOINTS):
            raise RuntimeError('motors already enabled')
        if max(abs(v) for v in s.actual_velocities[:6]) > .03:
            raise RuntimeError('initial arm is not stationary')
        validate_initial_pose(s.actual_positions, self.params)
        self.epoch = s.controller_epoch
        self.anchor = list(s.actual_positions); self.goal = np.array(self.anchor[:6])
        self.pump(.25)
        if self.node.count_publishers(PREFIX+'/vr_reference') != 1:
            raise RuntimeError('another reference producer present')
        self.phase = 'prepare'
        self.command_when_stationary(VrControl.PREPARE,VrState.DISABLED)
        self.enabled_once = True
        self.wait(lambda: self.state.prepare_ramp_complete, 3., 'prepare ramp')
        if self.mode == 'real':
            self.wait(lambda: all(self.hardware[n]['enabled']==1. for n in JOINTS[:6]), 2., 'six arm motors enabled')
        self.phase='prepared_settle'
        self.settle(np.array(self.anchor[:6]),5.)
        self.command_when_stationary(VrControl.ARM,VrState.HOLD)
        self.held = True; self.edge += 1; self.pump(.05)
        self.command_when_stationary(VrControl.CAPTURE,VrState.ARMED_HOLD)
        self.anchor = list(self.state.anchor_positions)+[self.anchor[6]]
        self.capture = list(self.anchor)
        self.goal = np.array(self.anchor[:6]); self.active = True
        self.wait(lambda: self.state.state==VrState.TELEOP and self.state.accepted_sequence>0, 2., 'captured reference')
        self.phase='captured_hold'; self.pump(1.)
        self.move('homing', self.goal, self.home, transfer_duration(self.goal,self.home), settle_timeout)
        if return_home_only:
            return
        self.test_started = True
        start, end = self.home.copy(), self.home.copy()
        start[1], end[1] = start_angle, end_angle
        self.move('outbound', start, end, duration, settle_timeout)
        self.move('return', end, start, duration, settle_timeout)


def write_analysis(directory, samples, home):
    with (directory/'trace.jsonl').open() as source:
        samples = [json.loads(line) for line in source]
    # Abort diagnostics also have a phase, but are not telemetry samples.
    samples = [r for r in samples if 't_ns' in r]
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
    parser.add_argument('--anchor-json',type=Path,required=True)
    parser.add_argument('--degrees',type=float,default=90.)
    parser.add_argument('--peak-speed-deg-s',type=float,default=90.)
    parser.add_argument('--settle-timeout',type=float,default=30.)
    parser.add_argument('--can-channel',default='/dev/ttyACM0')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--return-home-only', action='store_true',
        help='capture a stationary J2 inside the confirmed sweep and return at <=2.5 deg/s; no test sweep')
    args = parser.parse_args()
    args.move_seconds = validate_motion(args.degrees,args.peak_speed_deg_s)
    home, raw_model = load_home(args.anchor_json)
    args.start_degrees = math.degrees(home[1])
    args.end_degrees = args.start_degrees+args.degrees
    if not math.isfinite(args.settle_timeout) or not 1 <= args.settle_timeout <= 60:
        parser.error('settle timeout must be 1..60 seconds')
    domain = '90' if args.mode == 'mock' else os.environ.get('A6Y_REAL_ROS_DOMAIN_ID','0')
    if os.environ.get('ROS_DOMAIN_ID') != domain or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('use scripts/run_j2_highspeed_test.bash to select the ROS domain')
    directory = (args.output or Path('validation/j2_highspeed')/f'{time.time_ns()}_{args.mode}').resolve()
    directory.mkdir(parents=True,exist_ok=False)
    report = dict(mode=args.mode,start_degrees=args.start_degrees,end_degrees=args.end_degrees,move_seconds=args.move_seconds,
                  planned_peak_speed_deg_s=1.875*abs(args.end_degrees-args.start_degrees)/args.move_seconds,
                  return_home_only=args.return_home_only,
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
        node = rclpy.create_node('j2_highspeed_experiment')
        end = time.monotonic()+1.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if any(name=='controller_manager' for name,_ in node.get_node_names_and_namespaces()):
            raise RuntimeError('existing controller manager in this ROS domain')
        parameters, params, motors = make_configuration(directory,args.mode,args.can_channel,home,raw_model,args.peak_speed_deg_s)
        report.update(controller_parameters=params,motors=motors,home_positions=list(home),operator_anchor=str(args.anchor_json.resolve()))
        with (directory/'controller.log').open('w') as log, (directory/'trace.jsonl').open('w',buffering=1) as record:
            manager = subprocess.Popen(['ros2','run','controller_manager','ros2_control_node','--ros-args',
                '-r','__ns:=/j2_fast','--params-file',str(parameters)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            report['hardware_accessed'] = args.mode == 'real'
            for name in ('joint_state_broadcaster',NAME):
                subprocess.run(['ros2','run','controller_manager','spawner',name,'--controller-manager',
                    '/j2_fast/controller_manager','--param-file',str(parameters)],stdout=log,stderr=subprocess.STDOUT,
                    timeout=25,check=True)
            experiment = Experiment(node,args.mode,record,params,manager,home,raw_model)
            try:
                experiment.run(math.radians(args.start_degrees),math.radians(args.end_degrees),args.move_seconds,args.settle_timeout,args.return_home_only)
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
