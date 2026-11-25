#!/usr/bin/env python3
"""Supported physical HOLD test (PREPARE outputs torque). Only for stage:=hold.

No movement targets are sent. By default the controller remains in HOLD after
measurement. --disarm-at-end is only for an arm that stays mechanically supported.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from a6_y_teleop_msgs.msg import VrControl, VrState
from control_msgs.msg import DynamicJointState
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/a6_y_vr_teleop'))
from a6_y_vr_teleop.commissioning_metrics import hold_metrics, disarm_metrics
from a6_y_vr_teleop.commissioning import profile_paths, sha256

PREFIX = '/arm6/a6_y_joint_impedance_controller'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--controller', choices=('impedance', 'position'), default='impedance')
    parser.add_argument('--motion-range', choices=('trial','full'), default='trial')
    parser.add_argument('--duration', type=float, default=15.)
    parser.add_argument('--disarm-at-end', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prefix = '/arm6/a6_y_vr_position_controller' if args.controller == 'position' else PREFIX
    if not np.isfinite(args.duration) or args.duration < 10: parser.error('duration must be at least 10 s')
    domain = os.environ.get('ROS_DOMAIN_ID')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or domain in (None, '84') or (args.mock != (domain == '86')):
        parser.error('explicit physical domain or --mock with isolated domain 86 required')
    trace_path = args.output.with_suffix('.jsonl')
    if args.output.exists() or trace_path.exists(): parser.error('output/trace already exists')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    latest, states, hardware = {}, [], []
    report = dict(passed=False, hardware_accessed=False, actual_quest_tested=False,
                  controller=args.controller,
                  test='supported_hold', created_at=datetime.now().astimezone().isoformat(),
                  duration_requested_s=args.duration, trace=str(trace_path.resolve()))
    paths, _ = profile_paths()
    if args.controller == 'position':
        from a6_y_vr_teleop.position_profile import position_paths
        paths = position_paths(paths,args.motion_range)
    report['sha256'] = {k: sha256(p) for k, p in paths.items()}
    rclpy.init(); node = rclpy.create_node('vr_supported_hold_test')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(VrControl, prefix+'/vr_control', 10)
    trace = trace_path.open('x', buffering=1)
    trace.write(json.dumps(dict(kind='metadata', **report))+'\n')

    def state_cb(s):
        row = dict(kind='state', monotonic_ns=s.monotonic_ns, state=s.state, reason=s.reason,
                   controller_epoch=s.controller_epoch, feedback_valid=s.feedback_valid,
                   follow_permitted=s.follow_permitted, output_permitted=s.output_permitted,
                   actual=list(s.actual_positions), velocity=list(s.actual_velocities),
                   reference=list(s.reference_positions), position_age=list(s.position_age),
                   gripper_control=s.gripper_control,gripper_reference=s.gripper_reference_position,
                   feedback_age=list(s.feedback_age), saturated=list(s.saturated))
        latest['state'] = s; states.append(row); trace.write(json.dumps(row)+'\n')

    def hw_cb(msg):
        rows = {name: dict(zip(values.interface_names, values.values)) for name, values in zip(msg.joint_names, msg.interface_values)}
        latest.update(hardware=rows, hardware_ns=time.monotonic_ns()); hardware.append(rows)
        trace.write(json.dumps(dict(kind='hardware', monotonic_ns=latest['hardware_ns'], joints=rows))+'\n')

    node.create_subscription(VrState, prefix+'/vr_state', state_cb, qos)
    node.create_subscription(DynamicJointState, '/arm6/dynamic_joint_states', hw_cb, qos)

    def spin_until(predicate, timeout=3.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)
            if predicate(): return
        raise RuntimeError('timed out waiting for fresh state or command acknowledgement')

    def command(action):
        s = latest['state']; stamp = time.monotonic_ns()
        msg = VrControl(action=action, controller_epoch=s.controller_epoch,
                        request_id=stamp, generated_monotonic_ns=stamp)
        trace.write(json.dumps(dict(kind='command', action=action, request_id=stamp))+'\n')
        pub.publish(msg)
        spin_until(lambda: latest['state'].request_id == stamp)
        if not latest['state'].request_accepted:
            raise RuntimeError(f'command {action} rejected; actual={list(latest["state"].actual_positions)}')

    try:
        spin_until(lambda: 'state' in latest and 'hardware' in latest and pub.get_subscription_count() > 0)
        s, hw = latest['state'], latest['hardware']
        if s.position_control != (args.controller == 'position'): raise RuntimeError('controller backend mismatch')
        full=args.controller=='position' and args.motion_range=='full'
        if s.gripper_control != full: raise RuntimeError('gripper backend mismatch')
        if len(hw) != 7 or any(j['dry_run'] != int(args.mock) for j in hw.values()):
            raise RuntimeError('hardware/mock identity mismatch')
        report['hardware_accessed'] = not args.mock
        travel_valid=(s.session_reference_travel_limit==s.session_actual_travel_limit==0. if full else
                      0<s.session_reference_travel_limit<=np.deg2rad(1.)+1e-12 and
                      0<s.session_actual_travel_limit<=np.deg2rad(2.)+1e-12)
        if (s.state != VrState.DISABLED or not s.output_permitted or s.follow_permitted
                or not travel_valid
                or any(j['enabled'] != 0 for j in hw.values())):
            raise RuntimeError('requires disabled controller in stage:=hold with session travel protection')
        spin_until(lambda: latest['state'].feedback_valid and
                   0 <= time.monotonic_ns()-latest['state'].monotonic_ns < 50_000_000)
        command(VrControl.PREPARE)
        print('PREPARE accepted: '+('native position hold active' if args.controller == 'position'
                                   else 'support torque active')+'; measuring HOLD.', flush=True)
        prepared_at = time.monotonic(); measure_start = None; end = None
        while end is None or time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.005); s = latest['state']
            if s.state != VrState.HOLD or not s.feedback_valid:
                raise RuntimeError(f'HOLD exited or feedback invalid: state={s.state}, reason={s.reason}')
            if time.monotonic_ns()-s.monotonic_ns > 100_000_000 or time.monotonic_ns()-latest['hardware_ns'] > 100_000_000:
                raise RuntimeError('state/hardware stream stale')
            if np.max(np.abs(np.asarray(s.actual_positions[:6])-s.reference_positions)) > np.deg2rad(1.):
                raise RuntimeError('HOLD drift exceeded 1 degree')
            if s.gripper_control and abs(s.actual_positions[6]-s.gripper_reference_position)>.0015:
                raise RuntimeError('gripper HOLD drift exceeded 1.5 mm')
            # The driver's dry_run branch never reports physical motor enable.
            # Real hardware must still confirm all seven enables explicitly.
            if not args.mock and time.monotonic()-prepared_at > 1. and any(
                    j['enabled'] != (0 if args.controller == 'position' and not s.gripper_control and name == 'gripper_finger_joint1' else 1)
                    for name,j in latest['hardware'].items()):
                raise RuntimeError('motor enable feedback incomplete')
            if measure_start is None and s.prepare_ramp_complete:
                measure_start = time.monotonic_ns()+200_000_000
                end = time.monotonic()+.2+args.duration+.05
            if measure_start is None and time.monotonic()-prepared_at > 10.:
                raise RuntimeError('prepare ramp did not complete')
        window = [row for row in states if row['monotonic_ns'] >= measure_start]
        report['hold'] = hold_metrics(window, args.duration-.05)
        if full:
            report['gripper_hold']=dict(max_error_m=float(max(abs(r['actual'][6]-r['gripper_reference']) for r in window)),
                                       max_speed_m_s=float(max(abs(r['velocity'][6]) for r in window)),
                                       reference_fixed=all(r['gripper_reference']==window[0]['gripper_reference'] for r in window))
            report['hold']['passed'] = bool(report['hold']['passed'] and report['gripper_hold']['reference_fixed'] and report['gripper_hold']['max_speed_m_s']<=.003)
        report['passed'] = report['hold']['passed']
        if not report['passed']: command(VrControl.PAUSE)
    except (RuntimeError, KeyError, KeyboardInterrupt) as error:
        report['error'] = str(error) or 'interrupted'
        if 'state' in latest and latest['state'].state in (1, 2, 3, 4, 5):
            try: command(VrControl.PAUSE)
            except RuntimeError as stop_error: report['pause_error'] = str(stop_error)
    finally:
        if args.disarm_at_end and 'state' in latest and latest['state'].state in (1, 2, 3, 4, 5):
            try:
                command(VrControl.DISARM)
                spin_until(lambda: all(j['enabled'] == 0 for j in latest['hardware'].values()))
            except (RuntimeError, KeyError) as error:
                report.update(passed=False, disarm_error=str(error))
        if args.disarm_at_end and 'state' in latest:
            # A fault can arrive one cycle before the hardware's disabled status.
            # Drain fresh feedback instead of reporting the cached enabled frame.
            stop_check_ns = time.monotonic_ns()
            try:
                spin_until(lambda: latest.get('hardware_ns', 0) > stop_check_ns and
                           all(j['enabled'] == 0 for j in latest.get('hardware', {}).values()))
            except RuntimeError as error:
                report.update(passed=False, shutdown_confirmation_error=str(error))
            # A single DISABLED acknowledgement can precede a fault in the next
            # driver cycle. Include the post-stop state stream in the verdict.
            observe_until = time.monotonic()+.35
            while time.monotonic() < observe_until:
                rclpy.spin_once(node, timeout_sec=.005)
            report['disarm'] = disarm_metrics([row for row in states if row['monotonic_ns'] >= stop_check_ns])
            report['passed'] = bool(report['passed'] and report['disarm']['passed'])
        report['final_state'] = latest['state'].state if 'state' in latest else None
        report['final_all_motors_disabled'] = bool(latest.get('hardware')) and all(j['enabled'] == 0 for j in latest['hardware'].values())
        if args.disarm_at_end:
            report['passed'] = bool(report['passed'] and report['final_all_motors_disabled']
                                   and time.monotonic_ns()-latest.get('hardware_ns', 0) < 100_000_000)
        report['samples'] = len(states)
        trace.close(); node.destroy_node(); rclpy.shutdown()
        args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2)); return 0 if report['passed'] else 2


if __name__ == '__main__': raise SystemExit(main())
