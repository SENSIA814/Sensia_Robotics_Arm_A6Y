#!/usr/bin/env python3
"""Passive physical feedback audit; --mock adds protocol tests in domain 86 only."""
import argparse
import json
import os
from pathlib import Path
import socket
import sys
import time
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from control_msgs.msg import DynamicJointState
from a6_y_teleop_msgs.msg import VrState, VrControl, VrJointReference
from diagnostic_msgs.msg import DiagnosticArray
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/a6_y_vr_teleop'))
from a6_y_vr_teleop.input_protocol import make_packet

PREFIX = '/arm6/a6_y_joint_impedance_controller'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--controller', choices=('impedance', 'position'), default='impedance')
    parser.add_argument('--exercise-full-range', action='store_true',
                        help='mock full position only: sustain FK-derived J5 motion beyond the old 2-degree CAD box')
    parser.add_argument('--exercise-speed', action='store_true',
                        help='with --exercise-full-range: verify continuous following exceeds the former 0.196 rad/s bottleneck')
    parser.add_argument('--duration', type=float, default=15.)
    parser.add_argument('--output', type=Path, default=Path('validation/vr_real/readonly.json'))
    args = parser.parse_args()
    if args.exercise_full_range and (not args.mock or args.controller != 'position'):
        parser.error('--exercise-full-range requires --mock --controller position')
    if args.exercise_speed and not args.exercise_full_range:
        parser.error('--exercise-speed requires --exercise-full-range (isolated mock only)')
    prefix = '/arm6/a6_y_vr_position_controller' if args.controller == 'position' else PREFIX
    if args.mock and (os.environ.get('ROS_DOMAIN_ID') != '86' or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        parser.error('synthetic control requires domain 86 + localhost')
    rclpy.init(); node = rclpy.create_node('vr_validation')
    latest = {}; states = []; diagnostics = []; hardware = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    def state_cb(v): latest['state'] = v; states.append(v)
    def hw_cb(v):
        joints = {name: dict(zip(row.interface_names, row.values)) for name, row in zip(v.joint_names, v.interface_values)}
        latest['hardware'] = joints; hardware.append(joints)
    node.create_subscription(VrState, prefix+'/vr_state', state_cb, qos)
    node.create_subscription(VrJointReference, prefix+'/vr_reference', lambda v: latest.update(reference=v), qos)
    node.create_subscription(DynamicJointState, '/arm6/dynamic_joint_states', hw_cb, qos)
    node.create_subscription(DiagnosticArray, '/arm6/vr/status', lambda v: diagnostics.extend(v.status), qos)
    sock = None; pub = None; seq = edge = 0; held = False
    report = dict(hardware_accessed=not args.mock, actual_quest_tested=False, controller=args.controller, mode='mock' if args.mock else 'readonly')
    def pump(duration, grip=False, position=(0., 0., 0.), send=True, trigger=0., quaternion=(0.,0.,0.,1.), pose_valid=True):
        nonlocal seq, edge, held
        end = time.monotonic()+duration; deadline = 0.
        while time.monotonic() < end:
            if sock is not None and send and time.monotonic() >= deadline:
                if grip != held: edge += 1
                held = grip; seq += 1; now = time.monotonic_ns()
                sock.sendto(make_packet('mock-integration', seq, now, position, squeeze=held, clutch_epoch=edge,
                    trigger=trigger,quaternion=quaternion,pose_valid=pose_valid,
                    schema_version=2, purpose='real', source_kind='test'), ('127.0.0.1', 15154))
                deadline = time.monotonic()+1/90
            rclpy.spin_once(node, timeout_sec=.002)
    def command(action):
        s = latest['state']; now = time.monotonic_ns()
        c = VrControl(action=action, controller_epoch=s.controller_epoch, request_id=now, generated_monotonic_ns=now)
        if action == VrControl.ARM:
            ref = latest['reference']; assert ref.valid and not ref.squeeze, 'no fresh released input'
            c.producer_id = ref.producer_id; c.session_id = ref.session_id; c.clutch_epoch = ref.clutch_epoch
        pub.publish(c); pump(.08)
        assert latest['state'].request_id == now and latest['state'].request_accepted, 'request rejected or unacknowledged'
    try:
        pump(1.5)
        assert 'state' in latest and 'hardware' in latest, 'no controller/hardware feedback'
        assert len(latest['hardware']) == 7
        assert latest['state'].position_control == (args.controller == 'position'), 'controller backend mismatch'
        assert latest['state'].state == VrState.DISABLED and not latest['state'].prepared
        assert all(j['enabled'] == 0 for j in latest['hardware'].values()), 'initial motor enable not zero'
        kin=None
        if args.mock and latest['state'].gripper_control:
            # Load verification-only FK before PREPARE; input pumping must stay
            # uninterrupted during active teleoperation.
            from scipy.spatial.transform import Rotation
            from a6_y_vr_teleop.a5_kinematics import A5Kinematics
            from a6_y_vr_teleop.commissioning import profile_paths
            paths,_=profile_paths();cfg=yaml.safe_load(paths['bridge'].read_text())
            if args.controller == 'position':
                from a6_y_vr_teleop.position_profile import position_paths, position_profile, CONTROLLER
                paths=position_paths(paths,'full')
                params=yaml.safe_load(paths['controller'].read_text())['/arm6/'+CONTROLLER]['ros__parameters']
                _,cfg=position_profile('follow',params,cfg,'full')
            kin=A5Kinematics(paths['urdf'],paths['limits'],cfg)
        if args.mock:
            assert all(j['dry_run'] == 1 for j in latest['hardware'].values()), 'refuse synthetic test on physical hardware'
            if args.exercise_full_range:
                assert latest['state'].gripper_control, 'extended test requires full position profile'
                assert latest['state'].session_reference_travel_limit == 0.
                assert latest['state'].session_actual_travel_limit == 0.
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            pub = node.create_publisher(VrControl, prefix+'/vr_control', 10)
            pump(.5); command(VrControl.PREPARE)
            ramp_deadline = time.monotonic()+10.
            while not latest['state'].prepare_ramp_complete and time.monotonic()<ramp_deadline:
                pump(.05)
            assert latest['state'].prepare_ramp_complete, 'prepare ramp did not complete'
            assert latest['state'].state == VrState.HOLD
            command(VrControl.ARM)
            if latest['state'].auto_rearm_after_release:
                # Reproduce the user's failure: invalid pose while waiting for
                # the first grip, followed by tracking recovery on the same link.
                request_before=latest['state'].request_id
                pump(.3,pose_valid=False)
                assert latest['state'].state == VrState.PAUSED, 'invalid pose did not stop'
                pump(.2)
                assert latest['state'].state == VrState.ARMED_HOLD, 'valid release did not recover waiting state'
                assert latest['state'].request_id == request_before
                report['invalid_pose_while_armed_auto_rearm_passed']=True
            pump(.15, True)
            assert latest['state'].state == VrState.TELEOP, 'capture failed: '+str(latest['state'].reason)
            anchor = np.asarray(latest['state'].anchor_positions)
            pump(.35, True, (0., 0., -.003))
            assert latest['state'].state == VrState.TELEOP, 'tracking stopped: '+str(latest['state'].reason)
            assert np.max(np.abs(np.asarray(latest['state'].reference_positions)-anchor)) > 1e-6
            if latest['state'].gripper_control:
                before=kin.fk(np.asarray(latest['state'].actual_positions))
                grip_before=latest['state'].gripper_reference_position
                opening=grip_before>-.002
                # A measured folded/closed start must be tested toward open,
                # not by assuming every mock begins at the old -15 mm opening.
                if opening: pump(.06,True,(0.,0.,-.003),trigger=1.)
                pump(.45,True,(0.,0.,-.003),trigger=0. if opening else 1.,quaternion=Rotation.from_rotvec([0.,0.,.03]).as_quat())
                assert latest['state'].state==VrState.TELEOP, 'full rotation/gripper stopped: '+str(latest['state'].reason)
                after=kin.fk(np.asarray(latest['state'].actual_positions))
                angle=Rotation.from_matrix(after[:3,:3]@before[:3,:3].T).magnitude()
                assert angle>1e-4, 'orientation did not follow'
                travel=latest['state'].gripper_reference_position-grip_before
                assert (-travel if opening else travel)>.0001, 'gripper did not follow selected direction'
                report.update(full_rotation_gripper_passed=True,rotation_rad=angle,
                              gripper_test_direction='opening' if opening else 'closing',gripper_travel_m=travel)
            pump(.3)
            assert latest['state'].state == VrState.ARMED_HOLD, 'normal release did not rearm hold'
            if args.exercise_full_range:
                # A reachable joint-space goal is converted to the exact inverse
                # of the bridge's relative WebXR mapping. This isolates the old
                # anchored CAD wall from arbitrary IK-unreachable hand targets.
                pump(.2, True)
                assert latest['state'].state == VrState.TELEOP, 'extended capture failed'
                captured = np.r_[latest['state'].anchor_positions,
                                 latest['state'].gripper_anchor_position]
                joint_delta = -.3 if args.exercise_speed else -.06
                desired = captured.copy(); desired[4] += joint_delta
                assert kin.feedback_in_bounds(desired), 'extended goal outside accepted joint bounds'
                start_tcp, goal_tcp = kin.fk(captured), kin.fk(desired)
                basis = np.asarray(cfg['vr_to_base'], dtype=float)
                hand_goal = basis.T @ (goal_tcp[:3, 3]-start_tcp[:3, 3]) / cfg['position_scale']
                hand_rotation = Rotation.from_matrix(
                    basis.T @ goal_tcp[:3, :3] @ start_tcp[:3, :3].T @ basis)
                assert np.linalg.norm(hand_goal) < cfg['input_translation_jump']
                assert hand_rotation.magnitude() < cfg['input_rotation_jump']
                first = len(states); started = time.monotonic()
                ramp_s = .5 if args.exercise_speed else 2.
                while time.monotonic()-started < ramp_s:
                    fraction = min(1., (time.monotonic()-started)/ramp_s)
                    pump(.02, True, tuple(fraction*hand_goal),
                         quaternion=Rotation.from_rotvec(fraction*hand_rotation.as_rotvec()).as_quat())
                pump(4., True, tuple(hand_goal), quaternion=hand_rotation.as_quat())
                exercised = states[first:]
                assert exercised and all(s.state == VrState.TELEOP for s in exercised), \
                    'extended following stopped: '+str(latest['state'].reason)
                origin = np.asarray(latest['state'].session_origin_positions)
                actual = np.asarray([s.actual_positions for s in exercised])
                reference = np.asarray([s.reference_positions for s in exercised])
                actual_j5_deg = float(np.max(np.abs(actual[:, 4]-origin[4]))*180/np.pi)
                reference_j5_deg = float(np.max(np.abs(reference[:, 4]-origin[4]))*180/np.pi)
                captured_j5_deg = float(np.max(np.abs(actual[:, 4]-captured[4]))*180/np.pi)
                held_s = (exercised[-1].monotonic_ns-exercised[0].monotonic_ns)/1e9
                report['full_range_exercise'] = dict(
                    desired_joint5_delta_rad=joint_delta, ramp_s=ramp_s, held_teleop_s=held_s,
                    actual_j5_departure_from_prepare_deg=actual_j5_deg,
                    reference_j5_departure_from_prepare_deg=reference_j5_deg,
                    actual_j5_travel_from_capture_deg=captured_j5_deg,
                    hand_goal_position_m=hand_goal.tolist(),
                    hand_goal_rotation_rad=float(hand_rotation.magnitude()),
                    prepare_origin=origin.tolist(), captured_positions=captured.tolist(),
                    final_actual_positions=actual[-1].tolist(),
                    final_reference_positions=reference[-1].tolist(), samples=len(exercised))
                if args.exercise_speed:
                    peak=float(np.max(np.abs([s.actual_velocities[:6] for s in exercised])))
                    peak_reference=float(np.max(np.abs([s.reference_velocities for s in exercised])))
                    final_error=kin.error(actual[-1],goal_tcp)
                    report['speed_exercise'] = dict(peak_actual_joint_speed_rad_s=peak,
                        peak_reference_joint_speed_rad_s=peak_reference,
                        final_tcp_error_m=float(np.linalg.norm(final_error[:3])),
                        final_rotation_error_rad=float(np.linalg.norm(final_error[3:])),
                        configured_joint_speed_rad_s=kin.speed[:6].tolist(),
                        target_linear_speed_m_s=cfg['target_linear_speed'],
                        target_angular_speed_rad_s=cfg['target_angular_speed'])
                    assert peak > .45, 'continuous mock motion is still slowed by the short-goal braking bottleneck'
                    assert peak_reference <= 1.+1e-9, 'controller output exceeded the configured joint speed'
                    assert np.linalg.norm(final_error[:3]) < .002, 'continuous following did not settle at the TCP goal'
                    assert np.linalg.norm(final_error[3:]) < .035, 'continuous following did not settle at the orientation goal'
                    report['speed_exercise']['passed']=True
                assert held_s > 2., 'extended held TELEOP interval too short'
                assert actual_j5_deg > 2.2, 'actual J5 did not pass former CAD wall'
                assert reference_j5_deg > 2.2, 'reference J5 did not pass former CAD wall'
                assert captured_j5_deg > 2.2, 'new takeover did not move J5 beyond 2.2 degrees'
                report['full_range_exercise']['passed'] = True
                pump(.3)
                assert latest['state'].state == VrState.ARMED_HOLD, 'extended release did not rearm hold'
            pump(.2, True); assert latest['state'].state == VrState.TELEOP
            pump(.4, send=False); assert latest['state'].state == VrState.PAUSED, 'input loss did not revoke permission'
            pump(.1, True); assert latest['state'].state == VrState.PAUSED, 'resumed with held grip'
            automatic=latest['state'].auto_rearm_after_release
            request_before=latest['state'].request_id
            pump(.2)
            if automatic:
                assert latest['state'].state == VrState.ARMED_HOLD, 'released fresh input did not rearm'
                assert latest['state'].request_id == request_before, 'recovery needed another local command'
            else: command(VrControl.ARM)
            pump(.2, True)
            assert latest['state'].state == VrState.TELEOP
            if automatic:
                command(VrControl.PAUSE); pump(.35)
                assert latest['state'].state == VrState.PAUSED, 'manual pause was automatically undone'
                pump(.2, True); assert latest['state'].state == VrState.PAUSED, 'grip bypassed manual pause'
                pump(.1);command(VrControl.ARM);pump(.2, True)
                assert latest['state'].state == VrState.TELEOP
                report['input_loss_release_auto_rearm_and_manual_pause_passed']=True
            command(VrControl.DISARM); assert latest['state'].state == VrState.DISABLED
            report['handshake_release_timeout_rearm_disarm_passed'] = True
        else:
            assert not latest['state'].output_permitted
            assert all(j['dry_run'] == 0 for j in latest['hardware'].values())
            pump(args.duration)
            assert all(s.state == VrState.DISABLED and not s.prepared and not s.output_permitted for s in states)
            assert all(j['enabled'] == 0 for rows in hardware for j in rows.values())
            assert all(s.feedback_valid for s in states)
        stamps = np.array([s.monotonic_ns for s in states], dtype=float)
        gaps = np.diff(stamps)/1e6
        limits = yaml.safe_load((Path(get_package_share_directory('a6_y_controllers'))/'config/a5_motion_limits.yaml').read_text())['joint_limits']
        names = [f'joint{i}' for i in range(1, 7)]+['gripper_finger_joint1']
        outside = {name: float(value) for name, value in zip(names, latest['state'].actual_positions)
                   if not limits[name]['min_position'] <= value <= limits[name]['max_position']}
        report.update(passed=True, samples=len(states), observed_state_hz=(len(stamps)-1)*1e9/(stamps[-1]-stamps[0]),
            state_gap_ms_p99=float(np.percentile(gaps, 99)), state_gap_ms_max=float(np.max(gaps)),
            max_position_age_s=max(max(s.position_age) for s in states),
            max_feedback_age_s=max(max(s.feedback_age) for s in states),
            actual_positions=list(latest['state'].actual_positions),
            actual_outside_runtime_limits=outside,
            initial_all_motors_disabled=all(j['enabled'] == 0 for j in hardware[0].values()),
            final_all_motors_disabled=all(j['enabled'] == 0 for j in hardware[-1].values()),
            all_samples_motors_disabled=all(j['enabled'] == 0 for rows in hardware for j in rows.values()),
            states_seen=sorted({s.state for s in states}),
            bridge_diagnostics=sorted({s.message for s in diagnostics}))
    except (AssertionError, KeyError, RuntimeError) as error:
        report.update(passed=False, error=str(error), last_state=latest['state'].state if 'state' in latest else None,
                      last_reason=latest['state'].reason if 'state' in latest else None,
                      bridge_diagnostics=sorted({s.message for s in diagnostics}))
    finally:
        if sock: sock.close()
        node.destroy_node(); rclpy.shutdown()
    args.output.parent.mkdir(parents=True, exist_ok=True); args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2)); return 0 if report['passed'] else 2


if __name__ == '__main__': raise SystemExit(main())
