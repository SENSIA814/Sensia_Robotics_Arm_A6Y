"""Regression from the user's 2026-10-08 folded-pose physical trace.

These tests reconstruct one tick offline; they never start ROS or access motors.
"""
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from a6_y_vr_teleop.a5_kinematics import A5Kinematics
from a6_y_vr_teleop.geometry_guard import GeometryGuard
from a6_y_vr_teleop.input_protocol import Frame, make_packet
from a6_y_vr_teleop.real_bridge import RealBridge

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = ROOT / 'validation/vr_position/latest_first_mesh_failure_fixture.json'


def recorded_bridge(checking=None, no_geometry=False):
    data = json.loads(FIXTURE.read_text())
    cfg = data['config']
    if checking is not None:
        cfg['geometry_checking'] = checking
    urdf = ROOT / 'src/A5/urdf/A5.urdf'
    kin = A5Kinematics(urdf, ROOT / 'src/a6_y_controllers/config/a5_motion_limits.yaml', cfg)
    geometry = None if no_geometry else GeometryGuard(kin, urdf, cfg)
    arm = RealBridge(kin, geometry, cfg, 'execute', clock=data['arm']['clock'])
    for key, value in data['arm'].items():
        if key in ('frame', 'previous', 'anchor_frame') and value is not None:
            value = dict(value)
            value['position'] = np.asarray(value['position'])
            value['rotation'] = np.asarray(value['rotation'])
            value = Frame(**value)
        elif key in ('anchor_tcp', 'target', 'raw_target', 'preview') and value is not None:
            value = np.asarray(value)
        elif key == 'retired':
            value = set(value)
        setattr(arm, key, value)
    arm.state = SimpleNamespace(**data['controller_state'])
    if geometry is not None:
        geometry.zero_epoch = data['geometry_epoch']
        geometry.zero_anchor = np.asarray(data['geometry_anchor'])
        geometry.zero_seen_enabled = data['geometry_seen_enabled']
    return arm, data['now_ns'], data['dt_s']


@pytest.mark.parametrize('checking', [None, True])
def test_recorded_folded_candidate_stays_rejected_with_strict_or_default_geometry(checking):
    arm, now, dt = recorded_bridge(checking)
    reference, control = arm.tick(now, dt, enforce_budget=False)
    assert not reference['valid'] and control is None and arm.latched
    assert arm.reason == 'mesh_clearance:link1_0:link4_0'


@pytest.mark.parametrize('no_geometry', [False, True])
def test_recorded_folded_candidate_advances_with_explicit_full_position_policy(no_geometry):
    arm, now, dt = recorded_bridge(False, no_geometry)
    before = np.asarray(arm.state.reference_positions)
    reference, control = arm.tick(now, dt, enforce_budget=False)
    assert reference['valid'], arm.reason
    assert not arm.latched and control is None and arm.reason == 'following'
    command = np.asarray(reference['positions'])
    assert np.max(np.abs(command-before)) > 1e-5
    assert np.all(np.abs(command-before) <= arm.kin.speed[:6]*min(dt, 1/arm.cfg['update_rate']) + 1e-12)
    assert np.all(command >= arm.kin.lower[:6])
    assert np.all(command <= arm.kin.upper[:6])
    assert abs(command[4]-arm.state.session_origin_positions[4]) > np.deg2rad(2)


@pytest.mark.parametrize('fault', ['stale_state', 'stale_motor', 'invalid_feedback',
                                  'nonfinite_position', 'nonfinite_velocity'])
def test_full_geometry_policy_retains_feedback_rejection(fault):
    arm, now, dt = recorded_bridge(False)
    if fault == 'stale_state':
        arm.state.monotonic_ns = now-int((arm.cfg['feedback_timeout']+.01)*1e9)
    elif fault == 'stale_motor':
        arm.state.position_age[2] = arm.cfg['feedback_timeout']+.01
    elif fault == 'invalid_feedback':
        arm.state.feedback_valid = False
    elif fault == 'nonfinite_position':
        arm.state.actual_positions[2] = float('nan')
    elif fault == 'nonfinite_velocity':
        arm.state.actual_velocities[2] = float('inf')
    assert arm.tick(now, dt, enforce_budget=False) == (None, None)
    assert arm.latched and arm.reason == 'feedback_stale_or_invalid'


def test_full_geometry_policy_retains_stale_input_rejection():
    arm, now, dt = recorded_bridge(False)
    arm.frame = replace(arm.frame, pose_ns=now-int((arm.cfg['input_timeout']+.01)*1e9))
    reference, control = arm.tick(now, dt, enforce_budget=False)
    assert not reference['valid'] and control is None and arm.latched
    assert arm.reason == 'input_expired'


def packet_after_frame(arm, now, position):
    frame = arm.frame
    return make_packet(frame.session, frame.sequence+1, now, position,
        quaternion=Rotation.from_matrix(frame.rotation).as_quat(), squeeze=True,
        trigger=frame.trigger, clutch_epoch=frame.clutch_epoch,
        pose_sequence=frame.pose_sequence+1, schema_version=2, purpose='real', source_kind='live')


def test_full_geometry_policy_retains_nonfinite_input_rejection():
    arm, now, dt = recorded_bridge(False)
    payload = json.loads(packet_after_frame(arm, now, arm.frame.position))
    payload['position_m'][0] = float('nan')
    assert not arm.receive(json.dumps(payload).encode(), now)
    reference, control = arm.tick(now, dt, enforce_budget=False)
    assert not reference['valid'] and control is None and arm.latched
    assert arm.reason == 'invalid_packet'


def test_full_geometry_policy_retains_held_tracking_jump_rejection():
    arm, now, dt = recorded_bridge(False)
    jumped = arm.frame.position + np.array([.1419930678746726, 0., 0.])
    assert arm.receive(packet_after_frame(arm, now, jumped), now)
    reference, control = arm.tick(now, dt, enforce_budget=False)
    assert not reference['valid'] and control is None and arm.latched
    assert arm.reason == 'tracking_jump'


def test_full_geometry_policy_retains_actual_joint_limits():
    arm, now, dt = recorded_bridge(False)
    arm.state.actual_positions[4] = arm.kin.upper[4]+.01
    reference, control = arm.tick(now, dt, enforce_budget=False)
    assert not reference['valid'] and control is None and arm.latched
    assert arm.reason == 'actual_outside_model_limits_calibration_required'


def test_full_geometry_policy_accepts_fresh_idle_feedback_without_geometry():
    arm, now, dt = recorded_bridge(False, no_geometry=True)
    fresh = SimpleNamespace(**vars(arm.state))
    fresh.monotonic_ns += 1
    arm.observe(fresh, now)
    assert arm.state is fresh
    # Released idle input still passes the controller handshake; skipping CAD
    # construction must also work outside the moving-reference branch.
    arm.state.state = 2
    arm.frame = replace(arm.frame, squeeze=False)
    reference, control = arm.tick(now, dt, enforce_budget=False)
    assert reference['valid'] and control is None and not arm.latched
    np.testing.assert_array_equal(reference['positions'], arm.state.actual_positions[:6])
