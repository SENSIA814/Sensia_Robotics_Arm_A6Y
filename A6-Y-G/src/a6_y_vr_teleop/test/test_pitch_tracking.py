"""Pitch tracking with an independently lagging shoulder/base, no hardware."""
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from a6_y_vr_teleop.a5_kinematics import A5Kinematics
from a6_y_vr_teleop.input_protocol import decode,make_packet
from a6_y_vr_teleop.real_bridge import RealBridge,identifier

ROOT=Path(__file__).resolve().parents[3]


def pitch_bridge(direction,bounded=True):
    fixture=json.loads((ROOT/'validation/vr_position/pitch_tracking_fixture.json').read_text())['samples'][direction]
    cfg=dict(fixture['config'],bounded_ik=bounded)
    row=fixture['sample'];q=np.r_[row['reference'],row['actual'][6]];actual=np.array(row['actual'])
    blocked=1 if direction=='down' else 0
    actual[blocked]=q[blocked]-.03
    kin=A5Kinematics(ROOT/'src/A5/urdf/A5.urdf',ROOT/'src/a6_y_controllers/config/a5_vr_position_full_limits.yaml',cfg)
    arm=RealBridge(kin,None,cfg,'execute',clock=22);now=10**12
    anchor=kin.fk(q);target=np.asarray(row['target']);basis=np.asarray(cfg['vr_to_base'])
    position=basis.T@(target[:3,3]-anchor[:3,3])/cfg['position_scale']
    rotation=Rotation.from_matrix(basis.T@target[:3,:3]@anchor[:3,:3].T@basis).as_quat()
    def frame(seq,p,quat):
        return decode(make_packet('pitch',seq,now,p,quaternion=quat,squeeze=True,clutch_epoch=1,
                      schema_version=2,purpose='real',source_kind='live',trigger=.5),now,expected_schema=2)
    arm.anchor_frame=frame(1,[0.,0.,0.],[0.,0.,0.,1.]);arm.frame=frame(2,position,rotation)
    arm.anchor_tcp=anchor;arm.target=anchor.copy();arm.live=True;arm.latched=False
    arm.sequence=2;arm.first_sequence=1;arm.takeover=1
    arm.state=SimpleNamespace(clock_id=22,controller_epoch=1,monotonic_ns=now,feedback_valid=True,
        actual_positions=actual.tolist(),actual_velocities=[0.]*7,position_age=[0.]*7,feedback_age=[0.]*7,
        state=3,producer_id=arm.producer,session_id=identifier('pitch'),takeover_id=1,
        reference_positions=q[:6].tolist(),accepted_sequence=2,anchor_positions=q[:6].tolist(),
        anchor_monotonic_ns=now,gripper_reference_position=q[6],gripper_anchor_position=q[6])
    return arm,now,blocked


@pytest.mark.parametrize('direction', ['down','up'])
def test_lagging_base_or_shoulder_does_not_freeze_feasible_wrist_pitch(direction):
    arm,now,blocked=pitch_bridge(direction)
    before=np.r_[arm.state.reference_positions,arm.state.gripper_reference_position]
    ref,_=arm.tick(now,.008,enforce_budget=False)
    assert ref['valid'] and not arm.latched,arm.reason
    after=np.r_[ref['positions'],ref['gripper_position']]
    pitch_step=after[3]-before[3]
    assert (-pitch_step if direction=='down' else pitch_step)>1e-4, (direction,pitch_step,arm.reason)
    assert after[blocked]<=before[blocked]+1e-12
    assert np.max(np.abs(after[:6]-arm.state.actual_positions[:6]))<=.03+1e-12
    assert np.max(np.abs(after[:6]-before[:6]))<=.008+1e-12
    assert np.linalg.norm(arm.kin.error(after,arm.raw_target)[3:]) < np.linalg.norm(arm.kin.error(before,arm.raw_target)[3:])


@pytest.mark.parametrize('direction', ['down','up'])
def test_pitch_progress_continues_with_one_stationary_axis(direction):
    arm,now,blocked=pitch_bridge(direction)
    before=np.r_[arm.state.reference_positions,arm.state.gripper_reference_position]
    start_z=arm.kin.fk(before)[2,0]
    for _ in range(125):
        now+=8_000_000;arm.state.monotonic_ns=now
        arm.frame=replace(arm.frame,sequence=arm.frame.sequence+1,pose_sequence=arm.frame.pose_sequence+1,
                          pose_ns=now,event_ns=now,buttons_ns=now)
        actual=np.asarray(arm.state.actual_positions)
        previous=np.asarray(arm.state.reference_positions)
        for axis in range(6):
            if axis!=blocked: actual[axis]+=.5*(previous[axis]-actual[axis])
        arm.state.actual_positions=actual.tolist()
        ref,_=arm.tick(now,.008,enforce_budget=False)
        assert ref['valid'] and not arm.latched,arm.reason
        next_q=np.r_[ref['positions'],ref['gripper_position']]
        assert np.max(np.abs(next_q[:6]-actual[:6]))<=.03+1e-12
        assert np.max(np.abs(next_q[:6]-previous))<=.008+1e-12
        assert np.all(next_q>=arm.kin.lower) and np.all(next_q<=arm.kin.upper)
        arm.state.reference_positions=ref['positions'];arm.state.accepted_sequence=ref['sequence']
    end_z=arm.kin.fk(next_q)[2,0]
    assert (start_z-end_z if direction=='down' else end_z-start_z)>.1


@pytest.mark.parametrize('sign', [-1.,1.])
def test_existing_joint_ranges_allow_and_ik_reaches_vertical_gripper(sign):
    arm,_,_=pitch_bridge('down' if sign<0 else 'up')
    kin=arm.kin;q=np.asarray(arm.state.actual_positions)
    # A model-space witness with vertical tool X; its TCP position is also
    # specified, not an assertion that this attitude is reachable at every point.
    remainder=np.pi/2-1.55
    goal=np.array([0.,np.pi/3,np.pi/3+sign*remainder,sign*1.55,0.,0.,q[6]])
    target=kin.fk(goal)
    assert target[2,0]==pytest.approx(sign)
    for _ in range(1500):
        before=q.copy();q,_=kin.step(q,target,.008,allow_zero_tolerance=True)
        assert np.max(np.abs(q[:6]-before[:6]))<=.008+1e-12
        assert kin.feedback_in_bounds(q)
    error=kin.error(q,target)
    assert np.linalg.norm(error[:3])<1e-4
    assert np.linalg.norm(error[3:])<1e-3


def test_bounded_ik_handles_fixed_zero_boundary_without_outward_extension():
    arm,_,_=pitch_bridge('down')
    kin=arm.kin;q=np.array([0.,0.,1.,0.,0.,0.,-.015])
    measured=q[:6].copy();measured[1]=-.03
    goal=q.copy();goal[3]=-.1
    candidate,_=kin.step(q,kin.fk(goal),.008,True,tracking_position=measured,tracking_limit=.03)
    assert candidate[1]==0.
    assert candidate[3]<q[3]
    assert np.max(np.abs(candidate[:6]-measured))<=.03+1e-12


@pytest.mark.parametrize('measured,limit', [([0.]*5,.03),([float('nan')]*6,.03),([0.]*6,0.)])
def test_bounded_ik_rejects_invalid_tracking_limits(measured,limit):
    arm,_,_=pitch_bridge('down');kin=arm.kin;q=np.asarray(arm.state.actual_positions)
    with pytest.raises(ValueError,match='tracking bounds'):
        kin.step(q,kin.fk(q),.008,True,tracking_position=measured,tracking_limit=limit)
