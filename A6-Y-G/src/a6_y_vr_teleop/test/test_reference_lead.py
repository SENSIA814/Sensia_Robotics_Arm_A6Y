"""Offline regression for repeated short takeovers in the 19:27 physical trace."""
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from a6_y_vr_teleop.a5_kinematics import A5Kinematics
from a6_y_vr_teleop.input_protocol import Frame
from a6_y_vr_teleop.real_bridge import RealBridge

ROOT = Path(__file__).resolve().parents[3]


def recorded_lead_tick(soft=True):
    data=json.loads((ROOT/'validation/vr_position/lead_limit_repro_fixture.json').read_text())
    cfg=data['config'];cfg['limit_reference_lead']=soft
    kin=A5Kinematics(ROOT/'src/A5/urdf/A5.urdf',
                     ROOT/'src/a6_y_controllers/config/a5_vr_position_full_limits.yaml',cfg)
    arm=RealBridge(kin,None,cfg,'execute',clock=data['arm']['clock'])
    for name,value in data['arm'].items():
        if name in ('frame','previous','anchor_frame'):
            value['position']=np.asarray(value['position']);value['rotation']=np.asarray(value['rotation'])
            value=Frame(**value)
        elif name in ('anchor_tcp','target'): value=np.asarray(value)
        setattr(arm,name,value)
    arm.state=SimpleNamespace(**data['controller_state'])
    return arm,data['now_ns'],data['dt_s']


def test_recorded_fresh_input_limits_lead_without_latching():
    arm,now,dt=recorded_lead_tick()
    before=np.asarray(arm.state.reference_positions)
    actual=np.asarray(arm.state.actual_positions[:6])
    ref,control=arm.tick(now,dt,enforce_budget=False)
    assert ref['valid'] and not arm.latched, arm.reason
    assert control is None and arm.reason=='limited_reference_lead'
    command=np.asarray(ref['positions'])
    assert 0 < np.max(np.abs(command-before)) <= max(arm.kin.speed)*min(dt,.008)+1e-12
    assert np.max(np.abs(command-actual)) <= arm.cfg['max_joint_lead']+1e-12
    np.testing.assert_allclose(arm.target,arm.kin.fk(arm.preview),atol=1e-12)


def test_trial_policy_still_latches_on_excessive_lead():
    arm,now,dt=recorded_lead_tick(False)
    ref,_=arm.tick(now,dt,enforce_budget=False)
    assert not ref['valid'] and arm.latched and arm.reason=='reference_lead_exceeded'


def test_stationary_then_lagging_feedback_stays_bounded_and_resumes_without_regrip():
    arm,now,_=recorded_lead_tick()
    initial=np.asarray(arm.state.actual_positions).copy()
    peak_lead=0.;limited=0
    for index in range(300):
        now+=8_000_000
        arm.state.monotonic_ns=now
        arm.frame=replace(arm.frame,sequence=arm.frame.sequence+1,
                          pose_sequence=arm.frame.pose_sequence+1,
                          pose_ns=now,event_ns=now,buttons_ns=now)
        if index>=60:
            # A lagging motor, not the instantaneous position tracking of dry_run.
            actual=np.asarray(arm.state.actual_positions).copy()
            actual[:6]+=.10*(np.asarray(arm.state.reference_positions)-actual[:6])
            arm.state.actual_positions=actual.tolist()
        ref,_=arm.tick(now,.008,enforce_budget=False)
        assert ref['valid'] and not arm.latched, (index,arm.reason)
        lead=np.max(np.abs(np.asarray(ref['positions'])-arm.state.actual_positions[:6]))
        peak_lead=max(peak_lead,lead)
        assert lead<=.03+1e-12
        limited+=arm.reason=='limited_reference_lead'
        arm.state.reference_positions=ref['positions']
        arm.state.accepted_sequence=ref['sequence']
    assert limited>40 and peak_lead==pytest.approx(.03)
    assert np.max(np.abs(np.asarray(arm.state.actual_positions[:6])-initial[:6]))>.03


@pytest.mark.parametrize('sign', [-1.,1.])
@pytest.mark.parametrize('feedback_gap', [.025,.031,.06])
def test_lead_filter_never_increases_existing_out_of_band_gap_or_exceeds_step(sign,feedback_gap,monkeypatch):
    arm,now,dt=recorded_lead_tick()
    before=np.asarray(arm.state.reference_positions)
    arm.state.actual_positions[:6]=before.tolist()
    arm.state.actual_positions[0]=before[0]-sign*feedback_gap
    desired=np.r_[before,arm.state.gripper_reference_position]
    desired[0]+=sign*.008
    monkeypatch.setattr(arm.kin,'step',lambda *args,**kwargs:(desired.copy(),1.))
    ref,_=arm.tick(now,dt,enforce_budget=False)
    command=np.asarray(ref['positions'])
    assert ref['valid'] and not arm.latched
    assert abs(command[0]-arm.state.actual_positions[0])<=max(.03,feedback_gap)+1e-12
    assert abs(command[0]-before[0])<=.008+1e-12
    if feedback_gap>.03: np.testing.assert_array_equal(command,before)


@pytest.mark.parametrize('fault', ['old_input','old_feedback','bad_velocity','outside_joint_limit'])
def test_lead_limiting_does_not_clear_input_or_feedback_faults(fault):
    arm,now,dt=recorded_lead_tick()
    if fault=='old_input': arm.frame=replace(arm.frame,pose_ns=now-200_000_000)
    if fault=='old_feedback': arm.state.position_age[0]=.06
    if fault=='bad_velocity': arm.state.actual_velocities[0]=float('nan')
    if fault=='outside_joint_limit': arm.state.actual_positions[0]=arm.kin.upper[0]+.01
    ref,_=arm.tick(now,dt,enforce_budget=False)
    assert ref is None or not ref['valid']
    assert arm.latched
