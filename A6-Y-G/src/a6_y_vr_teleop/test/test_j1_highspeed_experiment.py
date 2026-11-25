"""No CAN: verify the operator-scoped 90 degree/s experiment."""
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import time

import numpy as np
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
import test_j1_highspeed as experiment


def test_90_degree_s_profile_has_requested_speed_and_bounded_acceleration():
    duration=experiment.validate_motion(90.,90.)
    assert duration==1.875
    t=np.linspace(0,duration,10001)
    q=np.array([experiment.base.smooth_position(0.,math.pi/2,x,duration) for x in t])
    dt=t[1]-t[0];dq=np.diff(q)/dt
    assert abs(max(dq)-math.pi/2)<1e-5
    assert max(abs(np.diff(dq)/dt))<3.
    assert np.all(dq>=0.)
    assert q[0]==0. and q[-1]==math.pi/2


@pytest.mark.parametrize('angle,speed',[(91,90),(-90,90),(90,91),(90,0),(0,90),(10,90),(90,float('nan'))])
def test_invalid_or_unachievable_profile_is_rejected(angle,speed):
    with pytest.raises(ValueError):experiment.validate_motion(angle,speed)


@pytest.mark.parametrize('mode',['mock','real'])
def test_only_j1_speed_limits_change_and_the_gripper_stays_readonly(tmp_path,mode):
    pose=np.array([-.02,-.004,.005,-.3,0.,-.02,.00026])
    anchor=tmp_path/'anchor.json';anchor.write_text(json.dumps({'model':list(pose)}))
    home,original=experiment.load_home(anchor)
    assert home[1]==0. and original[1]==-.004
    p,params,motors=experiment.make_configuration(tmp_path,mode,'/dev/NOT_OPENED',home,original)
    assert params['vr_reference_velocity']==[1.65,1.,1.,1.,1.,1.]
    assert params['position_velocity_limit']==[1.65,1.,1.,1.,1.,1.]
    assert params['vr_actual_velocity']==[1.9,1.25,1.25,1.25,1.25,1.25]
    assert params['vr_reference_acceleration']==[3.]*6
    assert params['vr_tracking_governor'] and params['vr_tracking_error']==.05
    assert params['vr_gripper_control'] is False
    assert {n for n,m in motors['joints'].items() if not m['read_only']}==set(experiment.JOINTS[:6])
    root=experiment.base.ET.fromstring(yaml.safe_load(p.read_text())['/j1_fast/controller_manager']['ros__parameters']['robot_description'])
    assert root.find("ros2_control/hardware/param[@name='control_mode']").text=='pos_vel'
    assert root.find("ros2_control/hardware/param[@name='can_channel']").text==('/dev/THIS_IS_A_MOCK_NO_CAN' if mode=='mock' else '/dev/NOT_OPENED')
    assert root.find("ros2_control/joint[@name='joint1']/param[@name='max_velocity']").text=='1.65'


@pytest.mark.parametrize('fault',['sweep','holding'])
def test_invalid_motion_stops_reference_publication(fault):
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.state=None;e.epoch=1;e.active=True;e.test_started=True;e.home=np.zeros(6)
    e.anchor=[0.]*7;e.abort=''
    s=SimpleNamespace(monotonic_ns=time.monotonic_ns(),controller_epoch=1,
        state=experiment.VrState.TELEOP,reason=4,feedback_valid=True,actual_positions=[0.]*6)
    if fault=='sweep':s.actual_positions[0]=math.radians(91.)
    else:s.actual_positions[3]=math.radians(3.)
    e.observe(s)
    assert e.abort


def test_preview_goals_stay_in_authorized_sweep_and_do_not_move_holding_axes():
    start=np.array([-.02,0.,.005,-.3,0.,-.02]);end=start.copy();end[0]+=math.pi/2
    duration=experiment.validate_motion(90.,90.)
    for t in np.linspace(0,duration,301):
        reference=experiment.base.smooth_position(start,end,t,duration)
        plan,goal=experiment.planned_goal(start,end,t,duration,reference,np.array([3.]*6))
        assert start[0]-1e-12<=goal[0]<=end[0]+1e-12
        np.testing.assert_array_equal(goal[1:],start[1:])
    assert goal[0]==end[0]


def test_low_speed_stage_also_caps_hardware_and_settling_speed(tmp_path):
    pose=np.array([-.02,0.,.005,-.3,0.,-.02,.00026])
    _,params,motors=experiment.make_configuration(tmp_path,'mock','/dev/NO',pose[:6],pose,20.)
    assert params['vr_reference_velocity'][0]<math.radians(22.)
    assert motors['joints']['joint1']['max_velocity']==params['position_velocity_limit'][0]


def test_lifecycle_success_alone_does_not_claim_requested_speed(tmp_path):
    import analyze_j1_highspeed as analysis
    criteria=dict(peak_speed_fraction=[.95,1.05],plan_error_max_deg=3.,
        hold_last_second_span_deg=.2,hold_mean_error_deg=.2,overshoot_deg=.5)
    report=dict(mode='real',passed=True,disarm_confirmed=True,planned_peak_speed_deg_s=90.,
        metrics=dict(encoder_peak_speed_deg_s=60.,plan_error_max_deg=2.,overshoot_deg=0.,
            holds={name:dict(span_deg=.02,mean_error_deg=.01) for name in ('outbound_hold','return_hold')}))
    path=tmp_path/'report.json';path.write_text(json.dumps(report))
    (tmp_path/'trace.jsonl').write_text('')
    result=analysis.summarize(path,criteria)
    assert result['passed'] and not result['performance_target_met']
    assert not result['criteria_checks']['speed']
    report['metrics']['encoder_peak_speed_deg_s']=89.
    path.write_text(json.dumps(report))
    assert analysis.summarize(path,criteria)['performance_target_met']
    report['passed']=False
    path.write_text(json.dumps(report))
    assert not analysis.summarize(path,criteria)['performance_target_met']
