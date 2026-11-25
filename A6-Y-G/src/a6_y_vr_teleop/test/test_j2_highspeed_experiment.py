"""No CAN: J2 high-speed trajectory, homing envelope, and feedback units."""
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
import test_j2_highspeed as experiment

HOME=np.array([.020027,0.,.558289,-.567826,.058556,-.000191])
INITIAL=np.array([.057412,-.003624,.005531,-.300412,-.048638,-.063516,.00026])


def test_requested_profile_reaches_90_without_exceeding_acceleration():
    duration=experiment.validate_motion(90.,90.)
    assert duration==1.875
    t=np.linspace(0,duration,10001);dt=t[1]-t[0]
    q=np.array([experiment.base.smooth_position(0.,math.pi/2,x,duration) for x in t])
    v=np.diff(q)/dt
    assert abs(max(v)-math.pi/2)<1e-5
    assert max(abs(np.diff(v)/dt))<3.


@pytest.mark.parametrize('mode',['mock','real'])
def test_j2_limits_and_actual_native_feedback_ranges(tmp_path,mode):
    path,params,motors=experiment.make_configuration(tmp_path,mode,'/dev/NO_CAN',HOME,INITIAL)
    assert params['vr_reference_velocity']==[1.,1.65,1.,1.,1.,1.]
    assert params['position_velocity_limit']==[1.,1.65,1.,1.,1.,1.]
    assert params['vr_actual_velocity']==[1.25,1.9,1.25,1.25,1.25,1.25]
    assert params['vr_tracking_error']==.05 and params['vr_tracking_governor']
    assert params['vr_reference_acceleration']==[3.]*6
    assert motors['joints']['gripper_finger_joint1']['read_only']
    root=experiment.base.ET.fromstring(yaml.safe_load(path.read_text())['/j2_fast/controller_manager']['ros__parameters']['robot_description'])
    for n in ('joint4','joint5','joint6'):
        assert motors['joints'][n]['protocol_vmax']==50.
        assert root.find(f"ros2_control/joint[@name='{n}']/param[@name='protocol_vmax']").text=='50.0'
    assert root.find("ros2_control/joint[@name='joint2']/param[@name='max_velocity']").text=='1.65'
    assert root.find("ros2_control/hardware/param[@name='can_channel']").text==('/dev/THIS_IS_A_MOCK_NO_CAN' if mode=='mock' else '/dev/NO_CAN')


def test_homing_accepts_passive_wrist_drift_but_rejects_invalid_positions():
    params={'vr_lower':[-2.62,0.,0.,-1.55,-.9,-3.14],
            'vr_upper':[2.62,3.16,3.16,1.55,.9,3.14],
            'vr_zero_tolerance':[math.radians(2.)]*7}
    experiment.validate_initial_pose(INITIAL,params)
    end=np.r_[HOME,INITIAL[6]]
    experiment.validate_initial_pose(end,params)
    end[4]=.10624094  # observed passive position after first completed test
    experiment.validate_initial_pose(end,params)
    for axis,value in ((0,3.),(1,math.radians(105.)),(4,1.),(0,float('nan'))):
        bad=end.copy();bad[axis]=value
        with pytest.raises(ValueError):experiment.validate_initial_pose(bad,params)


def test_modified_home_is_rejected_before_hardware_start(tmp_path):
    changed=HOME.copy();changed[3]+=.1
    with pytest.raises(ValueError):experiment.make_configuration(tmp_path,'mock','/dev/NO',changed,INITIAL)


def test_forward_and_reverse_preview_only_moves_j2():
    end=HOME.copy();end[1]=math.pi/2
    for start,goal in ((HOME,end),(end,HOME)):
        for t in np.linspace(0,1.875,101):
            reference=experiment.base.smooth_position(start,goal,t,1.875)
            _,target=experiment.planned_goal(start,goal,t,1.875,reference,np.array([3.]*6))
            assert 0.<=target[1]<=math.pi/2
            np.testing.assert_array_equal(target[[0,2,3,4,5]],HOME[[0,2,3,4,5]])


@pytest.mark.parametrize('fault',['sweep','holding'])
def test_out_of_bounds_motion_stops_publication(fault):
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.state=None;e.epoch=1;e.active=True;e.test_started=True;e.home=HOME
    e.anchor=[0.]*7;e.abort=''
    s=SimpleNamespace(monotonic_ns=time.monotonic_ns(),controller_epoch=1,
        state=experiment.VrState.TELEOP,reason=4,feedback_valid=True,actual_positions=list(HOME)+[0.])
    if fault=='sweep':s.actual_positions[1]=math.radians(105.)
    else:s.actual_positions[0]+=math.radians(3.)
    e.observe(s)
    assert e.abort


def test_telemetry_and_motor_metrics_use_j2_not_j1():
    import io
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.state=None;e.epoch=1;e.active=False;e.home=HOME;e.anchor=list(HOME)+[0.]
    e.abort='';e.phase='outbound';e.goal=HOME.copy();e.trajectory=None;e.sequence=0;e.producer=1;e.session=1;e.edge=1;e.held=True
    e.hardware={'joint1':{'effort':1.},'joint2':{'effort':7.}};e.hardware_ns=time.monotonic_ns()
    e.record=io.StringIO();e.last_print=time.monotonic()
    e.reference_pub=SimpleNamespace(publish=lambda msg:None)
    s=SimpleNamespace(monotonic_ns=time.monotonic_ns(),controller_epoch=1,clock_id=1,takeover_id=1,
        state=experiment.VrState.TELEOP,accepted_sequence=1,reason=4,
        actual_positions=[.01,.5,.55,-.56,.05,0.,0.],reference_positions=[.02,.51,.55,-.56,.05,0.],
        actual_velocities=[.1,.9,0.,0.,0.,0.,0.],reference_velocities=[.0,1.,0.,0.,0.,0.],
        position_age=[0.]*7,feedback_age=[0.]*7)
    e.observe(s)
    row=json.loads(e.record.getvalue())
    assert row['actual']==.5 and row['reference']==.51 and row['actual_velocity']==.9
    assert row['j2_hardware']['effort']==7.


def test_settle_matches_controller_duration_with_quantized_wrist_velocity(monkeypatch):
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.params={'vr_settle_duration':.15,'vr_settle_speed':.03}
    now=[0.]
    monkeypatch.setattr(experiment.time,'monotonic',lambda:now[0])
    def wait(predicate, timeout, description):
        for k in range(126):
            now[0]=k*.008
            # Static encoders, but every 0.096 s the wrist estimator jumps by
            # one feedback bin. The controller still requires raw speed<.03.
            speed=.036630 if k%12==0 else .012210
            e.state=SimpleNamespace(monotonic_ns=int(now[0]*1e9),actual_positions=[0.]*7,reference_positions=[0.]*6,
                actual_velocities=[0.,0.,0.,0.,0.,speed,0.])
            if predicate():return
        raise RuntimeError('settle never reached')
    e.wait=wait
    e.settle(np.zeros(6),1.)
    assert .15<=now[0]<.2


@pytest.mark.parametrize('fault',[False,True])
@pytest.mark.parametrize('action,expected_state',[(1,0),(2,1),(5,2)])
def test_stationary_retry_only_handles_preparation_rejection(fault,action,expected_state,monkeypatch):
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.params={'vr_settle_speed':.03};e.active=False;e.abort=''
    e.state=SimpleNamespace(state=expected_state,feedback_valid=True,actual_velocities=[0.]*7)
    now=[0.];attempts=[]
    monkeypatch.setattr(experiment.time,'monotonic',lambda:now[0])
    e.wait=lambda pred,timeout,desc:pred()
    e.pump=lambda duration:now.__setitem__(0,now[0]+duration)
    def command(action):
        attempts.append(action)
        if len(attempts)==1:
            if fault:e.state.state=experiment.VrState.FAULT
            raise RuntimeError(f'command {action} rejected')
    e.command=command
    if fault:
        with pytest.raises(RuntimeError):e.command_when_stationary(action,expected_state)
        assert len(attempts)==1
    else:
        e.command_when_stationary(action,expected_state)
        assert len(attempts)==2


def test_encoder_settle_rejects_actual_motion():
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.params={'vr_settle_duration':.15,'vr_settle_speed':.03}
    def wait(predicate, timeout, description):
        for k in range(126):
            t=k*.008
            # Accurate positions are moving despite a misleading zero speed
            # field: the position window must prevent a false settled state.
            e.state=SimpleNamespace(monotonic_ns=int(t*1e9),actual_positions=[.04*t]+[0.]*6,
                reference_positions=[0.]*6,actual_velocities=[0.]*7)
            if predicate():return
        raise RuntimeError('not settled')
    e.wait=wait
    with pytest.raises(RuntimeError):e.settle(np.zeros(6),1.)
