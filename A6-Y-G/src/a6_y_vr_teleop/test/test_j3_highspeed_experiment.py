"""No CAN: verify the J3 adaptation targets the intended motor and envelope."""
import io
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
import test_j3_highspeed as experiment

HOME = np.array([-.039101,1.112573,.005531,-.228695,-.051308,-.104715])
INITIAL = np.array([.019646,-.004768,.005531,-.300031,.083352,.000191,.0002612])


@pytest.mark.parametrize('mode',['mock','real'])
def test_only_j3_speed_ceilings_change_and_native_ranges_match(tmp_path,mode):
    p,params,motors = experiment.make_configuration(tmp_path,mode,'/dev/NO_CAN',HOME,INITIAL)
    assert params['vr_reference_velocity'] == [1.,1.,1.65,1.,1.,1.]
    assert params['position_velocity_limit'] == [1.,1.,1.65,1.,1.,1.]
    assert params['vr_actual_velocity'] == [1.25,1.25,1.9,1.25,1.25,1.25]
    assert params['vr_tracking_error'] == .05 and params['vr_tracking_governor']
    assert params['vr_reference_acceleration'] == [3.]*6
    assert motors['joints']['joint2']['max_velocity'] == 1.
    assert motors['joints']['joint3']['max_velocity'] == 1.65
    assert motors['joints']['joint3']['protocol_vmax'] == 20.
    assert motors['joints']['gripper_finger_joint1']['read_only']
    data = yaml.safe_load(p.read_text())
    root = experiment.base.ET.fromstring(data['/j3_fast/controller_manager']['ros__parameters']['robot_description'])
    assert root.find("ros2_control/joint[@name='joint3']/param[@name='max_velocity']").text == '1.65'
    assert root.find("ros2_control/joint[@name='joint2']/param[@name='max_velocity']").text == '1.0'
    assert root.find("ros2_control/hardware/param[@name='can_channel']").text == ('/dev/THIS_IS_A_MOCK_NO_CAN' if mode=='mock' else '/dev/NO_CAN')


def test_profile_stays_in_old_sweep_and_other_axes_hold():
    duration = experiment.validate_motion(85.,90.)
    assert math.isclose(duration,1.875*85/90)
    end = HOME.copy();end[2] += math.radians(85.)
    assert end[2] < experiment.legacy.J3_MAX
    t = np.linspace(0,duration,10001)
    q = np.array([experiment.base.smooth_position(HOME,end,x,duration) for x in t])
    v = np.diff(q[:,2])/(t[1]-t[0])
    assert abs(max(v)-math.pi/2) < 1e-5
    assert max(abs(np.diff(v)/(t[1]-t[0]))) < 3.
    for begin,finish in ((HOME,end),(end,HOME)):
        for x in np.linspace(0,duration,101):
            reference = experiment.base.smooth_position(begin,finish,x,duration)
            _,goal = experiment.planned_goal(begin,finish,x,duration,reference,np.array([3.]*6))
            np.testing.assert_array_equal(goal[[0,1,3,4,5]],HOME[[0,1,3,4,5]])
            assert HOME[2] <= goal[2] <= end[2]


@pytest.mark.parametrize('travel,speed',[(90,90),(86,90),(0,5),(-1,5),(85,91),(10,90),(85,float('nan'))])
def test_reject_unconfirmed_travel_speed_and_acceleration(travel,speed):
    with pytest.raises(ValueError):experiment.validate_motion(travel,speed)


def test_home_must_be_the_previous_j3_pose(tmp_path):
    changed = HOME.copy();changed[1] += .1
    with pytest.raises(ValueError):experiment.make_configuration(tmp_path,'mock','/dev/NO',changed,INITIAL)
    p=tmp_path/'anchor.json';p.write_text(json.dumps(dict(home_model=list(HOME),initial_model=list(INITIAL))))
    home,initial = experiment.load_home(p)
    np.testing.assert_array_equal(home,HOME)
    np.testing.assert_array_equal(initial,INITIAL)


@pytest.mark.parametrize('axis,value',[(1,1.2),(2,1.6)])
def test_holding_j2_drift_or_j3_outside_sweep_stops_publication(axis,value):
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.state=None;e.epoch=1;e.active=True;e.test_started=True;e.home=HOME
    e.anchor=list(HOME)+[0.];e.abort=''
    q=list(HOME)+[0.];q[axis]=value
    e.observe(SimpleNamespace(monotonic_ns=time.monotonic_ns(),controller_epoch=1,
        state=experiment.VrState.TELEOP,reason=4,feedback_valid=True,actual_positions=q))
    assert e.abort


def test_telemetry_is_j3_and_not_j2():
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.state=None;e.epoch=1;e.active=False;e.home=HOME;e.anchor=list(HOME)+[0.]
    e.abort='';e.phase='outbound';e.goal=HOME.copy();e.goal[2]=.5;e.trajectory=None
    e.sequence=0;e.producer=1;e.session=1;e.edge=1;e.held=True
    e.hardware={'joint2':{'effort':2.},'joint3':{'effort':7.}};e.hardware_ns=time.monotonic_ns()
    e.record=io.StringIO();e.last_print=time.monotonic();sent=[]
    e.reference_pub=SimpleNamespace(publish=sent.append)
    s=SimpleNamespace(monotonic_ns=time.monotonic_ns(),controller_epoch=1,clock_id=1,takeover_id=1,
        state=experiment.VrState.TELEOP,accepted_sequence=1,reason=4,
        actual_positions=[-.04,1.112,.4,-.23,-.05,-.10,0.],reference_positions=list(e.goal),
        actual_velocities=[0.,.2,.7,0.,0.,0.,0.],reference_velocities=[0.,0.,.8,0.,0.,0.],
        position_age=[0.]*7,feedback_age=[0.]*7)
    e.observe(s)
    row=json.loads(e.record.getvalue())
    assert row['actual']==.4 and row['plan']==.5 and row['actual_velocity']==.7
    assert row['j3_hardware']['effort']==7.
    assert sent[0].positions[1]==HOME[1] and sent[0].positions[2]==.5
