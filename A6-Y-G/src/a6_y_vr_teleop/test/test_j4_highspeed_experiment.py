"""No CAN: check the J4 reverse mapping, confirmed envelope and held axes."""
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
import test_j4_highspeed as experiment

HOME = np.array([-.047494,1.033989,.005531,-.278286,-.026894,-.108530])
INITIAL = np.r_[HOME, .000261214]


@pytest.mark.parametrize('seconds',[-1., 0., 1., 16., float('nan'), float('inf')])
def test_observation_hold_duration_is_bounded(seconds):
    with pytest.raises(ValueError):
        experiment.validate_hold_seconds(seconds)


def test_longer_observation_hold_keeps_feedback_loop_and_target_active():
    e = experiment.Experiment.__new__(experiment.Experiment)
    e.hold_seconds = experiment.validate_hold_seconds(8.)
    e.active = True
    e.record = io.StringIO()
    events = []
    e.pump = lambda seconds: events.append((e.phase, seconds, e.active))
    e.settle = lambda goal, timeout: events.append(('settled', tuple(goal), timeout))
    end = HOME.copy(); end[3] += math.radians(90.)
    e.move('outbound', HOME, end, 33.75, 30.)
    assert events == [('outbound', 33.75, True), ('settled', tuple(end), 30.),
                      ('outbound_hold', 8., True)]
    marker = json.loads(e.record.getvalue())
    assert marker['kind'] == 'hold_begin' and marker['phase'] == 'outbound_hold'
    assert marker['seconds'] == 8.


def test_disarm_marker_does_not_replace_motor_disable(monkeypatch):
    e = experiment.Experiment.__new__(experiment.Experiment)
    e.record = io.StringIO()
    calls = []
    def disable(instance):
        calls.append(instance)
        return True
    monkeypatch.setattr(experiment.base.Experiment, 'disarm', disable)
    assert e.disarm() is True
    assert calls == [e]
    assert json.loads(e.record.getvalue())['kind'] == 'disarm_begin'


def hardware_message(enabled, effort=0.):
    return SimpleNamespace(joint_names=experiment.JOINTS, interface_values=[
        SimpleNamespace(interface_names=['enabled', 'effort'],
                        values=[float(enabled and i < 6), effort if i == 3 else 0.])
        for i in range(7)])


@pytest.mark.parametrize('acknowledge,disable', [(True, True), (True, False), (False, False)])
def test_disarm_effort_warning_waits_for_ack_and_all_disabled(monkeypatch, acknowledge, disable):
    # Replay the real failure ordering: tracking stop, DISARM publication,
    # 3.030525 Nm feedback, then acknowledgement and disabled feedback.
    e = experiment.Experiment.__new__(experiment.Experiment)
    e.mode = 'real'; e.node = object(); e.manager = SimpleNamespace(poll=lambda: None)
    e.record = io.StringIO(); e.epoch = 1; e.active = True; e.enabled_once = True
    e.abort = 'controller state=4 reason=11 feedback=True'; e.phase = 'outbound'
    e.anchor = list(INITIAL); e.goal = HOME.copy(); e.trajectory = None
    e.producer = e.session = e.edge = 1
    e.hardware = {}; e.hardware_ns = 0
    clock = [100.]; ticks = []
    monkeypatch.setattr(experiment.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(experiment.time, 'monotonic_ns', lambda: int(clock[0]*1e9))
    e.state = SimpleNamespace(monotonic_ns=int(clock[0]*1e9), controller_epoch=1,
        state=experiment.VrState.STOPPING, reason=11, feedback_valid=True,
        request_id=0, request_accepted=False, actual_velocities=[0.]*7)
    controls = []; references = []
    e.control_pub = SimpleNamespace(publish=controls.append)
    e.reference_pub = SimpleNamespace(publish=references.append)

    def spin(node, timeout_sec):
        clock[0] += .01; ticks.append(1)
        if len(ticks) == 1:
            e.observe_hw(hardware_message(True, 3.0305250305250304))
        elif len(ticks) == 2 and acknowledge:
            s = SimpleNamespace(**vars(e.state))
            s.monotonic_ns = int(clock[0]*1e9)
            s.request_id = controls[-1].request_id; s.request_accepted = True
            s.state = experiment.VrState.DISABLED; s.reason = 17
            e.observe(s)
        elif len(ticks) == 3 and disable:
            e.observe_hw(hardware_message(False))

    monkeypatch.setattr(experiment.base.rclpy, 'spin_once', spin)
    if acknowledge and disable:
        assert e.disarm() is True
    else:
        with pytest.raises(RuntimeError, match='timeout:'):
            e.disarm()
    assert len(controls) == 1 and controls[0].action == experiment.VrControl.DISARM
    assert references == []
    rows = [json.loads(line) for line in e.record.getvalue().splitlines()]
    assert any(r['kind'] == 'disarm_feedback_warning' for r in rows)
    assert any(r['kind'] == 'disarm_confirmed' for r in rows) == (acknowledge and disable)
    assert e.disarming is False


@pytest.mark.parametrize('active', [True, False])
@pytest.mark.parametrize('effort', [3.0305250305250304, float('nan'), float('inf')])
def test_effort_protection_still_aborts_before_disarm(active, effort):
    e = experiment.Experiment.__new__(experiment.Experiment)
    e.mode = 'real'; e.active = active; e.enabled_once = True; e.abort = ''
    e.disarming = False
    e.observe_hw(hardware_message(True, effort))
    assert e.abort == 'joint4 feedback effort invalid or exceeds 3.0 Nm'


def test_disarm_does_not_hide_a_controller_restart():
    e = experiment.Experiment.__new__(experiment.Experiment)
    e.state = None; e.epoch = 1; e.disarming = True; e.abort = ''
    e.observe(SimpleNamespace(monotonic_ns=time.monotonic_ns(), controller_epoch=2))
    assert e.abort == 'controller restarted'


@pytest.mark.parametrize('mode',['mock','real'])
def test_only_j4_speed_ceilings_change_and_native_ranges_match(tmp_path, mode):
    p,params,motors = experiment.make_configuration(tmp_path,mode,'/dev/NO_CAN',HOME,INITIAL)
    assert params['vr_reference_velocity'] == [1.,1.,1.,1.65,1.,1.]
    assert params['position_velocity_limit'] == [1.,1.,1.,1.65,1.,1.]
    assert params['vr_actual_velocity'] == [1.25,1.25,1.25,1.9,1.25,1.25]
    assert params['vr_tracking_error'] == .05 and params['vr_tracking_governor']
    assert params['vr_reference_acceleration'] == [3.]*6
    assert motors['joints']['joint3']['max_velocity'] == 1.
    assert motors['joints']['joint4']['max_velocity'] == 1.65
    assert motors['joints']['joint4']['protocol_vmax'] == 50.
    assert motors['joints']['joint4']['sign'] == -1.
    assert motors['joints']['gripper_finger_joint1']['read_only']
    assert experiment.EFFORT_LIMITS[3] == 3.
    np.testing.assert_array_equal(experiment.home_from_motors(motors),HOME)
    root = experiment.base.ET.fromstring(yaml.safe_load(p.read_text())['/j4_fast/controller_manager']['ros__parameters']['robot_description'])
    assert root.find("ros2_control/joint[@name='joint4']/param[@name='max_velocity']").text == '1.65'
    assert root.find("ros2_control/joint[@name='joint3']/param[@name='max_velocity']").text == '1.0'
    assert root.find("ros2_control/hardware/param[@name='can_channel']").text == ('/dev/THIS_IS_A_MOCK_NO_CAN' if mode=='mock' else '/dev/NO_CAN')


def test_model_positive_sweep_is_raw_negative_and_within_user_range():
    duration = experiment.validate_motion(90.,90.)
    assert duration == 1.875
    end=HOME.copy();end[3]+=math.pi/2
    assert end[3] < experiment.J4_MAX
    assert -1.370451 < -end[3] < .278286
    t=np.linspace(0,duration,10001)
    q=np.array([experiment.base.smooth_position(HOME,end,x,duration) for x in t])
    v=np.diff(q[:,3])/(t[1]-t[0])
    assert abs(max(v)-math.pi/2) < 1e-5
    assert max(abs(np.diff(v)/(t[1]-t[0]))) < 3.
    for begin,finish in ((HOME,end),(end,HOME)):
        for x in np.linspace(0,duration,101):
            reference=experiment.base.smooth_position(begin,finish,x,duration)
            _,goal=experiment.planned_goal(begin,finish,x,duration,reference,np.array([3.]*6))
            np.testing.assert_array_equal(goal[[0,1,2,4,5]], HOME[[0,1,2,4,5]])
            assert HOME[3] <= goal[3] <= end[3]


@pytest.mark.parametrize('speed,feedback_limit',[(2,1.),(10,1.),(20,1.),(30,1.),(45,1.),(50,1.9*50/90),(90,1.9)])
def test_low_speed_feedback_margin_never_changes_command_speed_or_high_speed_limit(tmp_path,speed,feedback_limit):
    path,params,motors=experiment.make_configuration(tmp_path,'mock','/dev/NO',HOME,INITIAL,speed)
    assert math.isclose(params['vr_actual_velocity'][3],feedback_limit)
    saved=yaml.safe_load(path.read_text())['/j4_fast/'+experiment.NAME]['ros__parameters']
    assert saved['vr_actual_velocity'] == params['vr_actual_velocity']
    expected=min(1.65,max(.1,1.65*speed/90.))
    assert params['vr_reference_velocity'][3] == expected
    assert params['position_velocity_limit'][3] == expected
    assert motors['joints']['joint4']['max_velocity'] == expected
    assert [v for i,v in enumerate(params['vr_actual_velocity']) if i != 3] == [1.25]*5
    assert params['vr_tracking_error'] == .05
    assert params['vr_reference_acceleration'] == [3.]*6
    assert experiment.EFFORT_LIMITS[3] == 3.


@pytest.mark.parametrize('travel,speed',[(94,90),(91,90),(0,5),(-1,5),(90,91),(10,90),(90,float('nan'))])
def test_reject_out_of_scope_travel_speed_and_acceleration(travel,speed):
    with pytest.raises(ValueError):experiment.validate_motion(travel,speed)


def test_confirmed_home_and_negative_start_are_supported(tmp_path):
    changed=HOME.copy();changed[2]+=.1
    with pytest.raises(ValueError):experiment.make_configuration(tmp_path,'mock','/dev/NO',changed,INITIAL)
    p=tmp_path/'anchor.json';p.write_text(json.dumps(dict(home_model=list(HOME),initial_model=list(INITIAL))))
    home,initial=experiment.load_home(p)
    np.testing.assert_array_equal(home,HOME)
    np.testing.assert_array_equal(initial,INITIAL)
    _,params,_=experiment.make_configuration(tmp_path,'mock','/dev/NO',HOME,INITIAL)
    experiment.validate_initial_pose(INITIAL,params)
    invalid=INITIAL.copy();invalid[3]=-.4
    with pytest.raises(ValueError):experiment.validate_initial_pose(invalid,params)


@pytest.mark.parametrize('axis,value',[(2,.1),(3,1.5),(3,-.4)])
def test_holding_j3_drift_or_j4_outside_sweep_stops_publication(axis,value):
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.state=None;e.epoch=1;e.active=True;e.test_started=True;e.home=HOME
    e.anchor=list(INITIAL);e.abort=''
    q=list(INITIAL);q[axis]=value
    e.observe(SimpleNamespace(monotonic_ns=time.monotonic_ns(),controller_epoch=1,
        state=experiment.VrState.TELEOP,reason=4,feedback_valid=True,actual_positions=q))
    assert e.abort


def test_published_goal_and_telemetry_target_j4_not_j3():
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.state=None;e.epoch=1;e.active=False;e.home=HOME;e.anchor=list(INITIAL)
    e.abort='';e.phase='outbound';e.goal=HOME.copy();e.goal[3]=.5;e.trajectory=None
    e.sequence=0;e.producer=1;e.session=1;e.edge=1;e.held=True
    e.hardware={'joint3':{'effort':7.},'joint4':{'effort':1.}};e.hardware_ns=time.monotonic_ns()
    e.record=io.StringIO();e.last_print=time.monotonic();sent=[]
    e.reference_pub=SimpleNamespace(publish=sent.append)
    actual=list(INITIAL);actual[3]=.4
    s=SimpleNamespace(monotonic_ns=time.monotonic_ns(),controller_epoch=1,clock_id=1,takeover_id=1,
        state=experiment.VrState.TELEOP,accepted_sequence=1,reason=4,
        actual_positions=actual,reference_positions=list(e.goal),
        actual_velocities=[0.,0.,.1,.7,0.,0.,0.],reference_velocities=[0.,0.,0.,.8,0.,0.],
        position_age=[0.]*7,feedback_age=[0.]*7)
    e.observe(s)
    row=json.loads(e.record.getvalue())
    assert row['actual']==.4 and row['plan']==.5 and row['actual_velocity']==.7
    assert row['j4_hardware']['effort']==1.
    assert sent[0].positions[2]==HOME[2] and sent[0].positions[3]==.5


def test_fault_triggering_state_is_recorded_once_without_publishing_motion():
    e = experiment.Experiment.__new__(experiment.Experiment)
    e.state = None; e.epoch = 1; e.active = True; e.test_started = True
    e.home = HOME; e.anchor = list(INITIAL); e.abort = ''
    e.phase = 'outbound'; e.fault_state_logged = False
    e.record = io.StringIO(); e.hardware = {'joint4': {'velocity': .3}}
    e.hardware_ns = time.monotonic_ns()-8_000_000
    sent = []; e.reference_pub = SimpleNamespace(publish=sent.append)
    stamp = time.monotonic_ns()
    s = SimpleNamespace(monotonic_ns=stamp, controller_epoch=1,
        state=experiment.VrState.FAULT, reason=13, feedback_valid=False,
        actual_positions=list(INITIAL), actual_velocities=[0., 0., 0., .525, 0., 0., 0.],
        reference_positions=list(HOME), reference_velocities=[0.]*6,
        position_age=[.008]*7, feedback_age=[.008]*7)
    e.observe(s)
    # Later FAULT publications must not overwrite the first failing sample.
    later = SimpleNamespace(**vars(s))
    later.monotonic_ns += 8_000_000
    later.actual_velocities = [0.]*7
    e.observe(later)
    rows = [json.loads(line) for line in e.record.getvalue().splitlines()]
    assert len(rows) == 1
    row = rows[0]
    assert row['kind'] == 'fault_state' and row['controller_monotonic_ns'] == stamp
    assert row['actual_velocities'][3] == .525
    assert row['actual_positions'] == list(INITIAL)
    assert row['reference_positions'] == list(HOME)
    assert row['hardware']['joint4']['velocity'] == .3
    assert row['hardware_received_monotonic_ns'] == e.hardware_ns
    assert row['feedback_age'] == [.008]*7
    assert row['reason'] == 13 and not row['feedback_valid']
    assert 't_ns' not in row  # Offline motion statistics must skip this diagnostic.
    assert e.abort and sent == []
