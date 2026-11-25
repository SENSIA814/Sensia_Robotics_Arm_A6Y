"""Check physical test configuration and bounded trajectories without opening CAN."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
spec = importlib.util.spec_from_file_location('j3_experiment', ROOT/'tools/test_j3_position.py')
experiment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(experiment)


@pytest.mark.parametrize('mode', ['mock','real'])
def test_six_arm_axes_hold_and_gripper_is_readonly(tmp_path, mode):
    p, params, motors = experiment.make_configuration(tmp_path,mode,'/dev/NOT_OPENED')
    assert {n for n,m in motors['joints'].items() if not m['read_only']} == set(experiment.JOINTS[:6])
    assert params['vr_gripper_control'] is False
    assert params['vr_auto_rearm_after_release'] is False
    assert params['vr_tracking_error'] == .05
    assert params['vr_tracking_governor'] is True
    root = ET.fromstring(yaml.safe_load(p.read_text())['/j3_test/controller_manager']['ros__parameters']['robot_description'])
    def hardware(k): return root.find(f"ros2_control/hardware/param[@name='{k}']").text
    assert hardware('defer_enable_until_controller_ready') == 'true'
    assert hardware('control_mode') == 'pos_vel'
    assert hardware('dry_run') == ('true' if mode=='mock' else 'false')
    assert hardware('can_channel') == ('/dev/THIS_IS_A_MOCK_NO_CAN' if mode=='mock' else '/dev/NOT_OPENED')
    assert root.find("ros2_control/joint[@name='joint3']/param[@name='protocol_vmax']").text == '20.0'
    np.testing.assert_allclose(experiment.home_from_motors(motors), [-.039101,1.112573,.005531,-.228695,-.051308,-.104715])


@pytest.mark.parametrize('start,end,duration', [(0,104,30),(-1,10,12),(30,0,12),(0,0,12),(0,30,1),(0,float('nan'),12),(0,100,2)])
def test_reject_outside_authorized_sweep_or_excessive_speed(start,end,duration):
    with pytest.raises(ValueError):experiment.validate_motion(start,end,duration)


def test_homing_profile_speed_is_bounded_and_test_motion_only_changes_j3():
    home=np.array([-.039101,1.112573,.005531,-.228695,-.051308,-.104715])
    start=np.array([-.041009,1.031319,.005531,-.277523,-.026513,-.101282])
    duration=experiment.transfer_duration(start,home)
    assert 1.875*max(abs(home-start))/duration <= math.radians(2.5)+1e-12
    end=home.copy();end[2]=experiment.J3_MAX
    experiment.validate_motion(experiment.J3_START,math.degrees(experiment.J3_MAX),90.)
    for t in np.linspace(0,90,500):
        q=experiment.base.smooth_position(home,end,t,90.)
        np.testing.assert_array_equal(q[[0,1,3,4,5]],home[[0,1,3,4,5]])
        assert 0. <= q[2] <= experiment.J3_MAX+1e-12


@pytest.mark.parametrize('fault', ['controller_stop', 'holding_drift', 'j3_range'])
def test_abort_before_publishing_on_invalid_feedback(fault):
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.state=None; e.epoch=1; e.active=True; e.test_started=True
    e.home=np.zeros(6); e.anchor=[0.]*7; e.abort=''
    s=SimpleNamespace(monotonic_ns=time.monotonic_ns(), controller_epoch=1,
        state=experiment.VrState.TELEOP, reason=4, feedback_valid=True, actual_positions=[0.]*6)
    if fault=='controller_stop':s.state=experiment.VrState.STOPPING
    if fault=='holding_drift':s.actual_positions[1]=math.radians(3.)
    if fault=='j3_range':s.actual_positions[2]=experiment.J3_MAX+.1
    # No publisher installed: any publication would fail this test.
    e.observe(s)
    assert e.abort


def test_partial_trial_keeps_telemetry_when_abort_diagnostic_is_present(tmp_path):
    """A timeout record must not hide the measurements preceding the stop."""
    rows=[]
    for i,phase in enumerate(('outbound','outbound_hold','return','return_hold')):
        rows.append(dict(t_ns=1_000_000_000+i*10_000_000,phase=phase,
            plan=0.,reference=0.,actual=0.,plan_all=[0.]*6,
            reference_all=[0.]*6,actual_all=[0.]*6,hardware={},j3_hardware={},reason=4))
    rows.append(dict(kind='abort_diagnostics',phase='return_hold',error='feedback expired'))
    (tmp_path/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    metrics=experiment.write_analysis(tmp_path,[],np.zeros(6))
    assert metrics['samples']==4
    assert metrics['plan_error_max_deg']==0.
    assert (tmp_path/'tracking.png').exists()
    (tmp_path/'report.json').write_text(json.dumps(dict(passed=False,
        disarm_confirmed=True,error='feedback expired',metrics=metrics)))
    import analyze_j3_tuning
    result=analyze_j3_tuning.summarize(tmp_path/'report.json')
    assert result['passed'] is False
    assert result['tracking_target_met'] is False
    assert len((tmp_path/'trace.csv').read_text().splitlines())==5
