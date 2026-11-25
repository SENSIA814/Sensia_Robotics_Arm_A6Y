"""Check physical test configuration and bounded trajectories without opening CAN."""
import importlib.util
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
spec = importlib.util.spec_from_file_location('j2_experiment', ROOT/'tools/test_j2_position.py')
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
    root = ET.fromstring(yaml.safe_load(p.read_text())['/j2_test/controller_manager']['ros__parameters']['robot_description'])
    def hardware(k): return root.find(f"ros2_control/hardware/param[@name='{k}']").text
    assert hardware('defer_enable_until_controller_ready') == 'true'
    assert hardware('control_mode') == 'pos_vel'
    assert hardware('dry_run') == ('true' if mode=='mock' else 'false')
    assert hardware('can_channel') == ('/dev/THIS_IS_A_MOCK_NO_CAN' if mode=='mock' else '/dev/NOT_OPENED')
    np.testing.assert_allclose(experiment.home_from_motors(motors), [.020027,0.,.558289,-.567826,.058556,-.000191])


@pytest.mark.parametrize('start,end,duration', [(0,104,30),(-1,10,12),(30,0,12),(0,0,12),(0,30,1),(0,float('nan'),12),(0,100,2)])
def test_reject_outside_authorized_sweep_or_excessive_speed(start,end,duration):
    with pytest.raises(ValueError):experiment.validate_motion(start,end,duration)


def test_homing_profile_speed_is_bounded_and_test_motion_only_changes_j2():
    home=np.array([.020027,0.,.558289,-.567826,.058556,-.000191])
    start=np.array([.02575,-.003624,.00515,-.298505,.034905,.007057])
    duration=experiment.transfer_duration(start,home)
    assert 1.875*max(abs(home-start))/duration <= math.radians(2.5)+1e-12
    end=home.copy();end[1]=experiment.J2_MAX
    experiment.validate_motion(0.,math.degrees(experiment.J2_MAX),90.)
    for t in np.linspace(0,90,500):
        q=experiment.base.smooth_position(home,end,t,90.)
        np.testing.assert_array_equal(q[[0,2,3,4,5]],home[[0,2,3,4,5]])
        assert 0. <= q[1] <= experiment.J2_MAX+1e-12


@pytest.mark.parametrize('fault', ['controller_stop', 'holding_drift', 'j2_range'])
def test_abort_before_publishing_on_invalid_feedback(fault):
    e=experiment.Experiment.__new__(experiment.Experiment)
    e.state=None; e.epoch=1; e.active=True; e.test_started=True
    e.home=np.zeros(6); e.anchor=[0.]*7; e.abort=''
    s=SimpleNamespace(monotonic_ns=time.monotonic_ns(), controller_epoch=1,
        state=experiment.VrState.TELEOP, reason=4, feedback_valid=True, actual_positions=[0.]*6)
    if fault=='controller_stop':s.state=experiment.VrState.STOPPING
    if fault=='holding_drift':s.actual_positions[2]=math.radians(3.)
    if fault=='j2_range':s.actual_positions[1]=experiment.J2_MAX+.1
    # No publisher installed: any publication would fail this test.
    e.observe(s)
    assert e.abort
