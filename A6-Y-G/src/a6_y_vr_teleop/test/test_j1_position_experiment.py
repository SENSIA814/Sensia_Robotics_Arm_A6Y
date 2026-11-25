"""No hardware: verify the dedicated experiment's motion and device isolation."""
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('j1_experiment', ROOT/'tools/test_j1_position.py')
experiment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(experiment)


@pytest.mark.parametrize('direction', [-1., 1.])
def test_quintic_is_bounded_monotonic_and_has_smooth_endpoints(direction):
    start, end, duration = .2, .2+direction*math.pi/6, 12.
    t = np.linspace(0., duration, 1501)
    q = np.array([experiment.smooth_position(start, end, x, duration) for x in t])
    dq = np.diff(q)/.008
    assert q[0] == start and abs(q[-1]-end) < 1e-12
    assert np.all(direction*dq >= -1e-12)
    assert np.max(np.abs(dq)) <= 1.875*abs(end-start)/duration
    assert abs(dq[0]) < 1e-5 and abs(dq[-1]) < 1e-5
    assert np.max(np.abs(np.diff(dq)/.008)) < .03
    assert experiment.smooth_position(start,end,-1.,duration) == start
    assert abs(experiment.smooth_position(start,end,100.,duration)-end) < 1e-12


@pytest.mark.parametrize('delta,duration', [(0.,12.), (31.,12.), (-31.,12.),
                                         (float('nan'),12.), (30.,1.), (30.,float('inf'))])
def test_reject_unbounded_or_nonfinite_experiments(delta,duration):
    with pytest.raises(ValueError):experiment.validate_motion(delta,duration)


@pytest.mark.parametrize('mode', ['mock','real'])
def test_only_j1_is_commandable_and_dedicated_test_keeps_tracking_limits(tmp_path,mode):
    p,params,motors = experiment.make_configuration(tmp_path,mode,'/dev/NOT_OPENED')
    assert [n for n,m in motors['joints'].items() if not m['read_only']] == ['joint1']
    assert params['vr_allow_test_input'] is True
    assert params['vr_auto_rearm_after_release'] is False
    assert params['vr_gripper_control'] is False
    assert params['vr_tracking_error'] == .05
    assert params['vr_tracking_governor'] is True
    assert params['vr_reference_velocity'] == [1.]*6
    root = ET.fromstring(yaml.safe_load(p.read_text())['/j1_test/controller_manager']['ros__parameters']['robot_description'])
    def hardware(key):return root.find(f"ros2_control/hardware/param[@name='{key}']").text
    assert hardware('dry_run') == ('true' if mode=='mock' else 'false')
    assert hardware('can_channel') == ('/dev/THIS_IS_A_MOCK_NO_CAN' if mode=='mock' else '/dev/NOT_OPENED')
    assert hardware('defer_enable_until_controller_ready') == 'true'
    assert hardware('control_mode') == 'pos_vel'
    for name in experiment.JOINTS:
        value = root.find(f"ros2_control/joint[@name='{name}']/param[@name='read_only']").text
        assert value.lower() == ('false' if name=='joint1' else 'true')
