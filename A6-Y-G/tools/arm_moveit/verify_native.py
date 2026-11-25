#!/usr/bin/env python3
"""Integration of the user's C++ dry-run hardware, JTCs and real MoveIt actions."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import yaml

from native_session import PlanningSession


def wait(s, predicate, timeout=40, allow_error=False):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = s.snapshot()
        if predicate(state):
            return state
        if not state['running'] or (state['error'] and not allow_error):
            raise AssertionError(str(state['error']) + ' ' + str(state.get('directory')))
        time.sleep(.03)
    raise AssertionError('Timed out: ' + json.dumps(s.snapshot(), ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--screenshot')
    args = parser.parse_args()
    s = PlanningSession()
    s.start('motion_left', mock=True, with_rviz=args.rviz)
    try:
        ready = wait(s, lambda v: v['ready'] and v['target_valid'])
        print('Ready:', ready['directory'], flush=True)
        assert ready['controllers'] == {'joint_state_broadcaster': 'active', 'arm_controller': 'inactive', 'gripper_controller': 'inactive'}
        assert not ready['hardware']['enabled']
        folder = Path(ready['directory'])
        config = yaml.safe_load((folder / 'controllers_readonly.yaml').read_text())
        assert '<param name="dry_run">true</param>' in config[s.namespace + '/' + s.display.manager_name]['ros__parameters']['robot_description']
        q = s.runtime.measured()
        q[:3] += [.03, .02, .025]
        target = s.kin.fk(q)
        s.send('target', target)
        wait(s, lambda v: v['target_valid'] and v.get('target') == target.tolist())
        s.send('plan', {'velocity_scaling': .12, 'acceleration_scaling': .08})
        arm = wait(s, lambda v: v['plan_ready'])
        from rosidl_runtime_py.convert import message_to_ordereddict
        original_plan = message_to_ordereddict(s.plan)
        s.send('execute_arm')
        executed = wait(s, lambda v: v['hardware']['completed'] == 1)
        assert executed['hardware']['enabled']
        assert np.linalg.norm(s.kin.fk(s.runtime.measured())[:3, 3] - target[:3, 3]) < .001
        assert json.loads((folder / 'last_native_plan.json').read_text()) == original_plan
        config = yaml.safe_load((folder / 'controllers_active.yaml').read_text())
        assert '<param name="dry_run">true</param>' in config[s.namespace + '/' + s.display.manager_name]['ros__parameters']['robot_description']
        print('Arm: native MoveGroup -> ExecuteTrajectory -> arm_controller passed', flush=True)
        for count, opening in enumerate((0., 100., 30.), 2):
            before = s.runtime.measured()[:6].copy()
            s.send('gripper_target', opening)
            s.send('gripper_plan')
            wait(s, lambda v: v['plan_ready'] and v['plan_group'] == 'gripper')
            s.send('execute_gripper')
            wait(s, lambda v: v['hardware']['completed'] == count)
            assert abs(s.runtime.measured()[6] - s.gripper_target) < .0001
            assert np.max(abs(s.runtime.measured()[:6]-before)) < .001
        print('Gripper: close / open / intermediate all passed through gripper_controller', flush=True)
        if args.screenshot:
            time.sleep(.5)  # Let RViz receive the final status before capture.
            from PyQt5.QtWidgets import QApplication
            app = QApplication.instance() or QApplication([])
            app.primaryScreen().grabWindow(0).save(args.screenshot)
        # Test native action cancellation and holding, without tearing down ROS.
        q = s.runtime.measured()
        q[0] += .25
        s.send('target', s.kin.fk(q))
        s.send('plan')
        wait(s, lambda v: v['plan_ready'])
        s.send('execute_arm')
        wait(s, lambda v: v['hardware']['executing'])
        time.sleep(.15)
        s.send('stop')
        wait(s, lambda v: not v['hardware']['executing'] and not v['busy'], allow_error=True)
        assert s.snapshot()['running']
        assert all(p.poll() is None for name, p in s.display.children if not name.startswith('spawner_'))
        held = s.runtime.measured()
        time.sleep(.3)
        assert np.max(abs(held - s.runtime.measured())) < .01
        # Close must home through the same native planner and controller.
        s.stop()
        result = wait(s, lambda v: v['close_allowed'], timeout=90, allow_error=True)
        assert result['home_result'] is not None
        assert max(abs(x) for x in result['home_result']['joint_positions'][:6]) <= .005
        assert not result['hardware']['enabled']
        s.worker.join(15)
        assert not s.worker.is_alive()
        control_log = (folder / 'controller_manager.log').read_text()
        assert 'Goal reached, success!' in control_log
        assert 'Opened SLCAN bus' not in control_log
        result = {'result': 'PASS', 'backend': 'upstream C++ hardware dry_run + 100 Hz JTC + MoveIt',
                  'arm_and_gripper': True, 'cancel_and_hold': True, 'home_before_close': True,
                  'rviz': args.rviz, 'report': s.snapshot()['report']}
        (folder / 'verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    finally:
        s.abort()
        s.worker.join(15)


if __name__ == '__main__':
    main()
