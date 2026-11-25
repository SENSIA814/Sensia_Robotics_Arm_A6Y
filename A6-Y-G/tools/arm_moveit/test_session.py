"""Regression checks for asynchronous target changes and retained diagnostics."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from concurrent.futures import Future

import numpy as np

from hardware import ArmRuntime
from motion_model import Kinematics, MOCK_Q, planning_model
from planning_session import PlanningSession
from test_motion import small_trajectory


class SessionTests(unittest.TestCase):
    def session(self):
        s = PlanningSession()
        s.model = planning_model('motion_left')
        s.runtime = ArmRuntime(s.model)
        s.runtime.update(q=list(MOCK_Q), sample_time=time.monotonic())
        s.kin = Kinematics(s.model)
        s.target = s.kin.fk(MOCK_Q)
        s.revision, s.preview_revision = 1, 0
        s.preview_q = None
        s.plan = s.future = None
        s.preview_task = s.preview_check = s.preview_request = None
        s.plan_after_preview = ({'velocity_scaling': .5}, 1)
        s.update(ready=True, busy=True)
        return s

    def test_drag_supersedes_deferred_plan_without_leaving_ui_busy(self):
        s = self.session()
        target = s.target.copy()
        target[0, 3] += .01
        s._set_target(target)
        self.assertFalse(s.snapshot()['busy'])
        self.assertFalse(s.snapshot()['plan_ready'])
        self.assertIsNone(s.plan_after_preview)
        self.assertEqual(s.preview_request[0], s.revision)
        np.testing.assert_allclose(s.preview_request[1], target)
        self.assertIsNone(s.runtime.pending)

    def test_settings_and_gripper_cancel_deferred_arm_plan(self):
        for action in ('settings', 'gripper'):
            s = self.session()
            if action == 'settings':
                s.set_settings({'acceleration_scaling': .2})
                self.assertEqual(s.snapshot()['velocity_scaling'], .5)
                self.assertEqual(s.snapshot()['acceleration_scaling'], .2)
            else:
                s.set_gripper_target(40)
            self.assertFalse(s.snapshot()['busy'])
            self.assertIsNone(s.plan_after_preview)
            self.assertIsNone(s.runtime.pending)

    def test_later_display_failure_does_not_erase_planning_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            s = PlanningSession()
            s.update(directory=folder)
            s.update(error='夹爪起点超出范围')
            s.update(error='RViz exited -11')
            recorded = [json.loads(line)['error'] for line in (Path(folder) / 'events.jsonl').read_text().splitlines()]
            self.assertEqual(recorded, ['夹爪起点超出范围', 'RViz exited -11'])
            self.assertEqual(len(s.snapshot()['events']), 2)

    def test_readonly_unpermitted_exit_never_starts_home_or_enables(self):
        s = self.session()
        s.runtime.mock = False
        s.update(mock=False, permitted=False)
        s.request_home(close=True)
        self.assertTrue(s.cancel.is_set())
        self.assertTrue(s.snapshot()['close_allowed'])
        self.assertIsNone(s.home)
        self.assertIsNone(s.runtime.pending)

    def test_failed_home_cancels_exit_and_retains_owned_motor_hold(self):
        s = self.session()
        s.runtime.update(enabled=True)
        s.update(closing_requested=True, home_active=True)
        s.home = {'stage': 'planning'}
        s.future = Future()
        old = s.future
        s._fail_home('起点碰撞')
        self.assertTrue(old.cancelled())
        self.assertFalse(s.cancel.is_set())
        self.assertTrue(s.snapshot()['close_failed'])
        self.assertTrue(s.runtime.snapshot()['enabled'])
        self.assertFalse(s.runtime.release_requested.is_set())
        self.assertIsNone(s.home)

    def test_stop_cancels_late_home_plan_and_exit(self):
        s = self.session()
        s.update(closing_requested=True, home_active=True)
        s.home = {'stage': 'planning'}
        s.future = Future()
        old = s.future
        s._command('stop', None)
        self.assertTrue(old.cancelled())
        self.assertFalse(s.snapshot()['home_active'])
        self.assertTrue(s.snapshot()['close_failed'])
        self.assertFalse(s.cancel.is_set())
        self.assertIsNone(s.runtime.pending)

    def test_enabled_unpermitted_arm_cannot_bypass_home_on_exit(self):
        s = self.session()
        s.runtime.mock = False
        s.runtime.update(enabled=True)
        s.update(mock=False, permitted=False)
        with self.assertRaisesRegex(ValueError, '执行许可'):
            s.request_home(close=True)
        self.assertFalse(s.cancel.is_set())
        self.assertFalse(s.runtime.release_requested.is_set())

    def test_failed_execution_drops_pending_work_and_keeps_session_running(self):
        s = self.session()
        s.update(running=True, permitted=True, plan_ready=True)
        s.runtime.update(enabled=True)
        old = s.future = Future()
        generation = s.runtime.generation()
        q = np.array(MOCK_Q)
        plan = small_trajectory(s.model)
        s.runtime._reject_trajectory(plan, q, q[:6] + .2, .5, '轨迹跟踪偏差超过允许范围', plan.indices, plan.tracking_tolerance)
        s.commands.put(('execute', generation))
        s.commands.put(('home', None))
        s._handle_runtime_status(s.runtime.snapshot())
        self.assertTrue(old.cancelled())
        self.assertTrue(s.commands.empty())
        self.assertTrue(s.snapshot()['running'])
        self.assertTrue(s.snapshot()['ready'])
        self.assertFalse(s.snapshot()['permitted'])
        self.assertFalse(s.snapshot()['plan_ready'])
        self.assertFalse(s.cancel.is_set())
        self.assertTrue(s.runtime.snapshot()['enabled'])
        self.assertFalse(s.runtime.release_requested.is_set())
        s._handle_runtime_status(s.runtime.snapshot())
        self.assertEqual(len(s.snapshot()['events']), 1)
        self.assertEqual(s.snapshot()['events'][0]['execution_failure']['sequence'], 1)

    def test_home_tracking_failure_cancels_exit_without_closing(self):
        s = self.session()
        s.update(running=True, home_active=True, closing_requested=True)
        s.home = {'stage': 'executing'}
        s.runtime.update(enabled=True)
        plan = small_trajectory(s.model)
        q = np.array(MOCK_Q)
        s.runtime._reject_trajectory(plan, q, q[:6] + .2, .5, '轨迹跟踪偏差超过允许范围', plan.indices, plan.tracking_tolerance)
        s._handle_runtime_status(s.runtime.snapshot())
        self.assertTrue(s.snapshot()['close_failed'])
        self.assertFalse(s.snapshot()['closing_requested'])
        self.assertFalse(s.snapshot()['close_allowed'])
        self.assertFalse(s.snapshot()['home_active'])
        self.assertFalse(s.cancel.is_set())
        self.assertIsNone(s.home)

    def test_bus_fault_locks_motion_but_preserves_window_until_explicit_close(self):
        s = self.session()
        s.update(running=True, permitted=True)
        s.runtime.update(error='串口断开', q=None, release_confirmed=False, enabled=True)
        s._handle_runtime_status(s.runtime.snapshot())
        self.assertTrue(s.snapshot()['running'])
        self.assertFalse(s.snapshot()['ready'])
        self.assertFalse(s.snapshot()['permitted'])
        self.assertFalse(s.cancel.is_set())
        with self.assertRaisesRegex(ValueError, '硬件故障'):
            s.request_home(close=True)
        with self.assertRaisesRegex(ValueError, '硬件故障'):
            s._command('permit', True)
        self.assertFalse(s.snapshot()['close_allowed'])
        s.runtime.update(enabled=False, release_confirmed=True)
        s.request_home(close=True)
        self.assertTrue(s.snapshot()['close_allowed'])
        self.assertTrue(s.cancel.is_set())


if __name__ == '__main__':
    unittest.main()
