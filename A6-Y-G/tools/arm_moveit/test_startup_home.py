"""Startup cancellation, home reference and gripper endpoint regressions."""
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import numpy as np

from motion_model import ROLE_ORDER, JOINTS, Kinematics
from native_model import native_model
from native_session import PlanningSession, Cancelled
from native_stack import NativeFeedback
from gripper_policy import load_limits, position_from_percent, percent_from_position, validate_trajectory


class StartupTests(unittest.TestCase):
    def session(self):
        s = PlanningSession()
        s.model = native_model('motion_left')
        s.kin = Kinematics(s.model)
        s.runtime = NativeFeedback(s.model, True)
        s.runtime.measured = Mock(return_value=np.array([0.]*6+[-.01]))
        s.startup_generation = s.runtime.generation()
        s._stop_action = Mock()
        s.update(running=True, ready=True, home_on_start=True)
        return s

    def test_startup_at_zero_holds_and_records_one_measured_reference(self):
        s = self.session()
        s.reset_target = Mock()
        s._switch_control = Mock()
        s._plan = Mock()
        s._startup_home()
        self.assertEqual(s.snapshot()['startup_home'], 'complete')
        self.assertTrue(s.snapshot()['permitted'])
        s._plan.assert_not_called()
        s._switch_control.assert_called_once_with(True, s.startup_generation)
        reference = s.runtime.snapshot()['plot_zero_pose']
        np.testing.assert_allclose(reference, s.kin.fk(s.runtime.measured()))
        s._set_plot_zero(np.array([.2]*6+[-.01]))
        self.assertEqual(reference, s.runtime.snapshot()['plot_zero_pose'])

    def test_initialization_stop_close_and_permission_revoke_prevent_auto_enable(self):
        for command, value in [('stop', None), ('shutdown', None), ('permit', False)]:
            with self.subTest(command=command):
                s = self.session()
                s._home = Mock()
                s.send(command, value)
                s._startup_home()
                s._home.assert_not_called()
                self.assertFalse(s.snapshot()['permitted'])
                self.assertEqual(s.snapshot()['startup_home'], 'cancelled')

    def test_failed_home_keeps_session_and_never_sets_fake_zero(self):
        s = self.session()
        s.runtime.measured.side_effect = ValueError('feedback unavailable')
        s._startup_home()
        self.assertEqual(s.snapshot()['startup_home'], 'failed')
        self.assertTrue(s.snapshot()['running'])
        self.assertFalse(s.snapshot()['busy'])
        self.assertIsNone(s.runtime.snapshot().get('plot_zero_pose'))
        self.assertTrue(s.commands.empty())

    def test_stop_between_initial_check_and_home_retains_pending_command(self):
        s = self.session()
        def stop(**_):
            s.send('stop')
            raise Cancelled('stop')
        s._home = stop
        s._startup_home()
        self.assertTrue(s.snapshot()['running'])
        self.assertEqual(s.commands.get_nowait()[0], 'stop')
        self.assertFalse(s.snapshot()['busy'])

    def test_cancelled_acquisition_does_not_restart_or_enable_controller(self):
        s = self.session()
        s.display = SimpleNamespace(stop_control=Mock(), start_control=Mock())
        s.runtime.cancel_motion()
        with self.assertRaises(Cancelled):
            s._switch_control(True, s.startup_generation)
        s.display.stop_control.assert_not_called()
        s.display.start_control.assert_not_called()
        self.assertFalse(s.runtime.snapshot()['enabled'])


class GripperLimitsTests(unittest.TestCase):
    def test_four_roles_inset_goals_feedback_unclamped_and_invalid_percent_rejected(self):
        for role in ROLE_ORDER:
            model = native_model(role)
            limits = model['gripper_limits']
            self.assertAlmostEqual(position_from_percent(limits, 0), -.0007)
            self.assertAlmostEqual(position_from_percent(limits, 100), -.0343)
            for percent in (0, 35, 100):
                self.assertAlmostEqual(percent_from_position(limits, position_from_percent(limits, percent)), percent)
            self.assertLess(percent_from_position(limits, 0), 0)
            self.assertGreater(percent_from_position(limits, -.035), 100)
            for bad in (-1, 101, float('nan'), float('inf')):
                with self.assertRaises(ValueError):
                    position_from_percent(limits, bad)
            self.assertEqual(model['joints'][JOINTS[6]]['lower'], -.035)
            self.assertEqual(model['joints'][JOINTS[6]]['upper'], 0)

    def test_bad_margins_rejected_and_per_role_override_supported(self):
        joint = native_model('motion_left')['joints'][JOINTS[6]]
        for margin in (-.1, 0, 18, float('inf')):
            config = dict(schema_version=1, defaults=dict(open_margin_mm=margin, closed_margin_mm=margin))
            with patch.object(Path, 'read_text', return_value=json.dumps(config)), self.assertRaises(ValueError):
                load_limits('motion_left', joint)
        config = dict(schema_version=1, defaults=dict(open_margin_mm=.7, closed_margin_mm=.7),
                      role_overrides=dict(motion_right=dict(open_margin_mm=1.)))
        with patch.object(Path, 'read_text', return_value=json.dumps(config)):
            self.assertAlmostEqual(load_limits('motion_right', joint)['open_position_m'], -.034)
            self.assertAlmostEqual(load_limits('motion_left', joint)['open_position_m'], -.0343)

    def test_outer_start_can_only_return_inward_and_goal_must_be_inside(self):
        limits = native_model('motion_left')['gripper_limits']
        def check(values, start):
            t = SimpleNamespace(joint_names=[JOINTS[6]], points=[SimpleNamespace(positions=[v]) for v in values])
            validate_trajectory(limits, t, start)
        check([0, -.0002, -.0007], 0)
        check([-.035, -.0348, -.0343], -.035)
        check([-.01, -.02, -.0343], -.01)
        for values, start in [([0, .0001, -.0007], 0), ([-.01, -.035, -.02], -.01),
                              ([-.01, 0], -.01), ([0, -.0005, -.0002, -.0007], 0)]:
            with self.assertRaises(ValueError):
                check(values, start)


if __name__ == '__main__':
    unittest.main()
