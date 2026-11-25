"""Offline regressions for callbacks delayed by another arm's ROS shutdown."""
from collections import deque
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

from gravity_config import JOINTS
from gravity_session import GravitySession
from native_stack import NativeFeedback


class MultiFeedbackTests(unittest.TestCase):
    def session(self):
        session = GravitySession()
        session.update(mock=False)
        session.feedback = NativeFeedback({}, False)
        session.feedback.update(sample_source_time=None, diagnostic_source_time=None)
        return session

    def messages(self, age=0., fault=False):
        stamp = time.time()-age
        seconds = int(stamp)
        header = SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=int((stamp-seconds)*1e9)))
        joint = SimpleNamespace(header=header, name=list(JOINTS), position=[0.]*7, velocity=[0.]*7)
        interfaces = ['has_feedback','feedback_age','fault','motor_error']
        diagnostic = SimpleNamespace(header=header, joint_names=list(JOINTS), interface_values=[
            SimpleNamespace(interface_names=interfaces, values=[1.,.6 if fault else .01,0.,1. if i<6 else 0.])
            for i in range(7)])
        return joint, diagnostic

    def test_drain_backlog_after_other_arm_shutdown_updates_both_streams(self):
        session = self.session()
        old_joint, old_diagnostic = self.messages(age=2.)
        session._joint_states(old_joint)
        session._diagnostics(old_diagnostic)
        self.assertFalse(session._fresh(enabled=True))
        new_joint, new_diagnostic = self.messages()
        pending = deque([lambda:session._joint_states(old_joint)]*9 +
                        [lambda:session._diagnostics(old_diagnostic)]*9 +
                        [lambda:session._joint_states(new_joint),lambda:session._diagnostics(new_diagnostic)])
        waits = []
        def spin_once(timeout_sec):
            waits.append(timeout_sec)
            if pending: pending.popleft()()
        session.stack = SimpleNamespace(context=SimpleNamespace(ok=lambda:True))
        session.executor = SimpleNamespace(spin_once=spin_once)
        session._pump()
        self.assertFalse(pending)
        self.assertTrue(session._fresh(enabled=True))
        self.assertEqual(waits[0], .01)
        self.assertTrue(all(value == 0 for value in waits[1:]))
        self.assertLessEqual(len(waits), 33)

    def test_processing_old_messages_does_not_refresh_real_feedback(self):
        session = self.session()
        joint, diagnostic = self.messages(age=2.)
        session._joint_states(joint)
        session._diagnostics(diagnostic)
        state = session.feedback.snapshot()
        self.assertLess(time.monotonic()-state['sample_time'], .1)
        self.assertLess(time.monotonic()-state['diagnostic_time'], .1)
        self.assertFalse(session._fresh(enabled=True))

    def test_current_messages_cannot_hide_driver_feedback_loss(self):
        session = self.session()
        joint, diagnostic = self.messages(fault=True)
        session._joint_states(joint)
        session._diagnostics(diagnostic)
        self.assertFalse(session._fresh(enabled=True))

    def test_future_or_zero_stamp_does_not_authorize_output(self):
        session = self.session()
        for seconds in (0, int(time.time()+10)):
            joint, diagnostic = self.messages()
            joint.header.stamp.sec = seconds
            joint.header.stamp.nanosec = 0
            session._joint_states(joint)
            session._diagnostics(diagnostic)
            self.assertFalse(session._fresh(enabled=True))

    def test_pump_is_time_bounded_if_callback_is_slow(self):
        session = self.session()
        waits = []
        session.stack = SimpleNamespace(context=SimpleNamespace(ok=lambda:True))
        session.executor = SimpleNamespace(spin_once=lambda timeout_sec:waits.append(timeout_sec))
        with patch('gravity_session.time.monotonic', side_effect=[10.,10.021]):
            session._pump()
        self.assertEqual(waits, [.01])

    def test_worker_rechecks_arrivals_after_pause_between_pump_and_health_check(self):
        session = self.session()
        old_joint, old_diagnostic = self.messages(age=2.)
        session._joint_states(old_joint)
        session._diagnostics(old_diagnostic)
        joint, diagnostic = self.messages()
        def pending():
            session._joint_states(joint)
            session._diagnostics(diagnostic)
        with patch.object(session, '_pump', side_effect=pending) as pump:
            self.assertTrue(session._fresh_after_pending(enabled=True))
            pump.assert_called_once()
        session._joint_states(old_joint)
        session._diagnostics(old_diagnostic)
        with patch.object(session, '_pump') as pump:
            self.assertFalse(session._fresh_after_pending(enabled=True))
            pump.assert_called_once()


if __name__ == '__main__':
    unittest.main()
