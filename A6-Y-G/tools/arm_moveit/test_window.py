"""Offscreen desktop controls; no ROS processes and no hardware connection."""
from copy import deepcopy
import os
import time
import unittest
from types import SimpleNamespace
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtWidgets import QApplication
from moveit_view import MoveItWindow
from motion_model import MOCK_Q


class FakeSession:
    def __init__(self):
        self.worker = None
        self.commands = []
        self.state = {'running': False, 'ready': False, 'busy': False, 'mock': True,
                      'plan_ready': False, 'permitted': False, 'status': '', 'error': None}

    def snapshot(self):
        return deepcopy(self.state)

    def start(self, role, mock, collision_profile=None, home_on_start=False):
        self.state.update(role=role, mock=mock, running=True, ready=True,
                          home_on_start=home_on_start,
                          collision_profile=collision_profile,
                          hardware={'q': list(MOCK_Q), 'sample_time': time.monotonic(),
                                    'enabled': False, 'executing': False, 'status': 'ready'})
        self.worker = SimpleNamespace(is_alive=lambda: self.state['running'])

    def send(self, command, value=None):
        self.commands.append((command, value))
        if command == 'permit':
            self.state['permitted'] = value

    def stop(self):
        self.state.update(running=False, ready=False, busy=False, plan_ready=False)


class WindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)

    def test_no_default_arm_and_execution_needs_preview_and_permission(self):
        session = FakeSession()
        w = MoveItWindow(session)
        try:
            self.assertIsNone(w.selected())
            self.assertEqual(w.collision_profile.currentData(), 'bare')
            self.assertFalse(w.start_button.isEnabled())
            self.assertFalse(w.execute_button.isEnabled())
            self.assertEqual(w.mode.currentData(), True)
            self.assertIsNone(session.worker)
            self.assertFalse(w.position_only.isChecked())
            self.assertTrue(w.home_on_start.isChecked())
            w.home_on_start.setChecked(False)  # Exercise optional read-only mode.
            w.buttons['motion_right'].click()
            self.assertIn('link2↔link4', w.collision_hint.text())
            w.buttons['motion_left'].click()
            self.assertNotIn('link2↔link4', w.collision_hint.text())
            w.buttons['teaching_right'].click()
            self.assertNotIn('link2↔link4', w.collision_hint.text())
            w.mode.setCurrentIndex(1)
            w.start_button.click()
            w.tick()
            self.assertEqual(session.state['role'], 'teaching_right')
            self.assertEqual(session.state['collision_profile'], 'bare')
            self.assertFalse(session.state['home_on_start'])
            self.assertFalse(w.home_on_start.isEnabled())
            self.assertFalse(w.collision_profile.isEnabled())
            self.assertFalse(w.buttons['motion_left'].isEnabled())
            self.assertFalse(w.execute_button.isEnabled())
            session.state['plan_ready'] = True
            w.tick()
            self.assertFalse(w.execute_button.isEnabled())
            self.assertIn('等待授权', w.execution_hint.text())
            w.permission.setChecked(True)
            w.tick()
            self.assertTrue(w.execute_button.isEnabled())
            self.assertFalse(session.state['hardware']['enabled'])
            self.assertIn('自动激活', w.execution_hint.text())
            self.assertFalse(any(command.startswith('execute') for command, _ in session.commands))
            w.execute_button.click()
            self.assertIn(('execute', None), session.commands)
            session.state['plan_ready'] = False
            w.tick()
            self.assertFalse(w.execute_button.isEnabled())
            self.assertIn('重新规划', w.execution_hint.text())
            w.release_button.click()
            self.assertIn(('home', None), session.commands)
            w.stop_button.click()
            self.assertIn(('stop', None), session.commands)
            session.stop()
            w.tick()
            self.assertIsNone(w.selected())
            self.assertFalse(w.permission.isChecked())
        finally:
            session.stop()
            w.close()
            w.deleteLater()
            self.app.processEvents()

    def test_close_waits_for_home_and_stays_open_on_failure(self):
        class GracefulSession(FakeSession):
            def stop(self):
                self.commands.append(('shutdown', None))
                self.state.update(closing_requested=True, home_active=True, close_failed=False)
        session = GracefulSession()
        w = MoveItWindow(session)
        try:
            w.buttons['motion_left'].click()
            w.start_button.click()
            self.assertTrue(session.state['home_on_start'])
            w.close()
            self.assertTrue(w._closing)
            self.assertTrue(session.state['running'])
            self.assertIn(('shutdown', None), session.commands)
            session.state.update(close_failed=True, closing_requested=False, home_active=False)
            w.tick()
            self.assertFalse(w._closing)
            self.assertTrue(session.state['running'])
        finally:
            session.state.update(running=False, ready=False)
            w.close()
            w.deleteLater()
            self.app.processEvents()


if __name__ == '__main__':
    unittest.main()
