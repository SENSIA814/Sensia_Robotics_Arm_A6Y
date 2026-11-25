"""Regression tests for close/stop queues and orphan native process cleanup."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from native_session import PlanningSession, Cancelled
from native_stack import NativeFeedback, NativeStack


class LifecycleTests(unittest.TestCase):
    def session(self):
        s = PlanningSession()
        s.runtime = NativeFeedback({}, True)
        s.update(running=True)
        s._stop_action = lambda: None
        return s

    def test_repeated_close_merges_with_queued_home_without_canceling_it(self):
        s = self.session()
        s.send('home')
        generation = s.runtime.generation()
        for _ in range(40):
            s.send('shutdown')
        self.assertEqual(s.commands.qsize(), 1)
        self.assertEqual(s.runtime.generation(), generation)
        self.assertTrue(s.snapshot()['closing_requested'])

    def test_stop_discards_queued_home_and_execution_and_rejects_dequeued_home(self):
        s = self.session()
        s.send('home')
        command, value = s.commands.get_nowait()
        s.send('execute_arm')
        s.send('stop')
        self.assertEqual(s.commands.get_nowait()[0], 'stop')
        self.assertTrue(s.commands.empty())
        with self.assertRaises(Cancelled):
            s._command(command, value)
        self.assertFalse(s.home_queued)

    def test_cleanup_failure_cannot_skip_native_children_or_remaining_steps(self):
        s = self.session()
        calls = []
        def broken():
            calls.append('broken')
            raise RuntimeError('simulated invalid ROS handle')
        s._stop_action = broken
        s.display = SimpleNamespace(children=[], pump=None,
                                    terminate=lambda *_: calls.append('terminate'),
                                    close=lambda: calls.append('close'))
        s.marker_server = SimpleNamespace(shutdown=broken)
        s.executor = SimpleNamespace(shutdown=lambda **_: calls.append('executor'))
        s._cleanup()
        self.assertEqual(calls, ['broken', 'terminate', 'broken', 'executor', 'close'])
        self.assertEqual(len(s.snapshot()['cleanup_errors']), 2)

    def test_stop_cancels_close_even_before_home_worker_starts(self):
        s = self.session()
        s.send('shutdown')
        self.assertTrue(s.snapshot()['closing_requested'])
        s.send('stop')
        self.assertFalse(s.snapshot()['closing_requested'])
        self.assertTrue(s.snapshot()['close_failed'])
        self.assertEqual(s.commands.get_nowait()[0], 'stop')
        self.assertTrue(s.commands.empty())

    def test_invalid_motion_plan_names_logged_collision_without_disabling_checks(self):
        s = self.session()
        with tempfile.TemporaryDirectory() as folder:
            s.update(directory=folder)
            Path(folder, 'move_group.log').write_text("Found a contact between 'link1' (type 'Robot link') and 'link5' (type 'Robot link')")
            self.assertIn('link1 ↔ link5', s._planning_error(-2))

    def test_readonly_close_after_failed_plan_ignores_unused_permission(self):
        for mock in (False, True):
            for permitted in (False, True):
                with self.subTest(mock=mock, permitted=permitted):
                    s = self.session()
                    s.update(ready=True, mock=mock, permitted=permitted, close_failed=True,
                             home_stage='failed', error='MoveIt 规划失败', home_result=None)
                    s._discard_plan = Mock()
                    s._plan = Mock(side_effect=AssertionError('Must not plan on readonly exit'))
                    s._execute = Mock(side_effect=AssertionError('Must not move on readonly exit'))
                    s._switch_control = Mock(side_effect=AssertionError('Must not acquire motors'))
                    s.runtime.measured = Mock(side_effect=AssertionError('No feedback needed to close readonly'))
                    s._home(close=True)
                    state = s.snapshot()
                    self.assertTrue(state['close_allowed'] and s.cancel.is_set())
                    self.assertFalse(state['close_failed'] or state['home_active'])
                    self.assertEqual(state['home_stage'], 'not_required')
                    self.assertIsNone(state['home_result'])
                    s._discard_plan.assert_called_once()
                    s._plan.assert_not_called()
                    s._execute.assert_not_called()
                    s._switch_control.assert_not_called()

    def test_owned_executing_or_unconfirmed_release_never_bypasses_home(self):
        for hardware in ({'enabled':True, 'release_confirmed':False},
                         {'enabled':False, 'release_confirmed':False},
                         {'enabled':False, 'executing':True}):
            with self.subTest(hardware=hardware):
                s = self.session()
                s.update(ready=True, mock=False, permitted=True)
                s.runtime.update(**hardware)
                s.runtime.measured = Mock(side_effect=ValueError('feedback unavailable'))
                s._discard_plan = Mock()
                s._home(close=True)
                self.assertFalse(s.snapshot()['close_allowed'] or s.cancel.is_set())
                self.assertTrue(s.snapshot()['close_failed'])
                s.runtime.measured.assert_called_once()

    def test_manual_home_in_readonly_session_still_requests_motion(self):
        s = self.session()
        s.update(ready=True, mock=False, permitted=True)
        s.runtime.measured = Mock(side_effect=ValueError('feedback unavailable'))
        s._discard_plan = Mock()
        s._home(close=False)
        s.runtime.measured.assert_called_once()
        self.assertFalse(s.snapshot()['close_allowed'] or s.cancel.is_set())

    def test_parent_death_gracefully_ends_native_child(self):
        guard = Path(__file__).with_name('process_guard.py')
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            child = folder/'child.py'
            child.write_text("import os,signal,time\nfrom pathlib import Path\np=Path(__file__).parent\n(p/'child.pid').write_text(str(os.getpid()))\ndef stop(*_):\n (p/'stopped').write_text('SIGINT')\n raise SystemExit(0)\nsignal.signal(signal.SIGINT,stop)\nwhile True: time.sleep(.05)\n")
            parent = folder/'parent.py'
            parent.write_text(f"import os,subprocess,time\np=subprocess.Popen([{sys.executable!r},{str(guard)!r},str(os.getpid()),{sys.executable!r},{str(child)!r}])\nwhile True: time.sleep(.05)\n")
            process = subprocess.Popen([sys.executable, str(parent)], start_new_session=True)
            try:
                until = time.monotonic()+5
                while not (folder/'child.pid').exists() and time.monotonic()<until:
                    time.sleep(.02)
                self.assertTrue((folder/'child.pid').exists())
                process.kill()
                process.wait(timeout=2)
                until = time.monotonic()+5
                while not (folder/'stopped').exists() and time.monotonic()<until:
                    time.sleep(.02)
                self.assertEqual((folder/'stopped').read_text(), 'SIGINT')
            finally:
                try: os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                process.wait(timeout=2)

    def test_termination_deadline_applies_to_all_children_together(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'stubborn.py'
            path.write_text("import signal,time\nsignal.signal(signal.SIGINT,signal.SIG_IGN)\nsignal.signal(signal.SIGTERM,signal.SIG_IGN)\nwhile True: time.sleep(.05)\n")
            children=[(str(i),subprocess.Popen([sys.executable,str(path)],start_new_session=True)) for i in range(2)]
            try:
                time.sleep(.2)
                started=time.monotonic()
                NativeStack.terminate(children)
                self.assertLess(time.monotonic()-started, 7.)
                self.assertTrue(all(p.poll() is not None for _,p in children))
            finally:
                for _,p in children:
                    if p.poll() is None: p.kill()
                    p.wait(timeout=2)


if __name__ == '__main__':
    unittest.main()
