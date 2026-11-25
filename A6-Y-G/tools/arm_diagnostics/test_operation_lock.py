"""Offline concurrency checks: these tests never open a serial device."""
from contextlib import ExitStack
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import operation_lock as locks


class OperationLockTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / 'operation.lock'
        self.patch = patch.object(locks, 'LOCK_PATH', self.path)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.left = dict(name='示教左臂', usb_serial='left', port=str(self.path.parent/'left'))
        self.right = dict(name='示教右臂', usb_serial='right', port=str(self.path.parent/'right'))

    def child(self, arm=None):
        code = ('from pathlib import Path\nimport operation_lock as locks\n'
                f'locks.LOCK_PATH = Path({str(self.path)!r})\n'
                'try:\n'
                f' with locks.{"exclusive_operation()" if arm is None else "arm_operation(" + repr(arm) + ")"}: pass\n'
                'except RuntimeError:\n raise SystemExit(42)\n')
        return subprocess.run([sys.executable, '-c', code], cwd=Path(__file__).parent,
                              capture_output=True, timeout=10)

    def assert_child(self, expected, arm=None):
        result = self.child(arm)
        self.assertEqual(result.returncode, expected, result.stderr.decode())

    def test_distinct_arms_coexist_and_global_operation_waits_for_all(self):
        with ExitStack() as sessions:
            sessions.enter_context(locks.arm_operation(self.left))
            self.assert_child(0, self.right)
            with locks.arm_operation(self.right):
                self.assert_child(42)
            self.assert_child(42)  # the first arm still holds its shared lock
        self.assert_child(0)

    def test_same_arm_blocks_same_process_and_other_process(self):
        with locks.arm_operation(self.left):
            with self.assertRaisesRegex(RuntimeError, '占用'):
                with locks.arm_operation(self.left):
                    self.fail('same arm acquired twice')
            self.assert_child(42, self.left)
        self.assert_child(0, self.left)

    def test_stable_usb_identity_blocks_changed_port(self):
        replugged = {**self.left, 'port': self.right['port']}
        with locks.arm_operation(self.left):
            self.assert_child(42, replugged)

    def test_aliases_or_conflicting_roles_cannot_share_endpoint(self):
        alias = self.path.parent / 'alias'
        alias.symlink_to(self.left['port'])
        same_device = {**self.right, 'port': str(alias)}
        with locks.arm_operation(self.left):
            self.assert_child(42, same_device)

    def test_global_operation_blocks_every_arm_and_legacy_operation(self):
        with locks.exclusive_operation():
            self.assert_child(42, self.left)
            self.assert_child(42, self.right)
            self.assert_child(42)
        self.assert_child(0, self.left)
        self.assert_child(0, self.right)

    def test_exception_releases_all_locks(self):
        with self.assertRaisesRegex(ValueError, 'test failure'):
            with locks.arm_operation(self.left):
                raise ValueError('test failure')
        self.assert_child(0, self.left)
        self.assert_child(0)

    def test_failed_partial_acquisition_does_not_leave_locks_held(self):
        alias = {**self.right, 'port': self.left['port']}
        with locks.arm_operation(self.left):
            with self.assertRaises(RuntimeError):
                with locks.arm_operation(alias):
                    self.fail('shared endpoint acquired twice')
            self.assert_child(0, self.right)
        self.assert_child(0)

    def test_missing_identity_rejected(self):
        for arm in ({}, {'port': '/unused'}, {'usb_serial': 'left'}):
            with self.subTest(arm=arm), self.assertRaises(ValueError):
                with locks.arm_operation(arm):
                    self.fail('incomplete identity accepted')
        self.assert_child(0)


if __name__ == '__main__':
    unittest.main()
