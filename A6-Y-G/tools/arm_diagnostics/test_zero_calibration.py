"""Zero-write integration, failure and exclusivity checks without hardware."""
from contextlib import contextmanager
import errno
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import can

import operation_lock
import scanner
import zero_calibration as zero


def feedback(motor_id, raw, state=0, velocity=2047):
    return can.Message(arbitration_id=motor_id + 16, is_extended_id=False,
                       data=[state << 4 | motor_id, raw >> 8, raw & 255,
                             velocity >> 4, (velocity & 15) << 4 | 7, 255, 25, 26])


class MotorBus:
    def __init__(self, missing=None, enabled=None, moving=None, fail_write=None, stale=False, no_post_reply=False):
        self.sent, self.pending = [], []
        self.zeros = set()
        self.counts = {i: 0 for i in scanner.IDS}
        self.missing, self.enabled, self.moving = missing, enabled, moving
        self.fail_write, self.stale, self.no_post_reply = fail_write, stale, no_post_reply

    def send(self, msg, timeout):
        self.sent.append(msg)
        if bytes(msg.data) == zero.ZERO_PAYLOAD:
            i = msg.arbitration_id
            self.zeros.add(i)
            if i == self.fail_write:
                raise OSError('simulated transport uncertainty')
            # Some firmware replies to FE; it must not replace fresh readback.
            self.pending.append(feedback(i, 32768))
            return
        assert msg.arbitration_id == 0x7ff
        i = msg.data[0]
        assert bytes(msg.data) == scanner.status_payload(i)
        self.counts[i] += 1
        if i == self.missing or (self.no_post_reply and i in self.zeros):
            return
        raw = 32768 if i in self.zeros and not self.stale else 40000
        if i == self.moving and self.counts[i] > 1:
            raw += 1000
        self.pending.append(feedback(i, raw, 1 if i == self.enabled else 0))

    def recv(self, timeout):
        if self.pending:
            return self.pending.pop(0)
        time.sleep(timeout)
        return None

    def shutdown(self):
        pass


class ZeroTests(unittest.TestCase):
    def run_fake(self, bus=None, cancel=None, emit=None):
        bus = bus or MotorBus()
        @contextmanager
        def opened(arm, control_type):
            self.assertIs(control_type, zero.ZeroCalibrationControl)
            yield control_type(bus=bus, tx=arm['tx'])
        with patch.object(zero, 'open_arm_control', side_effect=opened), \
                patch.object(zero, 'STATUS_TIMEOUT', .001), patch.object(zero, 'SETTLE_SECONDS', 0):
            result = zero.calibrate_zero('motion_left', scanner.load_roles(), cancel=cancel, emit=emit)
        return result, bus

    def test_full_arm_checks_every_motor_before_writing_and_verifies_twice(self):
        report, bus = self.run_fake()
        self.assertTrue(report['ok'])
        arm = report['arms'][0]
        self.assertEqual(arm['verified_ids'], list(range(1, 8)))
        self.assertEqual(len(bus.sent), 42)
        self.assertTrue(all(m.arbitration_id == 0x7ff for m in bus.sent[:14]))
        writes = [m for m in bus.sent if bytes(m.data) == zero.ZERO_PAYLOAD]
        self.assertEqual([m.arbitration_id for m in writes], list(range(1, 8)))
        self.assertTrue(all(len(m['after_samples']) == 2 and m['last']['position_raw'] == 32768
                            and m['prewrite']['position_raw'] == 40000 for m in arm['motors']))
        self.assertFalse(report['power_cycle_verified'])

    def test_missing_enabled_or_moving_motor_prevents_all_writes(self):
        for option in ({'missing': 7}, {'enabled': 5}, {'moving': 4}):
            with self.subTest(option=option):
                report, bus = self.run_fake(MotorBus(**option))
                self.assertFalse(report['ok'])
                self.assertFalse(bus.zeros)
                self.assertTrue(report['arms'][0]['error'])
                self.assertEqual(report['arms'][0]['not_written_ids'], list(range(1, 8)))

    def test_stale_or_absent_readback_does_not_report_success_or_continue_writing(self):
        for option in ({'stale': True}, {'no_post_reply': True}):
            with self.subTest(option=option):
                report, bus = self.run_fake(MotorBus(**option))
                self.assertFalse(report['ok'])
                self.assertEqual(bus.zeros, {1})
                self.assertEqual(report['arms'][0]['uncertain_ids'], [1])
                self.assertEqual(report['arms'][0]['verified_ids'], [])

    def test_send_error_preserves_partial_and_uncertain_outcome_without_retry(self):
        report, bus = self.run_fake(MotorBus(fail_write=2))
        arm = report['arms'][0]
        self.assertFalse(report['ok'])
        self.assertEqual(arm['verified_ids'], [1])
        self.assertEqual(arm['uncertain_ids'], [2])
        self.assertEqual(arm['not_written_ids'], [3, 4, 5, 6, 7])
        self.assertEqual(len([m for m in bus.sent if bytes(m.data) == zero.ZERO_PAYLOAD]), 2)
        self.assertFalse(arm['tx'][-1]['sent'])

    def test_cancel_before_write_and_after_first_write(self):
        for stop_after_write in (False, True):
            with self.subTest(after_write=stop_after_write):
                cancel = threading.Event()
                def emit(event):
                    if event['event'] == 'zero_progress':
                        motor = event['motor']
                        if (stop_after_write and motor['zero_command_sent']) or (not stop_after_write and motor['id'] == 7 and motor['result'] == 'zero_ready'):
                            cancel.set()
                report, bus = self.run_fake(cancel=cancel, emit=emit)
                self.assertTrue(report['cancelled'])
                self.assertEqual(bus.zeros, {1} if stop_after_write else set())
                self.assertEqual(report['arms'][0]['verified_ids'], [1] if stop_after_write else [])

    def test_separate_command_allowlists_and_no_zero_retries(self):
        bus = MotorBus()
        control = zero.ZeroCalibrationControl(bus=bus)
        for can_id, payload in ((1, b'\xff'*7+b'\xfc'), (1, b'\xff'*7+b'\xfd'),
                                (8, zero.ZERO_PAYLOAD), (0, zero.ZERO_PAYLOAD),
                                (1, bytes(8)), (0x7ff, bytes.fromhex('0100550800000000'))):
            with self.assertRaises(RuntimeError):
                control.send_std(can_id, payload)
        self.assertFalse(bus.sent)
        readonly = scanner.StatusOnlyControl(bus=bus)
        with self.assertRaises(RuntimeError):
            readonly.send_std(1, zero.ZERO_PAYLOAD)
        control.send_std(1, zero.ZERO_PAYLOAD)
        with self.assertRaises(RuntimeError):
            control.send_std(1, zero.ZERO_PAYLOAD)
        self.assertEqual(len(bus.sent), 1)

    def test_identity_failure_and_busy_operation_never_open_hardware(self):
        roles = scanner.load_roles()
        with patch.object(scanner, 'verify_usb_identity', side_effect=RuntimeError('identity mismatch')), \
                patch.object(zero.ZeroCalibrationControl, 'open', side_effect=AssertionError('hardware opened')) as opened:
            report = zero.calibrate_zero('motion_left', roles)
        opened.assert_not_called()
        self.assertIn('identity mismatch', report['arms'][0]['error'])
        self.assertEqual(report['arms'][0]['tx'], [])
        with operation_lock.exclusive_operation(), \
                patch.object(zero, 'open_arm_control', side_effect=AssertionError('hardware opened')) as opened:
            report = zero.calibrate_zero('motion_left', roles)
            self.assertIn('不能同时运行', report['arms'][0]['error'])
            with self.assertRaisesRegex(RuntimeError, '不能同时运行'):
                scanner.scan_selected(['motion_right'], roles)
        opened.assert_not_called()

    def test_operation_lock_blocks_another_process_and_releases(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'shared.lock'
            code = ('from pathlib import Path\nimport operation_lock as o\n'
                    f'o.LOCK_PATH = Path({str(path)!r})\n'
                    'try:\n with o.exclusive_operation(): pass\n'
                    'except RuntimeError:\n raise SystemExit(42)\n')
            def child():
                return subprocess.run([sys.executable, '-c', code], cwd=Path(__file__).parent, capture_output=True).returncode
            with patch.object(operation_lock, 'LOCK_PATH', path):
                with operation_lock.exclusive_operation():
                    self.assertEqual(child(), 42)
                self.assertEqual(child(), 0)

    def test_real_slcan_path_writes_only_selected_pty_and_expected_frames(self):
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        os.close(slave)
        stop = threading.Event()
        adapter_closed = threading.Event()
        commands, bad, written = [], [], set()
        def emulate():
            buffer = b''
            while not stop.is_set():
                if not select.select([master], [], [], .01)[0]:
                    continue
                try:
                    buffer += os.read(master, 4096)
                except OSError as exc:
                    if exc.errno == errno.EIO:
                        stop.wait(.01)
                        continue
                    raise
                while b'\r' in buffer:
                    command, buffer = buffer.split(b'\r', 1)
                    if not command:
                        continue
                    commands.append(command)
                    if not command.startswith(b't'):
                        if command == b'C' and written:
                            adapter_closed.set()
                        continue
                    try:
                        can_id, data = int(command[1:4], 16), bytes.fromhex(command[5:].decode())
                        assert len(command) == 21 and command[4:5] == b'8'
                        if can_id == 0x7ff:
                            i = data[0]
                            assert data == scanner.status_payload(i)
                        else:
                            i = can_id
                            assert i in scanner.IDS and data == zero.ZERO_PAYLOAD and i not in written
                            written.add(i)
                        payload = bytes(feedback(i, 32768 if i in written else 40000).data)
                        os.write(master, f't{i+16:03X}8{payload.hex()}\r'.encode())
                    except Exception as exc:
                        bad.append((command, str(exc)))
        thread = threading.Thread(target=emulate, daemon=True)
        thread.start()
        try:
            roles = scanner.load_roles()
            roles['motion_left']['port'] = port
            with patch.object(zero, 'STATUS_TIMEOUT', .02), patch.object(zero, 'SETTLE_SECONDS', .01):
                report = zero.calibrate_zero('motion_left', roles)
            self.assertTrue(report['ok'], report['arms'][0]['error'])
            self.assertEqual(report['selected_roles'], ['motion_left'])
            self.assertEqual(written, set(scanner.IDS))
            self.assertFalse(bad, bad)
            self.assertEqual(len(report['arms'][0]['tx']), 42)
            self.assertTrue(adapter_closed.wait(1), 'adapter close was not received')
            self.assertEqual(commands[-1], b'C')
        finally:
            stop.set()
            thread.join(timeout=1)
            os.close(master)


if __name__ == '__main__':
    unittest.main()
