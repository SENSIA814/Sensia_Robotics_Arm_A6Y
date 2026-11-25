#!/usr/bin/env python3
"""Exercise the real SLCAN/feedback path via PTY; never opens a hardware device."""
import fcntl
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import threading
import unittest

TOOL = Path(__file__).resolve().parents[1] / 'tools/damiao_feedback_watch.py'


class WatchTest(unittest.TestCase):
    def run_capture(self, moving=None, missing=None, locked=False, interrupt=False, swapped=False):
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp) / 'motors.yaml'
            mapping = {i: (6 if i == 4 else 4 if i == 6 else i) if swapped else i for i in range(1, 7)}
            config.write_text('joints:\n' + ''.join(
                f'  joint{i}: {{motor_type: DM4310, slave_id: {motor_id}, master_id: {motor_id+16}}}\n'
                for i, motor_id in mapping.items()))
            log = Path(temp) / 'capture.jsonl'
            master, slave = pty.openpty()
            sent = []
            errors = []
            counts = {i: 0 for i in range(1, 7)}
            stop = threading.Event()
            ready = threading.Event()
            if locked:
                fcntl.flock(slave, fcntl.LOCK_EX | fcntl.LOCK_NB)
            def server():
                buffer = b''
                while not stop.is_set():
                    if not select.select([master], [], [], 0.05)[0]:
                        continue
                    buffer += os.read(master, 4096)
                    while b'\r' in buffer:
                        command, buffer = buffer.split(b'\r', 1)
                        if not command:
                            continue
                        sent.append(command)
                        if command.startswith(b't'):
                            if not (len(command) == 21 and command[:5] == b't7FF8'):
                                errors.append(command)
                                continue
                            payload = bytes.fromhex(command[5:].decode())
                            motor_id = payload[0]
                            if payload != bytes([motor_id, 0, 0xCC, 0, 0, 0, 0, 0]) or motor_id not in counts:
                                errors.append(command)
                                continue
                            counts[motor_id] += 1
                            ready.set()
                            if motor_id == missing:
                                continue
                            # Mid-capture motion followed by a plateau; quantized raw feedback.
                            position = (32767 + min(counts[motor_id], 20) * 100
                                        if motor_id == moving else 32767)
                            data = bytes([motor_id, position >> 8, position & 255, 0x7F, 0xF7, 0xFF, 25, 26])
                            os.write(master, f't{motor_id+16:03X}8{data.hex().upper()}\r'.encode())
            thread = threading.Thread(target=server, daemon=True)
            thread.start()
            process = None
            try:
                process = subprocess.Popen([sys.executable, str(TOOL), '--channel', os.ttyname(slave),
                    '--config', str(config), '--duration', '1.0', '--interval', '0.15',
                    *(['--expect-joint', 'joint4'] if swapped else ['--expect-id', '4']), '--log', str(log)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                if interrupt:
                    self.assertTrue(ready.wait(6), 'scanner did not start')
                    process.send_signal(signal.SIGINT)
                stdout, stderr = process.communicate(timeout=10)
                records = [json.loads(line) for line in log.read_text().splitlines()]
                self.assertFalse(errors, errors)
                if locked:
                    self.assertFalse(sent, sent)
                else:
                    # One persistent connection throughout; all CAN transmissions are status requests.
                    self.assertEqual(sent.count(b'O'), 1, sent)
                    self.assertGreater(sum(counts.values()), 0)
                return process.returncode, stdout, stderr, records
            finally:
                if process is not None and process.poll() is None:
                    process.kill()
                    process.communicate()
                stop.set()
                thread.join(timeout=1)
                os.close(master)
                os.close(slave)

    def test_static_feedback_is_red_for_expected_motion(self):
        code, stdout, _, records = self.run_capture()
        self.assertEqual(code, 1)
        summary = records[-1]
        self.assertTrue(summary['communication_ok'])
        self.assertFalse(summary['motion_change_seen'])
        self.assertEqual(summary['motors']['4']['q_span'], 0)
        self.assertIn('NO_SUFFICIENT_CHANGE', stdout)

    def test_j4_motion_is_green_and_raw_bytes_are_recorded(self):
        code, _, _, records = self.run_capture(moving=4)
        self.assertEqual(code, 0)
        self.assertTrue(records[-1]['motion_change_seen'])
        frames = [r for r in records if r['event'] == 'rx' and r.get('id') == 4]
        self.assertGreater(len({r['q_u16'] for r in frames}), 1)
        for row in frames:
            self.assertAlmostEqual(row['q'], row['q_u16'] * 25 / 65535 - 12.5)

    def test_expected_joint_follows_swapped_configuration(self):
        code, stdout, _, records = self.run_capture(moving=6, swapped=True)
        self.assertEqual(code, 0)
        self.assertEqual(records[0]['expected_id'], 6)
        self.assertEqual(records[0]['expected_joint'], 'joint4')
        self.assertTrue(records[-1]['motion_change_seen'])
        self.assertEqual(records[-1]['motors']['4']['q_span'], 0)
        self.assertIn('EXPECTED ID=0x06: CHANGE_SEEN', stdout)

    def test_different_id_motion_is_visible_but_not_j4_success(self):
        code, _, _, records = self.run_capture(moving=5)
        self.assertEqual(code, 1)
        self.assertGreater(records[-1]['motors']['5']['q_span'], 0.02)
        self.assertEqual(records[-1]['motors']['4']['q_span'], 0)

    def test_missing_feedback_is_not_fabricated_zero(self):
        code, stdout, _, records = self.run_capture(missing=6)
        self.assertEqual(code, 1)
        self.assertIn('ID06=MISS', stdout)
        self.assertIsNone(records[-1]['motors']['6']['q_span'])
        self.assertFalse(records[-1]['communication_ok'])

    def test_lock_refuses_before_serial_commands(self):
        code, _, _, _ = self.run_capture(locked=True)
        self.assertEqual(code, 2)

    def test_interrupt_has_no_traceback(self):
        code, stdout, stderr, records = self.run_capture(interrupt=True)
        self.assertEqual(code, 130)
        self.assertNotIn('Traceback', stdout + stderr)
        self.assertTrue(records[-1]['interrupted'])


if __name__ == '__main__':
    unittest.main()
