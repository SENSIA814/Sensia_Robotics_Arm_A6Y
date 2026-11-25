#!/usr/bin/env python3
"""Hardware-free integration tests: real CANable2/SLCAN path through a PTY."""
import errno
import http.client
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
import time
import unittest
from unittest.mock import patch

import yaml

import scanner
import server


class ScanTests(unittest.TestCase):
    def test_no_arm_default_and_invalid_selection(self):
        result = subprocess.run([sys.executable, scanner.__file__, '--dry-run'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('--arms', result.stderr)
        roles = scanner.load_roles()
        for selected in ([], ['unknown'], ['all', 'motion_left'], ['motion_left'] * 2, None, [{}]):
            with self.assertRaises(ValueError):
                scanner.validate_selection(selected, roles)
        for passes, timeout in ((0, .06), (True, .06), (31, .06), (1, float('nan'))):
            with self.assertRaises(ValueError):
                scanner.validate_options(passes, timeout)

    def test_transmission_allowlist(self):
        class Bus:
            sent = []
            def send(self, msg, timeout):
                self.sent.append(msg)
        bus = Bus()
        control = scanner.StatusOnlyControl(bus=bus)
        for can_id, data in ((1, b'\xff'*7+b'\xfc'), (1, b'\xff'*7+b'\xfd'),
                             (0x7ff, bytes.fromhex('0100330800000000')),
                             (0x7ff, bytes.fromhex('0800cc0000000000')),
                             (0x7ff, scanner.status_payload(1) + b'\x00')):
            with self.assertRaises(RuntimeError):
                control.send_std(can_id, data)
        self.assertEqual(bus.sent, [])
        control.send_std(0x7ff, scanner.status_payload(7))
        self.assertEqual(len(bus.sent), 1)

    def capture(self, missing=None, intermittent=None, faulty=None, malformed=None, interrupt=False, busy=False, position_test=False):
        with tempfile.TemporaryDirectory() as temp:
            temp = Path(temp)
            master, slave = pty.openpty()
            device = os.ttyname(slave)
            roles = {'roles': {}}
            for role in scanner.ROLE_ORDER:
                serial = role.upper()
                if position_test and role == 'motion_left':
                    serial = '304C33733036'
                port = temp / f'usb-CANable2_{serial}-if00'
                if role == 'motion_left':
                    port.symlink_to(device)
                roles['roles'][role] = dict(display_name=role, usb_serial=serial, by_id=str(port))
            config = temp / 'roles.yaml'
            config.write_text(yaml.safe_dump(roles))
            if not busy:
                os.close(slave)
                slave = None
            commands, bad = [], []
            counts = {i: 0 for i in scanner.IDS}
            ready, stop = threading.Event(), threading.Event()
            def emulate():
                buffer = b''
                while not stop.is_set():
                    if not select.select([master], [], [], .02)[0]:
                        continue
                    try:
                        buffer += os.read(master, 4096)
                    except OSError as exc:
                        if exc.errno == errno.EIO:
                            stop.wait(.02)
                            continue
                        raise
                    while b'\r' in buffer:
                        command, buffer = buffer.split(b'\r', 1)
                        if not command:
                            continue
                        commands.append(command)
                        if not command.startswith(b't'):
                            continue
                        if len(command) != 21 or not command.startswith(b't7FF8'):
                            bad.append(command)
                            continue
                        data = bytes.fromhex(command[5:].decode())
                        i = data[0]
                        if i not in counts or data != scanner.status_payload(i):
                            bad.append(command)
                            continue
                        counts[i] += 1
                        ready.set()
                        if i == missing or (i == intermittent and counts[i] > 1):
                            continue
                        state = 8 if i == faulty and counts[i] == 1 else 0
                        payload = bytes([state << 4 | i, 0x80, 0, 0x7f, 0xf7, 0xff, 25, 26])
                        if position_test:
                            # Change the sampled position after the first pass.
                            payload = payload[:1] + (b'\x00\x00' if counts[i] == 1 else b'\xff\xff') + payload[3:]
                        if i == malformed:
                            payload = bytes([6]) + payload[1:]
                        os.write(master, f't{i+16:03X}8{payload.hex().upper()}\r'.encode())
            emulator = threading.Thread(target=emulate, daemon=True)
            emulator.start()
            proc = None
            try:
                proc = subprocess.Popen([sys.executable, scanner.__file__, '--roles', str(config),
                                         '--arms', 'motion_left', '--passes', '3', '--timeout', '.02',
                                         '--report-dir', str(temp / 'reports')],
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                if interrupt:
                    self.assertTrue(ready.wait(7), 'serial scanner never sent a status request')
                    proc.send_signal(signal.SIGINT)
                stdout, stderr = proc.communicate(timeout=12)
                self.assertNotIn('Traceback', stderr, stdout + stderr)
                reports = list((temp / 'reports').glob('*.json'))
                self.assertEqual(len(reports), 1, stdout + stderr)
                report = json.loads(reports[0].read_text())
                if position_test:
                    self.assertIn('位置 179.91° / 3.1400 rad（原始计数 65535）', stdout)
                self.assertEqual(report['selected_roles'], ['motion_left'])
                self.assertEqual(len(report['arms']), 1)
                self.assertFalse(bad, bad)
                if busy:
                    self.assertEqual(commands, [])
                else:
                    # python-can versions differ in whether initialization sends
                    # O once or twice. Bitrate configuration must occur once;
                    # no adapter reconfiguration is allowed between requests.
                    self.assertEqual(commands.count(b'S8'), 1, commands)
                    can_indices = [i for i, cmd in enumerate(commands) if cmd.startswith(b't')]
                    self.assertTrue(can_indices, commands)
                    self.assertTrue(all(cmd.startswith(b't') for cmd in commands[can_indices[0]:can_indices[-1]+1]))
                    self.assertEqual(commands[-1], b'C', commands)
                    self.assertGreater(sum(counts.values()), 0)
                    self.assertTrue(all(bytes.fromhex(t['data']) == scanner.status_payload(bytes.fromhex(t['data'])[0]) for t in report['arms'][0]['tx']))
                return proc.returncode, report
            finally:
                if proc and proc.poll() is None:
                    proc.kill()
                    proc.communicate()
                stop.set()
                emulator.join(timeout=1)
                os.close(master)
                if slave is not None:
                    os.close(slave)

    def test_seven_motors_and_only_selected_arm(self):
        code, report = self.capture()
        self.assertEqual(code, 0)
        self.assertTrue(report['ok'])
        arm = report['arms'][0]
        self.assertEqual(arm['responding'], 7)
        self.assertEqual(len(arm['tx']), 21)
        self.assertTrue(all(m['hits'] == 3 and m['last']['mos_temperature_c'] == 25 for m in arm['motors']))
        self.assertTrue(all(m['last']['position_rad'] is None for m in arm['motors']))

    def test_latest_position_uses_each_motor_range_without_extra_commands(self):
        code, report = self.capture(position_test=True)
        self.assertEqual(code, 0)
        arm = report['arms'][0]
        self.assertEqual(len(arm['tx']), 21)
        for motor in arm['motors']:
            self.assertEqual(motor['last']['position_raw'], 65535)
            self.assertAlmostEqual(motor['last']['position_rad'], 3.140000104904175 if motor['id'] == 7 else 12.5)
            self.assertIn('2026-11-24', motor['position_scale']['measured_at'])

    def test_missing_motor_is_not_zero_feedback(self):
        code, report = self.capture(missing=7)
        self.assertEqual(code, 1)
        motor = report['arms'][0]['motors'][6]
        self.assertEqual(motor['result'], 'missing')
        self.assertIsNone(motor['last'])
        self.assertEqual(report['arms'][0]['missing_ids'], [7])

    def test_one_reply_does_not_pass_stability(self):
        code, report = self.capture(intermittent=3)
        self.assertEqual(code, 1)
        self.assertEqual(report['arms'][0]['motors'][2]['result'], 'intermittent')

    def test_earlier_fault_is_preserved(self):
        code, report = self.capture(faulty=4)
        self.assertEqual(code, 1)
        self.assertEqual(report['arms'][0]['motors'][3]['states'], [0, 8])
        self.assertEqual(report['arms'][0]['motors'][3]['result'], 'state_warning')

    def test_mismatched_id_rejected(self):
        code, report = self.capture(malformed=5)
        self.assertEqual(code, 1)
        self.assertEqual(report['arms'][0]['motors'][4]['hits'], 0)
        self.assertGreater(report['arms'][0]['unexpected_frames'], 0)

    def test_interrupt_saves_partial_and_closes(self):
        code, report = self.capture(interrupt=True)
        self.assertEqual(code, 130)
        self.assertTrue(report['cancelled'])
        self.assertFalse(report['ok'])
        self.assertLess(len(report['arms'][0]['tx']), 21)

    def test_busy_device_does_not_initialize_serial(self):
        code, report = self.capture(busy=True)
        self.assertEqual(code, 1)
        self.assertIn('占用', report['arms'][0]['error'])
        self.assertEqual(report['arms'][0]['tx'], [])

    def test_disconnected_selection_never_opens_or_falls_back(self):
        roles = scanner.load_roles()
        for arm in roles.values():
            arm['port'] = '/this/adapter/does/not/exist'
        with patch.object(scanner.StatusOnlyControl, 'open', side_effect=AssertionError('opened hardware')):
            result = scanner.scan_selected(['motion_left', 'teaching_right'], roles, passes=1)
        self.assertFalse(result['ok'])
        self.assertEqual(len(result['arms']), 2)
        self.assertTrue(all(a['error'] and len(a['motors']) == 7 and not a['tx'] for a in result['arms']))

    def test_http_loading_has_no_hardware_effect_and_posts_require_target(self):
        session = server.ScanSession(scanner.load_roles(), Path('/tmp/unused-scan-report'))
        httpd = server.make_server(session, 0)
        thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        thread.start()
        port = httpd.server_address[1]
        def request(method, path, body=None, headers=None):
            conn = http.client.HTTPConnection('127.0.0.1', port)
            conn.request(method, path, json.dumps(body) if body is not None else None, headers or {})
            response = conn.getresponse()
            data = response.read()
            conn.close()
            return response.status, data
        headers = {'Origin': f'http://127.0.0.1:{port}', 'X-Scan-Token': session.token, 'Content-Type': 'application/json'}
        try:
            with patch.object(scanner.StatusOnlyControl, 'open', side_effect=AssertionError('unexpected serial open')):
                self.assertEqual(request('GET', '/')[0], 200)
                self.assertEqual(request('GET', '/api/init')[0], 200)
                self.assertIsNone(session.worker)
                self.assertEqual(request('POST', '/api/scan', {'arms': [], 'passes': 3}, headers)[0], 400)
                self.assertEqual(request('POST', '/api/scan', {'arms': ['typo'], 'passes': 3}, headers)[0], 400)
                self.assertEqual(request('POST', '/api/scan', {'arms': ['motion_left'], 'passes': True}, headers)[0], 400)
                self.assertEqual(request('POST', '/api/scan', {'arms': ['motion_left'], 'passes': 3})[0], 403)
                self.assertEqual(request('POST', '/api/scan', {}, {**headers, 'Origin': 'http://evil.invalid'})[0], 403)
                self.assertEqual(request('GET', '/', headers={'Host': 'evil.invalid'})[0], 403)
                self.assertIsNone(session.worker)
        finally:
            httpd.shutdown()
            httpd.server_close()
            thread.join(timeout=1)


if __name__ == '__main__':
    unittest.main()
