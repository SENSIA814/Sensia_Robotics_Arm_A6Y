#!/usr/bin/env python3
"""Exercise ID migration against simulated firmware and a real SLCAN PTY."""
import contextlib
import fcntl
import io
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'Motor Drive'))
import damiao_set_id as tool
from dm_canable2 import MotorControl


def frame(can_id, data, **kwargs):
    return SimpleNamespace(arbitration_id=can_id, data=bytes(data), is_extended_id=False,
                           is_remote_frame=False, is_error_frame=False, **kwargs)


class FirmwareBus:
    """Independent register/flash model: IDs take effect on the next frame."""

    def __init__(self, slave=1, master=0x11, mode=1):
        self.regs = {7: master, 8: slave, 10: mode}
        self.flash = self.regs.copy()
        self.state = 1
        self.queue = []
        self.sent = []
        self.drop_reads = set()
        self.read_drops_remaining = {}
        self.read_counts = {}
        self.drop_status = False
        self.status_drops_remaining = 0
        self.ignore_writes = set()
        self.extra_motors = {}
        self.disable_works = True
        self.save_ack = True
        self.save_works = True
        self.save_ack_code = 1
        self.ack_old_address = False
        self.bad_frames = False
        self.conflicting_reply = False
        self.shutdown_called = False
        self.interrupt_rid = None

    def send(self, msg, timeout=None):
        can_id, data = msg.arbitration_id, bytes(msg.data)
        self.sent.append((can_id, data))
        slave, master = self.regs[8], self.regs[7]
        if can_id != 0x7FF:
            if data != b'\xff' * 7 + b'\xfd':
                raise AssertionError(f'Unexpected control command: {data.hex()}')
            if can_id == slave + (self.regs[10] - 1) * 0x100 and self.disable_works:
                self.state = 0
            return
        address, op, rid, value = struct.unpack('<HBBI', data)
        if address != slave:
            if op == 0x33 and address in self.extra_motors:
                self.queue.append(frame(self.extra_motors[address], struct.pack('<HBBI', address, op, rid, address)))
            return
        if op == 0x33:
            key = (address, rid)
            self.read_counts[key] = self.read_counts.get(key, 0) + 1
            if self.read_drops_remaining.get(key, 0):
                self.read_drops_remaining[key] -= 1
                return
            if (address, rid) in self.drop_reads:
                return
            payload = struct.pack('<HBBI', address, op, rid, self.regs[rid])
            response = frame(master, payload)
            if self.bad_frames:
                response.is_extended_id = True
                self.queue.extend([frame(master, payload[:4]), frame(master, payload, dlc=7)])
            self.queue.append(response)
            if self.conflicting_reply:
                self.queue.append(frame(master + 1, payload))
        elif op == 0xCC:
            if self.drop_status:
                return
            if self.status_drops_remaining:
                self.status_drops_remaining -= 1
                return
            self.queue.append(frame(master, [slave | self.state << 4, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 25, 25]))
        elif op == 0x55:
            if self.state != 0 or rid not in (7, 8):
                raise AssertionError('Write while enabled, or non-ID register write')
            if rid not in self.ignore_writes:
                self.regs[rid] = value
            ack_slave = slave if self.ack_old_address else self.regs[8]
            ack_master = master if self.ack_old_address else self.regs[7]
            self.queue.append(frame(ack_master, struct.pack('<HBBI', ack_slave, op, rid, value)))
            if rid == self.interrupt_rid:
                raise KeyboardInterrupt()
        elif op == 0xAA:
            if self.state != 0 or rid != 1 or value != 0:
                raise AssertionError('Incorrect save opcode or motor still enabled')
            if self.save_works:
                self.flash = self.regs.copy()
            if self.save_ack:
                self.queue.append(frame(master, struct.pack('<HBB', slave, op, self.save_ack_code)))
        else:
            raise AssertionError(f'Unexpected opcode: {op}')

    def recv(self, timeout=0.0):
        if self.queue:
            return self.queue.pop(0)
        if timeout:
            time.sleep(timeout)
        return None

    def shutdown(self):
        self.shutdown_called = True

    def power_cycle(self):
        self.regs = self.flash.copy()
        self.state = 0
        self.queue.clear()

    def writes(self):
        return [struct.unpack('<HBBI', data) for cid, data in self.sent
                if cid == 0x7FF and data[2] in (0x55, 0xAA)]


class ProgramTest(unittest.TestCase):
    def setUp(self):
        self.bus = FirmwareBus()
        self.output = []
        self.programmer = tool.IdProgrammer(MotorControl(bus=self.bus), timeout=0.002, report=self.output.append)

    def program(self):
        self.programmer.program(1, 0x11, 4, 0x14)

    def test_changes_both_ids_and_persists_after_power_cycle(self):
        self.program()
        self.assertEqual(self.bus.writes(), [(1, 0x55, 7, 0x14), (1, 0x55, 8, 4), (4, 0xAA, 1, 0)])
        self.bus.power_cycle()
        self.assertEqual(self.programmer.read_pair(4, 0x14), 0x14)
        self.assertEqual(self.bus.regs[10], 1)
        self.assertEqual(self.bus.state, 0)
        self.assertTrue(self.output[-2].startswith('SAVED'))

    def test_supports_old_address_write_ack(self):
        self.bus.ack_old_address = True
        self.program()
        self.assertEqual(self.bus.flash[8], 4)

    def test_disable_uses_position_mode_command_id(self):
        self.bus.regs[10] = 2
        self.program()
        self.assertEqual([cid for cid, _ in self.bus.sent if cid != 0x7FF], [0x101])
        self.assertEqual(self.bus.flash[10], 2)

    def test_zero_feedback_id_can_be_discovered_and_replaced(self):
        self.bus.regs[7] = 0
        self.programmer.program(1, None, 4, 0x14)
        self.assertEqual(self.bus.flash[7], 0x14)

    def test_disabled_motor_without_ctrl_mode_reply_can_change_feedback_id(self):
        self.bus.regs[7] = 0
        self.bus.state = 0
        self.bus.drop_reads.add((1, 10))
        self.programmer.program(1, 0, 1, 0x11)
        self.assertEqual(self.bus.writes(), [(1, 0x55, 7, 0x11), (1, 0xAA, 1, 0)])
        self.bus.power_cycle()
        self.assertEqual(self.programmer.read_pair(1, 0x11), 0x11)

    def test_transient_control_mode_timeout_is_retried_before_disable(self):
        self.bus.regs[7] = 0
        self.bus.read_drops_remaining[(1, 10)] = 1
        self.programmer.program(1, 0, 1, 0x11)
        self.assertEqual(self.bus.read_counts[(1, 10)], 2)
        self.assertEqual(self.bus.flash[7], 0x11)
        self.assertTrue(any('RETRY' in line and 'RID=10' in line for line in self.output))

    def test_enabled_motor_without_control_mode_never_disables_or_writes(self):
        self.bus.drop_reads.add((1, 10))
        with self.assertRaisesRegex(RuntimeError, 'RID=10'):
            self.program()
        self.assertEqual(self.bus.read_counts[(1, 10)], 3)
        self.assertFalse(self.bus.writes())
        self.assertTrue(all(cid == 0x7FF for cid, _ in self.bus.sent))

    def test_missing_status_never_disables_or_writes(self):
        self.bus.drop_status = True
        with self.assertRaisesRegex(RuntimeError, 'No fresh status reply'):
            self.program()
        self.assertFalse(self.bus.writes())
        self.assertTrue(all(cid == 0x7FF for cid, _ in self.bus.sent))

    def test_transient_status_timeout_is_retried(self):
        self.bus.status_drops_remaining = 1
        self.program()
        self.assertEqual(self.bus.flash[8], 4)

    def test_trace_reports_actual_tx_and_rx_including_zero_master(self):
        self.bus.regs[7] = 0
        programmer = tool.IdProgrammer(MotorControl(bus=self.bus), timeout=0.002,
                                       report=self.output.append, trace=True)
        programmer.read_pair(1, 0)
        self.assertTrue(any('TX CAN=0x7FF DATA=0100330800000000' in line for line in self.output))
        self.assertTrue(any('RX CAN=0x000 DATA=0100330801000000' in line for line in self.output))
        self.assertFalse(self.bus.writes())

    def test_diagnosis_reports_status_even_when_control_mode_unavailable(self):
        self.bus.drop_reads.add((1, 10))
        self.bus.state = 0
        self.assertFalse(self.programmer.diagnose(1, 0x11))
        self.assertTrue(any('RID=10' in line for line in self.output))
        self.assertIn('STATE=0 (disabled)', self.output)
        self.assertTrue(all(cid == 0x7FF and data[2] in (0x33, 0xCC) for cid, data in self.bus.sent))

    def test_feedback_only_change(self):
        self.programmer.program(1, 0x11, 1, 0x21)
        self.assertEqual(self.bus.writes(), [(1, 0x55, 7, 0x21), (1, 0xAA, 1, 0)])

    def test_same_pair_saves_once_without_register_rewrite(self):
        self.programmer.program(1, 0x11, 1, 0x11)
        self.assertEqual(self.bus.writes(), [(1, 0xAA, 1, 0)])

    def test_occupied_target_refused_before_disable(self):
        self.bus.extra_motors[4] = 0x14
        with self.assertRaisesRegex(RuntimeError, 'Another motor'):
            self.program()
        self.assertFalse(self.bus.writes())
        self.assertTrue(all(cid == 0x7FF for cid, _ in self.bus.sent))

    def test_any_other_motor_refused_even_if_target_is_free(self):
        self.bus.extra_motors[7] = 0x17
        with self.assertRaisesRegex(RuntimeError, 'Another motor'):
            self.program()
        self.assertFalse(self.bus.writes())

    def test_wrong_current_master_refused(self):
        self.bus.regs[7] = 0x21
        with self.assertRaisesRegex(RuntimeError, 'Unexpected feedback ID'):
            self.program()
        self.assertFalse(self.bus.writes())

    def test_conflicting_identity_refused(self):
        self.bus.conflicting_reply = True
        with self.assertRaisesRegex(RuntimeError, 'Conflicting replies'):
            self.program()
        self.assertFalse(self.bus.writes())

    def test_no_source_or_stale_reply_cannot_authorize_write(self):
        self.bus.drop_reads.add((1, 8))
        self.bus.queue.append(frame(0x11, struct.pack('<HBBI', 1, 0x33, 8, 1)))
        with self.assertRaisesRegex(RuntimeError, 'No fresh parameter reply'):
            self.program()
        self.assertFalse(self.bus.writes())

    def test_malformed_or_extended_replies_cannot_authorize_write(self):
        self.bus.bad_frames = True
        with self.assertRaisesRegex(RuntimeError, 'No fresh parameter reply'):
            self.program()
        self.assertFalse(self.bus.writes())

    def test_disabled_feedback_required(self):
        self.bus.disable_works = False
        with self.assertRaisesRegex(RuntimeError, 'Disabled state not confirmed'):
            self.program()
        self.assertFalse(self.bus.writes())

    def test_fault_feedback_blocks_write(self):
        self.bus.disable_works = False
        self.bus.state = 8
        with self.assertRaisesRegex(RuntimeError, 'Disabled state not confirmed'):
            self.program()
        self.assertFalse(self.bus.writes())

    def test_unknown_control_mode_blocks_disable_and_write(self):
        self.bus.regs[10] = 99
        with self.assertRaisesRegex(RuntimeError, 'Unsupported CTRL_MODE'):
            self.program()
        self.assertFalse(self.bus.writes())
        self.assertTrue(all(cid == 0x7FF for cid, _ in self.bus.sent))

    def test_write_echo_is_not_readback_proof(self):
        self.bus.ignore_writes.add(7)
        with self.assertRaisesRegex(RuntimeError, 'Unexpected feedback ID'):
            self.program()
        self.assertEqual(self.bus.writes(), [(1, 0x55, 7, 0x14)])
        self.assertEqual(self.bus.flash[8], 1)

    def test_partial_id_change_does_not_save_or_roll_back(self):
        self.bus.ignore_writes.add(8)
        with self.assertRaisesRegex(RuntimeError, 'No fresh parameter reply'):
            self.program()
        self.assertEqual(self.bus.writes(), [(1, 0x55, 7, 0x14), (1, 0x55, 8, 4)])
        self.assertTrue(self.programmer.write_attempted)
        self.assertFalse(self.programmer.save_attempted)
        self.assertEqual(self.bus.regs[7], 0x14)

    def test_missing_save_ack_never_reports_saved(self):
        self.bus.save_ack = False
        with self.assertRaisesRegex(RuntimeError, 'SAVE_UNCONFIRMED'):
            self.program()
        self.assertFalse(any(line.startswith('SAVED') for line in self.output))
        self.assertEqual(sum(op == 0xAA for _, op, _, _ in self.bus.writes()), 1)

    def test_wrong_save_ack_code_is_not_success(self):
        self.bus.save_ack_code = 0
        with self.assertRaisesRegex(RuntimeError, 'SAVE_UNCONFIRMED'):
            self.program()

    def test_post_save_readback_failure_is_not_success(self):
        original_send = self.bus.send
        def fail_after_save(msg, timeout=None):
            original_send(msg, timeout)
            if msg.arbitration_id == 0x7FF and msg.data[2] == 0xAA:
                self.bus.drop_reads.add((4, 8))
        self.bus.send = fail_after_save
        with self.assertRaisesRegex(RuntimeError, 'No fresh parameter reply'):
            self.program()
        self.assertFalse(any(line.startswith('SAVED') for line in self.output))

    def test_verify_detects_nonpersistent_ram_ids_after_power_cycle(self):
        self.bus.save_works = False
        self.program()  # An ACK alone cannot establish nonvolatile persistence.
        self.bus.power_cycle()
        with self.assertRaisesRegex(RuntimeError, 'No fresh parameter reply'):
            self.programmer.read_pair(4, 0x14)

    def test_read_only_verify_sends_no_disable_write_or_save(self):
        self.programmer.read_pair(1, 0x11)
        self.assertTrue(all(cid == 0x7FF and data[2] == 0x33 for cid, data in self.bus.sent))

    def test_interrupt_during_write_marks_partial_state(self):
        self.bus.interrupt_rid = 7
        with self.assertRaises(KeyboardInterrupt):
            self.program()
        self.assertTrue(self.programmer.write_attempted)
        self.assertFalse(self.programmer.save_attempted)


class CliTest(unittest.TestCase):
    def test_default_preview_never_opens_port(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = tool.main(['--channel', '/does/not/exist', '--current-id', '1', '--new-id', '0x04'])
        self.assertEqual(result, 0)
        self.assertIn('MST_ID -> 0x14', output.getvalue())
        self.assertIn('DRY RUN', output.getvalue())

    def test_invalid_arguments_are_rejected_before_port_access(self):
        cases = [('--current-id', '0'), ('--new-id', '16'), ('--new-master-id', '0'),
                 ('--new-master-id', '4'), ('--new-master-id', '0x7FF'),
                 ('--current-master-id', '-1'), ('--timeout', 'nan'), ('--timeout', '0'),
                 ('--can-bitrate', '999'), ('--tty-baudrate', '-1'), ('--verify',),
                 ('--read-attempts', '0'), ('--read-attempts', '11'), ('--diagnose',)]
        for extra in cases:
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as caught:
                    tool.main(['--current-id', '1', '--new-id', '4', '--write', *extra])
                self.assertEqual(caught.exception.code, 2)

    def test_lock_refuses_before_slcan_initialization(self):
        with tempfile.NamedTemporaryFile() as device:
            fcntl.flock(device.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = subprocess.run([sys.executable, str(ROOT / 'tools/damiao_set_id.py'),
                '--channel', device.name, '--current-id', '1', '--new-id', '4', '--write'],
                capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn('open/lock serial device', result.stderr)
            self.assertEqual(Path(device.name).read_bytes(), b'')

    def test_real_slcan_cli_write_and_power_cycle_verify(self):
        firmware = FirmwareBus(master=0, mode=2)
        firmware.read_drops_remaining[(1, 10)] = 1
        master_fd, slave_fd = pty.openpty()
        stop = threading.Event()
        failures = []
        def serve():
            buffer = b''
            try:
                while not stop.is_set():
                    if not select.select([master_fd], [], [], 0.02)[0]:
                        continue
                    buffer += os.read(master_fd, 4096)
                    while b'\r' in buffer:
                        command, buffer = buffer.split(b'\r', 1)
                        if not command.startswith(b't'):
                            continue
                        cid = int(command[1:4], 16)
                        firmware.send(frame(cid, bytes.fromhex(command[5:].decode())))
                        while firmware.queue:
                            msg = firmware.queue.pop(0)
                            os.write(master_fd, f't{msg.arbitration_id:03X}{len(msg.data):X}{msg.data.hex()}\r'.encode())
            except Exception as exc:
                failures.append(exc)
        thread = threading.Thread(target=serve, daemon=True)
        thread.start()
        try:
            common = [sys.executable, str(ROOT / 'tools/damiao_set_id.py'),
                      '--channel', os.ttyname(slave_fd), '--timeout', '0.05']
            result = subprocess.run([*common, '--current-id', '1', '--new-id', '4', '--write'],
                                    capture_output=True, text=True, timeout=12)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('SAVED ESC_ID=0x04 MST_ID=0x14', result.stdout)
            self.assertEqual(firmware.read_counts[(1, 10)], 2)
            firmware.power_cycle()
            firmware.sent.clear()
            result = subprocess.run([*common, '--current-id', '4', '--current-master-id', '0x14', '--verify'],
                                    capture_output=True, text=True, timeout=8)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('VERIFIED', result.stdout)
            self.assertTrue(all(cid == 0x7FF and data[2] == 0x33 for cid, data in firmware.sent))
            firmware.drop_reads.add((4, 10))
            firmware.sent.clear()
            result = subprocess.run([*common, '--current-id', '4', '--current-master-id', '0x14',
                                     '--diagnose', '--trace', '--read-attempts', '2'],
                                    capture_output=True, text=True, timeout=8)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            self.assertIn('RID=10 after 2 attempts', result.stdout)
            self.assertIn('STATE=0 (disabled)', result.stdout)
            self.assertIn('TX CAN=0x7FF DATA=0400330A00000000', result.stdout)
            self.assertTrue(all(cid == 0x7FF and data[2] in (0x33, 0xCC) for cid, data in firmware.sent))
            self.assertFalse(failures)
        finally:
            stop.set()
            thread.join(timeout=1)
            os.close(master_fd)
            os.close(slave_fd)


if __name__ == '__main__':
    unittest.main()
