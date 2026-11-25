#!/usr/bin/env python3
"""Exercise the ported parameter CLIs against independent simulated registers."""
import contextlib
import fcntl
import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'Motor Drive'))
import read_motor_params as reader
import write_motor_params_temp as writer
import save_motor_params_flash as saver
from dm_canable2 import DM_variable, MotorControl
from test_damiao_set_id import FirmwareBus, frame


def float_bits(value):
    return struct.unpack('<I', struct.pack('<f', value))[0]


class ParameterBus(FirmwareBus):
    def __init__(self):
        super().__init__(mode=2)
        self.state = 0
        self.regs.update({4: float_bits(2), 5: float_bits(-2),
                          25: float_bits(0.00384), 26: float_bits(0.002),
                          27: float_bits(54), 28: float_bits(0)})
        self.flash = self.regs.copy()

    def send(self, msg, timeout=None):
        payload = bytes(msg.data)
        if msg.arbitration_id == 0x7FF and payload[2] == 0x55:
            self.sent.append((msg.arbitration_id, payload))
            slave, op, rid, value = struct.unpack('<HBBI', payload)
            if rid not in self.ignore_writes:
                self.regs[rid] = value
            # Even firmware echoing a rejected value must fail the later read.
            self.queue.append(frame(self.regs[7], payload))
            return
        super().send(msg, timeout)


class MotorParameterTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.channel = Path(self.temp.name) / 'serial'
        self.channel.touch()
        self.config = Path(self.temp.name) / 'motors.yaml'
        self.config.write_text('joints:\n  joint1:\n    motor_type: DM4340P\n    slave_id: 1\n    master_id: 17\n')
        self.bus = ParameterBus()

    def invoke(self, module, arguments, *, connect=True):
        def opened(control):
            if not connect:
                self.fail('preview must not open a serial bus')
            # Test the real lock helper, including ordering before SLCAN open.
            with self.channel.open('rb') as device:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(device.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            control.bus = self.bus

        command = [module.__file__, '--channel', str(self.channel), '--config', str(self.config), *arguments]
        output = io.StringIO()
        with patch.object(sys, 'argv', command), patch.object(MotorControl, 'open', opened), \
                contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            result = module.run_cli(module.main)
        return result, output.getvalue()

    def test_a5_default_and_legacy_config(self):
        default = reader.load_config_targets(reader.default_config_path(), None)
        self.assertEqual(len(default), 7)
        self.assertEqual((default[0].name, default[0].slave_id, default[0].master_id), ('joint1', 1, 17))
        legacy = Path(self.temp.name) / 'legacy.yaml'
        legacy.write_text('a6_y_damiao_system:\n  ros__parameters:\n    joints:\n      joint1:\n        motor_type: DM4340P\n        slave_id: 0x01\n        master_id: 0x11\n')
        self.assertEqual(reader.load_config_targets(legacy, ['joint1'])[0], default[0])

    def test_read_only_frames_and_float_decode(self):
        code, output = self.invoke(reader, ['--joints', 'joint1'])
        self.assertEqual(code, 0, output)
        self.assertIn('0.00384,0.002,54,0', output)
        self.assertTrue(all(can_id == 0x7FF and data[2] == 0x33 for can_id, data in self.bus.sent))
        self.assertTrue(self.bus.shutdown_called)

    def test_preview_and_missing_yes_never_open_bus(self):
        cases = [(reader, ['--joints', 'joint1', '--dry-run'], 0),
                 (writer, ['--joints', 'joint1', '--set', 'KP_APR=60', '--dry-run', '--yes'], 0),
                 (writer, ['--joints', 'joint1', '--set', 'KP_APR=60'], 2),
                 (saver, ['--joints', 'joint1', '--dry-run', '--yes'], 0),
                 (saver, ['--joints', 'joint1'], 2)]
        for module, args, expected in cases:
            with self.subTest(module=module.__name__, args=args):
                code, output = self.invoke(module, args, connect=False)
                self.assertEqual(code, expected, output)
        self.assertEqual(self.bus.sent, [])

    def test_temp_write_readback_without_flash_or_enable(self):
        before_flash = self.bus.flash.copy()
        code, output = self.invoke(writer, ['--joints', 'joint1', '--set', 'KP_APR=60', '--yes'])
        self.assertEqual(code, 0, output)
        self.assertEqual(self.bus.regs[27], float_bits(60))
        self.assertEqual(self.bus.flash, before_flash)
        self.assertEqual([data[2] for _, data in self.bus.sent], [0x33, 0x55, 0x33])
        self.assertIn('54,60,OK', output)

    def test_small_gain_rejected_even_if_write_ack_echoes_requested_value(self):
        self.bus.ignore_writes.add(26)
        code, output = self.invoke(writer, ['--joints', 'joint1', '--set', 'KI_ASR=0.003',
                                            '--set', 'KP_APR=60', '--yes'])
        self.assertEqual(code, 1, output)
        self.assertEqual(self.bus.regs[27], float_bits(54))
        self.assertEqual(sum(data[2] == 0x55 for _, data in self.bus.sent), 1)

    def test_no_write_when_before_read_fails(self):
        with patch.object(MotorControl, 'read_motor_param', return_value=None):
            code, output = self.invoke(writer, ['--joints', 'joint1', '--set', 'KP_APR=60', '--yes'])
        self.assertEqual(code, 1, output)
        self.assertEqual(self.bus.sent, [])

    def test_invalid_parameter_values_rejected_offline(self):
        for assignment in ['KP_APR=nan', 'KP_APR=inf', 'KP_APR=1e100', 'CTRL_MODE=-1', 'CTRL_MODE=4294967296', '256=1']:
            with self.subTest(assignment=assignment):
                code, output = self.invoke(writer, ['--joints', 'joint1', '--set', assignment, '--yes'], connect=False)
                self.assertEqual(code, 2, output)

    def test_device_lock_prevents_any_bus_open(self):
        with self.channel.open('rb') as device:
            fcntl.flock(device.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            code, output = self.invoke(reader, ['--joints', 'joint1'], connect=False)
        self.assertEqual(code, 1, output)
        self.assertIn('busy', output)

    def test_save_disables_pos_vel_at_correct_id_then_saves_all_ram(self):
        self.bus.state = 1
        self.bus.regs[27] = float_bits(60)
        code, output = self.invoke(saver, ['--joints', 'joint1', '--params', 'KP_APR', '--yes'])
        self.assertEqual(code, 0, output)
        self.assertEqual(self.bus.state, 0)
        self.assertEqual(self.bus.flash, self.bus.regs)
        self.assertIn((0x101, b'\xff' * 7 + b'\xfd'), self.bus.sent)
        self.assertIn((0x7FF, bytes.fromhex('01 00 AA 01 00 00 00 00')), self.bus.sent)
        self.assertFalse(any(data == b'\xff' * 7 + b'\xfc' for _, data in self.bus.sent))

    def test_save_missing_ack_is_unconfirmed_without_retry(self):
        self.bus.save_ack = False
        code, output = self.invoke(saver, ['--joints', 'joint1', '--params', 'KP_APR', '--yes'])
        self.assertEqual(code, 1, output)
        self.assertIn('SAVE_UNCONFIRMED', output)
        self.assertEqual(sum(data[2] == 0xAA for can_id, data in self.bus.sent if can_id == 0x7FF), 1)


if __name__ == '__main__':
    unittest.main()
