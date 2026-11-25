"""Independent register-bus checks; no serial device is opened."""
import copy
import io
import json
import struct
import time
from contextlib import redirect_stdout
from types import SimpleNamespace
import unittest
import apply_gains as trial


class Bus:
    def __init__(self, arm):
        self.regs = {m['motor_id']: {rid: int(m['raw'][name], 16) for rid, name in trial.REGISTERS.items()}
                     for m in arm['motors']}
        self.states = {i: 0 for i in self.regs}
        self.queue, self.sent = [], []
        self.ignore = set()
        self.drop_reads = set()
        self.drop_ack = False
        self.ack_sender = None

    def send(self, msg, timeout=None):
        data = bytes(msg.data)
        self.sent.append((msg.arbitration_id, data))
        if msg.arbitration_id != 0x7FF:
            raise AssertionError('No control frames permitted')
        motor, op, rid, raw = struct.unpack('<HBBI', data)
        sender = motor + 16
        if op == 0xCC:
            data = bytes([motor | self.states[motor] << 4, 0x7f, 0xff, 0x7f, 0xf7, 0xff, 25, 25])
        elif op == 0x33:
            if (motor, rid) in self.drop_reads: return
            data = struct.pack('<HBBI', motor, op, rid, self.regs[motor][rid])
        elif op == 0x55:
            if self.states[motor] != 0: raise AssertionError('Write to enabled motor')
            if (motor, rid) not in self.ignore: self.regs[motor][rid] = raw
            if self.drop_ack: return
            if self.ack_sender is not None: sender = self.ack_sender
        else:
            raise AssertionError('Unexpected command opcode')
        self.queue.append(SimpleNamespace(arbitration_id=sender, data=data, dlc=8,
                                         is_extended_id=False, is_remote_frame=False, is_error_frame=False))

    def recv(self, timeout=0):
        if self.queue: return self.queue.pop(0)
        if timeout: time.sleep(timeout)
        return None


class TrialTests(unittest.TestCase):
    def setUp(self):
        self.baseline = json.loads(trial.BASELINE.read_text())
        self.buses, self.controls, arms = {}, {}, {}
        for role in trial.ROLES:
            bus = self.buses[role] = Bus(self.baseline['arms'][role])
            arm = arms[role] = dict(tx=[], actions=[])
            self.controls[role] = trial.GainControl(bus=bus, tx=arm['tx'])
        self.report = dict(arms=arms)

    def run_trial(self):
        with redirect_stdout(io.StringIO()):
            trial.transact(self.controls, self.report, self.baseline, timeout=.0005)

    def writes(self):
        return [(role, data) for role,b in self.buses.items() for _,data in b.sent if data[2]==0x55]

    def test_exact_gains_both_arms_other_registers_unchanged(self):
        originals = {r: copy.deepcopy(b.regs) for r,b in self.buses.items()}
        self.run_trial()
        self.assertEqual(self.report['status'], 'applied_verified')
        self.assertEqual(len(self.writes()), 18)
        for role,bus in self.buses.items():
            for motor,regs in bus.regs.items():
                for rid,val in regs.items():
                    expected = originals[role][motor][rid]
                    if motor in trial.GAINS and rid in trial.GAIN_RIDS.values():
                        by_rid = dict(zip(trial.GAIN_RIDS.values(),trial.GAINS[motor]))
                        expected = trial.bits(by_rid[rid])
                    self.assertEqual(val, expected)

    def test_enabled_motor_in_second_arm_prevents_all_writes(self):
        self.buses['motion_right'].states[7] = 1
        with self.assertRaises(RuntimeError): self.run_trial()
        self.assertEqual(self.writes(), [])

    def test_changed_baseline_prevents_all_writes(self):
        self.buses['motion_right'].regs[2][27] = trial.bits(130.)
        with self.assertRaises(RuntimeError): self.run_trial()
        self.assertEqual(self.writes(), [])

    def test_missing_read_prevents_all_writes(self):
        self.buses['motion_right'].drop_reads.add((4,26))
        with self.assertRaises(RuntimeError): self.run_trial()
        self.assertEqual(self.writes(), [])

    def test_echoed_rejected_write_stops_before_next_write(self):
        self.buses['motion_left'].ignore.add((1,27))
        with self.assertRaisesRegex(RuntimeError,'readback mismatch'): self.run_trial()
        self.assertEqual(len(self.writes()), 1)

    def test_small_gain_exact_readback_not_loose_ack_tolerance(self):
        self.buses['motion_left'].ignore.add((2,26))
        with self.assertRaisesRegex(RuntimeError,'readback mismatch'): self.run_trial()
        self.assertEqual(len(self.writes()), 4)
        self.assertEqual(self.buses['motion_left'].regs[3][27], trial.bits(54.))

    def test_missing_ack_can_only_succeed_with_independent_readbacks(self):
        self.buses['motion_left'].drop_ack = True
        self.run_trial()
        self.assertEqual(self.report['status'], 'applied_verified')
        for action in self.report['arms']['motion_left']['actions']:
            self.assertFalse(action['ack'])
            self.assertEqual(len(action['readbacks']), 2)

    def test_frame_allowlist_and_one_shot_write_gate(self):
        c = self.controls['motion_left']
        bad = [trial.payload(5,27,trial.bits(2500.)),trial.payload(1,22,trial.bits(50.)),
               struct.pack('<HBBI',1,0xaa,1,0), b'\xff'*7+b'\xfc']
        for data in bad:
            c.armed = data
            with self.assertRaises(RuntimeError): c.send_std(0x7ff,data)
        allowed = trial.payload(1,27,trial.bits(2500.))
        c.armed = None
        with self.assertRaises(RuntimeError): c.send_std(0x7ff,allowed)
        c.armed = allowed
        c.send_std(0x7ff,allowed)
        with self.assertRaises(RuntimeError): c.send_std(0x7ff,allowed)
        self.assertEqual(len(self.writes()),1)


if __name__=='__main__': unittest.main()
