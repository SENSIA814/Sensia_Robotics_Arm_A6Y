#!/usr/bin/env python3
"""Round 01: one RAM-only KP_APR change on USB-bound motion_left J2.

No enable, disable, motion, zeroing or Flash frames are permitted. An already
disabled arm and fresh expected register value are mandatory. Default is an
offline preview; --check reads only; --apply writes 54 -> 60; --rollback writes
60 -> 54. Every invocation keeps a separate audit and full parameter snapshot.
"""
import argparse
import json
import os
from pathlib import Path
import struct
import sys
import time

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parent.parent
ROOT = WORKSPACE / 'A6-Y-G'
sys.path.insert(0, str(ROOT / 'tools/arm_diagnostics'))
sys.path.insert(0, str(ROOT / 'tools'))
from scanner import load_roles, open_arm_control, timestamp, MotorControl
from operation_lock import exclusive_operation
from read_motor_position_params import REGISTERS, read_frame_allowed, decode_parameter, IdProgrammer

USB_SERIAL = '304C33733036'
MOTOR = 2
RID = 27
BASELINE = 54.0
CANDIDATE = 60.0


def float_bits(value):
    return struct.unpack('<I', struct.pack('<f', value))[0]


class RoundControl(MotorControl):
    def __init__(self, *args, tx=None, **kwargs):
        self.tx = tx if tx is not None else []
        self.armed_payload = None
        self.audit = lambda entry: None
        super().__init__(*args, **kwargs)

    def send_std(self, arbitration_id, payload, *, pad_to_8=True):
        data = bytes(payload)
        readonly = any(read_frame_allowed(i, arbitration_id, data) for i in range(1, 8))
        if not readonly:
            allowed = {struct.pack('<HBBI', MOTOR, 0x55, RID, float_bits(v))
                       for v in (BASELINE, CANDIDATE)}
            if arbitration_id != 0x7FF or data != self.armed_payload or data not in allowed:
                raise RuntimeError('Blocked frame outside the single approved RAM change')
            self.armed_payload = None  # At most one write; never retry a write.
        entry = dict(can_id=arbitration_id, data=data.hex(), at=timestamp(),
                     kind='read' if readonly else 'ram_write', sent=False)
        self.tx.append(entry)
        self.audit(dict(event='tx_attempt', **entry))  # Durable BEFORE transport.
        super().send_std(arbitration_id, data, pad_to_8=pad_to_8)
        entry['sent'] = True


def states_disabled(reader):
    states = {str(i): reader.read_state(i, i + 16) for i in range(1, 8)}
    if any(s != 0 for s in states.values()):
        raise RuntimeError(f'Arm must already be disabled; no disable is sent: {states}')
    return states


def snapshot(reader):
    result = {}
    for motor in range(1, 8):
        reader.read_pair(motor, motor + 16)
        entry = result[str(motor)] = dict(parameters={}, raw={})
        for rid, name in REGISTERS.items():
            _, raw = reader.read_uint(motor, rid, motor + 16)
            entry['raw'][name] = f'{raw:08x}'
            entry['parameters'][name] = decode_parameter(rid, raw)
    return result


def execute(control, report, mode, timeout=.1, checkpoint=lambda: None):
    reader = IdProgrammer(control, timeout=timeout, read_attempts=3, report=lambda s: None)
    report['states_before'] = states_disabled(reader)
    report['before'] = snapshot(reader)
    expected, desired = ((CANDIDATE, BASELINE) if mode == 'rollback' else (BASELINE, CANDIDATE))
    report.update(expected_before=expected, requested_after=desired)
    checkpoint()
    if mode == 'check':
        report['states_after'] = states_disabled(reader)
        report['status'] = 'read_only_complete'
        return
    before = report['before'][str(MOTOR)]
    if before['raw']['KP_APR'] != f'{float_bits(expected):08x}':
        raise RuntimeError(f"Expected KP_APR={expected}, freshly read {before['parameters']['KP_APR']}; no write")
    if before['parameters']['CTRL_MODE'] != 2:
        raise RuntimeError('Target must already be in POS_VEL mode; no mode change is sent')
    # Check the entire arm again, with the target last immediately before write.
    report['states_pre_write'] = states_disabled(reader)
    if reader.read_state(MOTOR, MOTOR + 16) != 0:
        raise RuntimeError('Target is no longer disabled; no write')
    payload = struct.pack('<HBBI', MOTOR, 0x55, RID, float_bits(desired))
    reader.clear_rx()
    report['status'] = 'write_attempt_pending'
    checkpoint()
    control.armed_payload = payload
    control.send_std(0x7FF, payload)
    report['ack_received'] = False
    for msg in reader.receive():
        if reader.frame_data(msg) == payload and msg.arbitration_id == MOTOR + 16:
            report['ack_received'] = True
    # A write echo alone is not proof. Require two independent register reads.
    report['verification_reads'] = []
    for _ in range(2):
        _, raw = reader.read_uint(MOTOR, RID, MOTOR + 16)
        report['verification_reads'].append(f'{raw:08x}')
        if raw != float_bits(desired):
            raise RuntimeError('Fresh KP_APR readback mismatch; do not start a trial')
    report['after'] = snapshot(reader)
    changes = []
    for motor, entry in report['before'].items():
        for name, value in entry['raw'].items():
            if report['after'][motor]['raw'][name] != value:
                changes.append((motor, name))
    report['changed_registers'] = changes
    if changes != [(str(MOTOR), 'KP_APR')]:
        raise RuntimeError(f'Unexpected parameter changes: {changes}')
    report['states_after'] = states_disabled(reader)
    report['status'] = 'applied_verified' if mode == 'apply' else 'rollback_verified'
    checkpoint()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    for flag in ('check', 'apply', 'rollback'):
        group.add_argument('--' + flag, action='store_true')
    args = parser.parse_args()
    mode = next((x for x in ('check', 'apply', 'rollback') if getattr(args, x)), 'preview')
    arm = load_roles()['motion_left']
    if arm['usb_serial'] != USB_SERIAL:
        raise RuntimeError('USB role binding differs from approved first-round plan')
    report = dict(schema_version=1, round='01', mode=mode, role='motion_left',
                  usb_serial=USB_SERIAL, motor_id=MOTOR, joint='J2', parameter='KP_APR',
                  baseline=BASELINE, candidate=CANDIDATE, flash_save=False,
                  enable_sent=False, disable_sent=False, motion_sent=False,
                  reference_session='teaching_394b0c668f', started_at=timestamp(),
                  status='pending', tx=[])
    if mode == 'preview':
        report['status'] = 'offline_preview'
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    output = HERE / f'{time.time_ns()}_{mode}'
    output.mkdir()
    arm['tx'] = report['tx']
    with (output / 'audit.jsonl').open('x', encoding='utf-8') as audit:
        def log(event):
            audit.write(json.dumps(event, ensure_ascii=False) + '\n')
            audit.flush()
            os.fsync(audit.fileno())

        def checkpoint():
            path = output / 'result.json'
            temp = output / 'result.tmp'
            with temp.open('w', encoding='utf-8') as stream:
                json.dump(report, stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            temp.replace(path)

        try:
            checkpoint()
            with exclusive_operation(), open_arm_control(arm, RoundControl) as control:
                control.audit = log
                report['device'] = arm['device']
                execute(control, report, mode, checkpoint=checkpoint)
        except (Exception, KeyboardInterrupt) as exc:
            report['error'] = str(exc) or type(exc).__name__
            report['status'] = ('write_unconfirmed' if any(x['kind'] == 'ram_write' for x in report['tx'])
                                else 'blocked_before_write')
        finally:
            report['ended_at'] = timestamp()
            report['ram_write_attempts'] = sum(x['kind'] == 'ram_write' for x in report['tx'])
            checkpoint()
            log(dict(event='finished', status=report['status'], at=report['ended_at']))
    print(json.dumps({k: report.get(k) for k in ('status', 'error', 'ram_write_attempts',
                    'ack_received', 'verification_reads', 'changed_registers', 'states_after')},
                    ensure_ascii=False, indent=2))
    print('Result:', output / 'result.json', flush=True)
    return 0 if report['status'] in ('read_only_complete', 'applied_verified', 'rollback_verified') else 1


if __name__ == '__main__':
    raise SystemExit(main())
