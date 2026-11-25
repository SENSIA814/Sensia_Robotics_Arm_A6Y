#!/usr/bin/env python3
"""Apply the conservative J4 retry gains to both motion arms, RAM only.

Default is offline preview. --apply requires both USB identities, exclusive
ownership, fresh disabled states and the last reviewed parameter baseline.
No enable, disable, motion, zero, range, ID, mode or Flash commands are allowed.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import struct
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parent.parent
ROOT = WORKSPACE / 'A6-Y-G'
sys.path.insert(0, str(ROOT / 'tools/arm_diagnostics'))
sys.path.insert(0, str(ROOT / 'tools'))
from scanner import load_roles, open_arm_control, timestamp, MotorControl
from operation_lock import exclusive_operation
from read_motor_position_params import REGISTERS, read_frame_allowed, decode_parameter, IdProgrammer

ROLES = {'motion_left': '304C33733036', 'motion_right': '209B3971534B'}
GAIN_RIDS = {'KP_APR': 27, 'KI_APR': 28, 'KP_ASR': 25, 'KI_ASR': 26}
GAINS = {4: (54., 0., .00372, .002)}
BASELINE = HERE / 'motion_parameters_1795619145890042556.json'
EVIDENCE = WORKSPACE / 'diagnostics/teaching_tuning_20261125/motion_parameters_1795616484295559165.json'



def bits(value):
    return struct.unpack('<I', struct.pack('<f', value))[0]


def payload(motor, rid, raw):
    return struct.pack('<HBBI', motor, 0x55, rid, raw)


APPROVED = {payload(m, GAIN_RIDS[n], bits(v)) for m, vals in GAINS.items()
            for n, v in zip(GAIN_RIDS, vals)}


class GainControl(MotorControl):
    def __init__(self, *args, tx=None, **kwargs):
        self.tx = tx if tx is not None else []
        self.armed = None
        self.audit = lambda x: None
        self.role = None
        super().__init__(*args, **kwargs)

    def send_std(self, arbitration_id, data, *, pad_to_8=True):
        data = bytes(data)
        readonly = any(read_frame_allowed(i, arbitration_id, data) for i in range(1, 8))
        if not readonly:
            if arbitration_id != 0x7FF or data not in APPROVED or data != self.armed:
                raise RuntimeError('Blocked command outside reviewed J4 gain writes')
            self.armed = None
        entry = dict(role=self.role, at=timestamp(), can_id=arbitration_id,
                     data=data.hex(), kind='read' if readonly else 'gain_write', sent=False)
        self.tx.append(entry)
        self.audit(dict(event='tx_attempt', **entry))
        super().send_std(arbitration_id, data, pad_to_8=pad_to_8)
        entry['sent'] = True


def disabled(reader):
    states = {str(i): reader.read_state(i, i + 16) for i in range(1, 8)}
    if any(v != 0 for v in states.values()):
        raise RuntimeError(f'Motors must already be disabled; no disable sent: {states}')
    return states


def snapshot(reader):
    motors = {}
    for i in range(1, 8):
        reader.read_pair(i, i + 16)
        m = motors[str(i)] = dict(parameters={}, raw={})
        for rid, name in REGISTERS.items():
            _, raw = reader.read_uint(i, rid, i + 16)
            m['raw'][name] = f'{raw:08x}'
            m['parameters'][name] = decode_parameter(rid, raw)
    return motors


def collect(controls, report, phase, timeout):
    def one(role):
        reader = IdProgrammer(controls[role], timeout=timeout, read_attempts=3, report=lambda x: None)
        arm = report['arms'][role]
        arm[phase + '_states_begin'] = disabled(reader)
        arm[phase] = snapshot(reader)
        arm[phase + '_states_end'] = disabled(reader)
        print(role, phase, 'snapshot complete; all seven disabled', flush=True)
    # Independent adapters; no writes occur inside these workers.
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = [pool.submit(one, role) for role in ROLES]
        errors = []
        for f in futures:
            try: f.result()
            except Exception as exc: errors.append(exc)
        if errors: raise errors[0]


def transact(controls, report, baseline, checkpoint=lambda: None, timeout=.1):
    collect(controls, report, 'before', timeout)
    for role in ROLES:
        old = {str(m['motor_id']): m for m in baseline['arms'][role]['motors']}
        for motor, entry in report['arms'][role]['before'].items():
            if entry['raw'] != old[motor]['raw']:
                raise RuntimeError(f'{role} J{motor}: baseline changed since review; no writes permitted')
            if int(motor) in GAINS and entry['parameters']['CTRL_MODE'] != 2:
                raise RuntimeError(f'{role} J{motor}: expected POS_VEL; no mode change permitted')
    # Complete baseline for BOTH arms is durably saved before any write.
    report['status'] = 'baseline_verified'
    checkpoint()
    for role in ROLES:
        control = controls[role]
        reader = IdProgrammer(control, timeout=timeout, read_attempts=3, report=lambda x: None)
        arm = report['arms'][role]
        arm['pre_write_states'] = disabled(reader)
        for motor, values in GAINS.items():
            for name, value in zip(GAIN_RIDS, values):
                raw_before = int(arm['before'][str(motor)]['raw'][name], 16)
                if raw_before == bits(value):
                    continue
                # Guard immediately before every individual write.
                if reader.read_state(motor, motor + 16) != 0:
                    raise RuntimeError(f'{role} J{motor} no longer disabled; stopped')
                _, fresh = reader.read_uint(motor, GAIN_RIDS[name], motor + 16)
                if fresh != raw_before:
                    raise RuntimeError(f'{role} J{motor} {name}: changed during operation; stopped')
                action = dict(motor=motor, parameter=name, before=decode_parameter(GAIN_RIDS[name], fresh),
                              requested=value, status='pending', readbacks=[])
                arm['actions'].append(action)
                report['status'] = 'writing'
                checkpoint()
                data = payload(motor, GAIN_RIDS[name], bits(value))
                reader.clear_rx()
                action['status'] = 'write_attempted'
                control.armed = data
                control.send_std(0x7FF, data)
                action['ack'] = False
                for msg in reader.receive():
                    if reader.frame_data(msg) == data and msg.arbitration_id == motor + 16:
                        action['ack'] = True
                for _ in range(2):
                    _, raw = reader.read_uint(motor, GAIN_RIDS[name], motor + 16)
                    action['readbacks'].append(f'{raw:08x}')
                    if raw != bits(value):
                        raise RuntimeError(f'{role} J{motor} {name}: independent readback mismatch; stopped')
                action['status'] = 'readback_verified'
                checkpoint()
                print(role, 'J'+str(motor), name, action['before'], '->', value, 'verified', flush=True)
    collect(controls, report, 'after', timeout)
    for role in ROLES:
        arm = report['arms'][role]
        arm['changed_registers'] = []
        for motor, entry in arm['before'].items():
            for name, raw in entry['raw'].items():
                expected = raw
                if int(motor) in GAINS and name in GAIN_RIDS:
                    expected = f'{bits(dict(zip(GAIN_RIDS, GAINS[int(motor)]))[name]):08x}'
                actual = arm['after'][motor]['raw'][name]
                if actual != expected:
                    raise RuntimeError(f'{role} J{motor} {name}: final snapshot mismatch')
                if actual != raw:
                    arm['changed_registers'].append(dict(motor=int(motor), parameter=name))
    report['status'] = 'applied_verified'
    checkpoint()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    roles = load_roles()
    baseline = json.loads(BASELINE.read_text())
    sources = {str(BASELINE): hashlib.sha256(BASELINE.read_bytes()).hexdigest()}
    for role, serial in ROLES.items():
        if roles[role]['usb_serial'] != serial or baseline['arms'][role]['usb_serial'] != serial:
            raise RuntimeError('USB role differs from reviewed identity')
    # Both arms use the earlier left J4 set. Right KP_APR is also reduced
    # below its earlier 120; no manufacturer's universal tuning claim.
    source = json.loads(EVIDENCE.read_text())['arms']['motion_left']['motors'][3]
    if source['motor_id'] != 4: raise RuntimeError('Evidence motor mismatch')
    for name, value in zip(GAIN_RIDS, GAINS[4]):
        if source['raw'][name] != f'{bits(value):08x}':
            raise RuntimeError('Plan differs from reviewed prior left J4 gains')
    sources[str(EVIDENCE)] = hashlib.sha256(EVIDENCE.read_bytes()).hexdigest()
    report = dict(schema_version=1, trial='j4_oscillation_retry_02', started_at=timestamp(),
                  status='offline_preview', reference_session='teaching_cc3c0a6bfe',
                  parameter_source_hashes=sources, planned_gains={str(m): dict(zip(GAIN_RIDS, v)) for m,v in GAINS.items()},
                  storage='RAM only; lost on motor power cycle', arms={},
                  rates=dict(control_hz=250, per_axis_command_hz=250, target_hz=100, can_bitrate=1000000))
    if not args.apply:
        print(json.dumps(report, ensure_ascii=False, indent=2)); return 0
    output = HERE / f'{time.time_ns()}_apply'
    output.mkdir(parents=True)
    audit_lock = threading.Lock()
    with (output / 'audit.jsonl').open('x', encoding='utf-8') as audit:
        def log(event):
            with audit_lock:
                audit.write(json.dumps(event, ensure_ascii=False) + '\n'); audit.flush(); os.fsync(audit.fileno())
        def checkpoint():
            temp = output / 'result.tmp'
            with temp.open('w', encoding='utf-8') as f:
                json.dump(report, f, ensure_ascii=False, indent=2); f.flush(); os.fsync(f.fileno())
            temp.replace(output / 'result.json')
        try:
            with exclusive_operation(), ExitStack() as stack:
                controls = {}
                for role in ROLES:
                    arm = report['arms'][role] = dict(roles[role], tx=[], actions=[])
                    control = stack.enter_context(open_arm_control(arm, GainControl))
                    control.audit, control.role = log, role
                    controls[role] = control
                transact(controls, report, baseline, checkpoint)
        except (Exception, KeyboardInterrupt) as exc:
            report['error'] = str(exc) or type(exc).__name__
            wrote = any(t['kind']=='gain_write' for a in report['arms'].values() for t in a['tx'])
            report['status'] = 'partial_or_unconfirmed_do_not_start' if wrote else 'blocked_no_writes'
        finally:
            report['ended_at'] = timestamp()
            report['gain_write_attempts'] = sum(t['kind']=='gain_write' for a in report['arms'].values() for t in a['tx'])
            report['enable_disable_motion_flash_frames'] = 0
            checkpoint()
            if all('before' in a for a in report['arms'].values()) and len(report['arms'])==2:
                rollback = {role: {m: {n: a['before'][m]['parameters'][n] for n in GAIN_RIDS}
                                   for m in map(str, GAINS)} for role,a in report['arms'].items()}
                (output/'rollback_values.json').write_text(json.dumps(rollback,ensure_ascii=False,indent=2))
            log(dict(event='finished', status=report['status'], at=report['ended_at']))
    print(json.dumps({k:report.get(k) for k in ('status','error','gain_write_attempts','enable_disable_motion_flash_frames')},ensure_ascii=False))
    print('Result:',output/'result.json',flush=True)
    return 0 if report['status']=='applied_verified' else 1


if __name__=='__main__': raise SystemExit(main())
