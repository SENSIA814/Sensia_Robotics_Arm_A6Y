#!/usr/bin/env python3
"""Continuous raw feedback diagnosis; only status requests are permitted on CAN."""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import sys
import time
from contextlib import ExitStack
from pathlib import Path

from damiao_bus_scan import find_driver_root, load_targets, apply_protocol_limits


def positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--channel', default='/dev/ttyACM0')
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--duration', type=positive_float, default=20.0)
    parser.add_argument('--interval', type=positive_float, default=0.5, help='Print interval, seconds.')
    parser.add_argument('--log', type=Path, required=True, help='New JSONL file; existing files are never overwritten.')
    expected = parser.add_mutually_exclusive_group()
    expected.add_argument('--expect-id', type=lambda s: int(s, 0))
    expected.add_argument('--expect-joint', help='Resolve the expected motor ID from the joint config.')
    parser.add_argument('--min-span', type=positive_float, default=0.02)
    args = parser.parse_args()
    targets = load_targets(args.config)
    ids = [t.slave_id for t in targets]
    masters = [t.master_id for t in targets]
    if (not targets or len(set(ids)) != len(ids) or len(set(masters)) != len(masters)
            or not all(1 <= i <= 15 for i in ids) or not all(1 <= m <= 0x7FF for m in masters)
            or set(ids) & set(masters)):
        parser.error('unique 4-bit motor IDs and unique, distinct nonzero feedback IDs are required')
    if args.expect_joint is not None:
        matching = [t for t in targets if t.name == args.expect_joint]
        if len(matching) != 1:
            parser.error('--expect-joint must name exactly one configured joint')
        args.expect_id = matching[0].slave_id
    if args.expect_id is not None and args.expect_id not in ids:
        parser.error('--expect-id is absent from --config')
    sys.path.insert(0, str(find_driver_root()))
    from dm_canable2 import DM_Motor_Type, Motor, MotorControl
    from dm_canable2.core import parse_feedback

    motors = {t.slave_id: Motor(DM_Motor_Type[t.motor_type], t.slave_id, t.master_id) for t in targets}
    for target in targets:
        apply_protocol_limits(motors[target.slave_id], target)
    feedback_ids = {t.master_id: t.slave_id for t in targets}
    stats = {i: {'count': 0, 'q_min': None, 'q_max': None, 'last': None, 'states': set()} for i in ids}
    origin = time.monotonic()
    unexpected = 0

    with ExitStack() as stack:
        log = stack.enter_context(args.log.open('x', encoding='utf-8', buffering=1))
        def record(event):
            log.write(json.dumps(event, ensure_ascii=False) + '\n')
        record({'event': 'start', 'channel': args.channel, 'config': str(args.config),
                'status_requests_only': True, 'duration': args.duration, 'expected_id': args.expect_id, 'expected_joint': args.expect_joint,
                'targets': [vars(t) for t in targets]})

        class ReadOnlyControl(MotorControl):
            def send_std(self, arbitration_id, payload, *, pad_to_8=True):
                payload = bytes(payload)
                allowed = [bytes([i, 0, 0xCC, 0, 0, 0, 0, 0]) for i in ids]
                if arbitration_id != 0x7FF or payload not in allowed:
                    raise RuntimeError('Non-status CAN command blocked')
                return super().send_std(arbitration_id, payload, pad_to_8=pad_to_8)

            def _process_rx_message(self, msg):
                nonlocal unexpected
                data = bytes(msg.data)
                elapsed = time.monotonic() - origin
                motor_id = feedback_ids.get(int(msg.arbitration_id))
                valid = (motor_id is not None and len(data) == 8 and not msg.is_extended_id
                         and not msg.is_remote_frame and not msg.is_error_frame
                         and (data[0] & 15) == motor_id)
                event = {'event': 'rx', 't': elapsed, 'can_id': int(msg.arbitration_id),
                         'data': data.hex().upper(), 'accepted': valid}
                if not valid:
                    unexpected += 1
                    record(event)
                    return
                motor = motors[motor_id]
                # IDs and frame shape already identify feedback; avoid parameter-frame heuristics.
                feedback = parse_feedback(data, motor.limit_param)
                motor.receive_data(feedback.q, feedback.dq, feedback.tau, feedback.err)
                q = motor.getPosition()
                entry = stats[motor_id]
                entry['count'] += 1
                entry['q_min'] = q if entry['q_min'] is None else min(entry['q_min'], q)
                entry['q_max'] = q if entry['q_max'] is None else max(entry['q_max'], q)
                entry['last'] = elapsed
                entry['states'].add(motor.getError())
                event.update({'id': motor_id, 'q': q, 'q_u16': data[1] << 8 | data[2],
                              'dq': motor.getVelocity(), 'tau': motor.getTorque(), 'err': motor.getError()})
                record(event)

        # Honor the hardware plugin's advisory device lock before SLCAN initialization.
        device = stack.enter_context(open(args.channel, 'rb', buffering=0))
        fcntl.flock(device.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        control = ReadOnlyControl(channel=args.channel, auto_open=False, receive_timeout=0.01)
        stack.callback(control.close)
        for motor in motors.values():
            control.addMotor(motor)
        interrupted = False
        try:
            control.open()
            origin = time.monotonic()
            next_print = 0.0
            print('READ ONLY: connection ready. Move the intended joint now; raw q in radians.', flush=True)
            while time.monotonic() - origin < args.duration:
                for motor in motors.values():
                    control.refresh_motor_status(motor)
                    control.drain_rx(timeout=0.0, limit=64)
                elapsed = time.monotonic() - origin
                if elapsed >= next_print:
                    cells = []
                    for i in ids:
                        entry = stats[i]
                        if not entry['count']:
                            cell = f'ID{i:02X}=MISS'
                        elif elapsed - entry['last'] > 0.5:
                            cell = f'ID{i:02X}=STALE'
                        else:
                            cell = f'ID{i:02X}={motors[i].getPosition():+.6f}'
                        cells.append(cell)
                    print(f'{elapsed:5.1f}s ' + '  '.join(cells), flush=True)
                    next_print = elapsed + args.interval
                time.sleep(0.01)
        except KeyboardInterrupt:
            interrupted = True
            print('\nInterrupted; closing connection.', flush=True)
        elapsed = time.monotonic() - origin
        summary = {}
        for i, entry in stats.items():
            span = entry['q_max'] - entry['q_min'] if entry['count'] else None
            summary[str(i)] = {**entry, 'states': sorted(entry['states']), 'q_span': span,
                               'age': elapsed - entry['last'] if entry['last'] is not None else None}
            span_text = f'{span:.6f}' if span is not None else 'NA'
            print(f"SUMMARY ID=0x{i:02X} frames={entry['count']} span={span_text} rad states={sorted(entry['states'])}", flush=True)
        communication_ok = (unexpected == 0 and all(v['count'] and v['age'] <= 0.5 and v['states'] == [0]
                                                   for v in summary.values()))
        motion_ok = None
        if args.expect_id is not None:
            span = summary[str(args.expect_id)]['q_span']
            motion_ok = span is not None and span >= args.min_span
            print(f'EXPECTED ID=0x{args.expect_id:02X}: ' + ('CHANGE_SEEN' if motion_ok else 'NO_SUFFICIENT_CHANGE')
                  + ' (interpret only if physical movement occurred during capture)', flush=True)
        record({'event': 'summary', 'motors': summary, 'unexpected_frames': unexpected,
                'communication_ok': communication_ok, 'motion_change_seen': motion_ok,
                'interrupted': interrupted})
        return 130 if interrupted else (0 if communication_ok and motion_ok is not False else 1)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (OSError, RuntimeError, ValueError, KeyError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(2)
