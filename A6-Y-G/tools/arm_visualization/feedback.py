"""Continuous queries through the existing status-only transmission allowlist."""
import time

from model import JOINTS, convert_position, out_of_limits
from scanner import decode_status, status_payload


def read_cycle(control, model, cancel, timeout=.04):
    rows = {}
    for name in JOINTS:
        if cancel.is_set():
            break
        entry = model['joints'][name]
        motor_id = entry['motor_id']
        # Never reuse an earlier query's buffered reply as a new sample.
        for _ in range(512):
            if control.bus.recv(timeout=0) is None:
                break
        else:
            raise RuntimeError('接收队列持续繁忙，请停止其他总线通信')
        control.send_std(0x7ff, status_payload(motor_id))
        deadline = time.monotonic() + timeout
        row = {'name': name, 'motor_id': motor_id, 'feedback': None,
               'q': None, 'sample_monotonic': None, 'out_of_limits': False}
        while not cancel.is_set() and time.monotonic() < deadline:
            msg = control.bus.recv(timeout=min(.01, max(0, deadline - time.monotonic())))
            if msg is None:
                continue
            feedback = decode_status(msg, motor_id, entry['position_scale_evidence'])
            if feedback is None:
                continue
            q = convert_position(feedback['position_rad'], entry)
            row.update(feedback=feedback, q=q, sample_monotonic=time.monotonic(),
                       out_of_limits=out_of_limits(q, entry))
            break
        rows[name] = row
    return rows


def complete_positions(rows, now=None, max_age=.4):
    """Publish only a complete fresh sweep, including visible fault-state angles."""
    now = time.monotonic() if now is None else now
    if set(rows) != set(JOINTS):
        return None
    for row in rows.values():
        if (row['q'] is None or row['sample_monotonic'] is None
                or not 0 <= now - row['sample_monotonic'] <= max_age):
            return None
    return [rows[name]['q'] for name in JOINTS]
