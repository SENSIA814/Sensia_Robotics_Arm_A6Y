#!/usr/bin/env python3
"""Read ID 1..7 and decode feedback using each motor's measured protocol limits.

Only CAN ID 0x7ff status (0xcc) and selected register reads (0x33) may be sent.
SLCAN initialization opens/closes the adapter; no motor enable/disable is sent.
"""
import argparse
import datetime
import fcntl
import json
import math
from pathlib import Path
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'A6-Y-G/src/a6_y_ros2_control/tools'))
from damiao_bus_scan import find_driver_root
from damiao_set_id import IdProgrammer
sys.path.insert(0, str(find_driver_root()))
from dm_canable2 import MotorControl
from dm_canable2.core import parse_feedback

REGISTERS = {7: 'MST_ID', 8: 'ESC_ID', 10: 'CTRL_MODE', 21: 'PMAX',
             22: 'VMAX', 23: 'TMAX', 25: 'KP_ASR', 26: 'KI_ASR', 27: 'KP_APR'}


def probe(port):
    record = {'port': str(port), 'device': str(port.resolve()), 'motors': [], 'tx': []}
    busy = subprocess.run(['fuser', str(port)], capture_output=True, timeout=3)
    if busy.returncode != 1:
        record['error'] = 'Device occupied or occupancy check failed'
        return record

    class ReadOnlyControl(MotorControl):
        def send_std(self, arbitration_id, payload, *, pad_to_8=True):
            data = bytes(payload)
            allowed = (arbitration_id == 0x7ff and len(data) == 8
                       and 1 <= data[0] <= 7 and data[1] == 0 and data[4:] == bytes(4)
                       and ((data[2] == 0x33 and data[3] in REGISTERS)
                            or (data[2] == 0xcc and data[3] == 0)))
            if not allowed:
                raise RuntimeError('Non-read CAN frame blocked')
            record['tx'].append({'id': arbitration_id, 'data': data.hex()})
            return super().send_std(arbitration_id, data, pad_to_8=pad_to_8)

    with open(port, 'rb', buffering=0) as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        with ReadOnlyControl(channel=str(port)) as control:
            reader = IdProgrammer(control, timeout=.08, read_attempts=2,
                                  report=lambda message: None)
            for slave in range(1, 8):
                item = {'requested_id': slave}
                record['motors'].append(item)
                try:
                    pair = reader.read_uint(slave, 8, required=False)
                    if pair is None:
                        item['found'] = False
                        continue
                    master, value = pair
                    if value != slave:
                        raise RuntimeError('ESC_ID mismatch')
                    item.update(found=True, feedback_id=master, parameters={'ESC_ID': value})
                    for rid, name in REGISTERS.items():
                        if rid == 8:
                            continue
                        _, raw = reader.read_uint(slave, rid, master)
                        value = raw if rid in (7, 8, 10) else struct.unpack('<f', struct.pack('<I', raw))[0]
                        if not math.isfinite(value):
                            raise RuntimeError('Non-finite register')
                        item['parameters'][name] = value
                    params = item['parameters']
                    if params['MST_ID'] != master:
                        raise RuntimeError('MST_ID mismatch')
                    limits = tuple(params[k] for k in ('PMAX', 'VMAX', 'TMAX'))
                    if min(limits) <= 0:
                        raise RuntimeError('Invalid protocol limits')
                    reader.clear_rx()
                    reader.send_param(slave, 0xcc, 0)
                    feedback = []
                    for msg in reader.receive():
                        data = reader.frame_data(msg)
                        if (data is not None and msg.arbitration_id == master
                                and data[0] & 15 == slave
                                and not (data[1] == 0 and data[2] in (0x33, 0x55, 0xaa))):
                            decoded = parse_feedback(data, limits)
                            feedback.append({'raw_hex': data.hex(), 'q_motor_rad': decoded.q,
                                             'dq_motor_rad_s': decoded.dq, 'tau_motor_Nm': decoded.tau,
                                             'state': decoded.err})
                    if not feedback:
                        raise RuntimeError('No matching fresh status')
                    item['feedback'] = feedback[-1]
                except Exception as error:
                    item['error'] = str(error)
    record['complete_7'] = all(m.get('found') and 'feedback' in m and 'error' not in m
                               for m in record['motors']) and len(record['motors']) == 7
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path, help='New JSON file')
    parser.add_argument('--serial', action='append', help='Limit to an adapter serial')
    args = parser.parse_args()
    with args.output.open('x', encoding='utf-8') as output:
        result = {'time': datetime.datetime.now().astimezone().isoformat(),
                  'readonly': True, 'motor_writes_sent': False, 'adapters': []}
        try:
            ports = sorted(Path('/dev/serial/by-id').glob('*CANable2*'))
            for port in ports:
                if args.serial and not any(s + '-if' in port.name for s in args.serial):
                    continue
                try:
                    item = probe(port)
                except Exception as error:
                    item = {'port': str(port), 'error': str(error)}
                result['adapters'].append(item)
                print(port.name, 'found', sum(bool(m.get('found')) for m in item.get('motors', [])),
                      '/7', item.get('error', ''), flush=True)
        finally:
            json.dump(result, output, ensure_ascii=False, indent=2)
            output.write('\n')


if __name__ == '__main__':
    main()
