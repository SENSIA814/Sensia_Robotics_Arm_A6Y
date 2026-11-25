#!/usr/bin/env python3
"""Authorized one-off read-only scan of the four USB-bound arms."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]/'A6-Y-G'
sys.path.insert(0,str(ROOT/'tools/arm_diagnostics'))
sys.path.insert(0,str(ROOT/'tools'))
from scanner import load_roles, open_arm_control, timestamp, MotorControl
from operation_lock import exclusive_operation
from read_motor_position_params import REGISTERS, read_frame_allowed, decode_parameter, IdProgrammer


class ReadOnlyControl(MotorControl):
    def __init__(self,*args,tx,**kwargs):
        self.tx=tx
        super().__init__(*args,**kwargs)

    def send_std(self,arbitration_id,payload,*,pad_to_8=True):
        data=bytes(payload)
        if not any(read_frame_allowed(i,arbitration_id,data) for i in range(1,8)):
            raise RuntimeError('Blocked non-read motor frame')
        super().send_std(arbitration_id,data,pad_to_8=pad_to_8)
        self.tx.append(dict(can_id=arbitration_id,data=data.hex(),at=timestamp()))


def scan(arm):
    arm=dict(arm,tx=[],motors=[],started_at=timestamp(),error=None)
    try:
        with open_arm_control(arm,ReadOnlyControl) as control:
            reader=IdProgrammer(control,timeout=.1,read_attempts=3,report=lambda s:None)
            for motor in range(1,8):
                record=dict(motor_id=motor,parameters={},raw={},errors={})
                arm['motors'].append(record)
                reader.read_pair(motor,motor+16)
                record['state_before']=reader.read_state(motor,motor+16)
                for rid,name in REGISTERS.items():
                    try:
                        _,value=reader.read_uint(motor,rid,motor+16)
                        record['parameters'][name]=decode_parameter(rid,value)
                        record['raw'][name]=f'{value:08x}'
                    except (RuntimeError,ValueError) as exc:
                        record['errors'][name]=str(exc)
                record['state_after']=reader.read_state(motor,motor+16)
                print(arm['role'],motor,'KP_APR=',record['parameters'].get('KP_APR'),
                      'errors=',record['errors'],flush=True)
    except Exception as exc:
        arm['error']=str(exc)
    arm['ended_at']=timestamp()
    arm['complete']=len(arm['motors'])==7 and not arm['error'] and all(not m['errors'] for m in arm['motors'])
    return arm


if __name__=='__main__':
    output=Path(__file__).with_name('motor_parameters.json')
    if output.exists():raise SystemExit('Refusing to overwrite previous scan')
    report=dict(schema_version=1,observed_at=timestamp(),readonly=True,motor_writes_sent=0,
                enable_sent=0,flash_save_sent=0,arms={})
    with exclusive_operation():
        with ThreadPoolExecutor(max_workers=4) as pool:
            for arm in pool.map(scan,load_roles().values()):
                report['arms'][arm['role']]=arm
    report['complete']=all(a['complete'] for a in report['arms'].values())
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print('Saved',output,'complete=',report['complete'],flush=True)
    raise SystemExit(0 if report['complete'] else 1)
