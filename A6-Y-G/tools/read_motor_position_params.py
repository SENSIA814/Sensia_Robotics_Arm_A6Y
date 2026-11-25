#!/usr/bin/env python3
"""Read-only Damiao position-loop snapshot; no enable, write, zero or flash save."""
import argparse
from contextlib import ExitStack
import fcntl
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/a6_y_ros2_control/tools'))
from damiao_bus_scan import find_driver_root, load_targets
from damiao_set_id import IdProgrammer

REGISTERS = {
    7:'MST_ID', 8:'ESC_ID', 10:'CTRL_MODE', 13:'hw_ver', 14:'sw_ver', 36:'sub_ver',
    4:'ACC', 5:'DEC', 6:'MAX_SPD', 3:'OC_Value', 9:'TIMEOUT',
    24:'I_BW', 25:'KP_ASR', 26:'KI_ASR', 27:'KP_APR', 28:'KI_APR',
    31:'Deta', 32:'V_BW', 33:'IQ_c1', 34:'VL_c1',
    20:'Gr', 21:'PMAX', 22:'VMAX', 23:'TMAX',
}
UINT_RIDS = {7,8,9,10,13,14,36}


def read_frame_allowed(slave, can_id, payload):
    data = bytes(payload)
    return (can_id == 0x7FF and len(data) == 8 and
            data[:2] == struct.pack('<H',slave) and data[4:] == bytes(4) and
            ((data[2] == 0x33 and data[3] in REGISTERS) or
             (data[2] == 0xCC and data[3] == 0)))


def decode_parameter(rid, raw):
    value = raw if rid in UINT_RIDS else struct.unpack('<f',struct.pack('<I',raw))[0]
    if not math.isfinite(value):raise ValueError('non-finite parameter response')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--channel',default='/dev/ttyACM0')
    parser.add_argument('--joint',default='joint1')
    parser.add_argument('--config',type=Path,default=ROOT/'src/a6_y_controllers/config/a5_vr_position_full_motors.yaml')
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    targets = load_targets(args.config)
    selected = [t for t in targets if t.name == args.joint]
    if len(selected) != 1:parser.error('joint must identify one configured motor')
    t = selected[0]
    if (not 1 <= t.slave_id <= 15 or not 0 < t.master_id < 0x7FF or
            sum(x.slave_id == t.slave_id for x in targets) != 1 or
            sum(x.master_id == t.master_id for x in targets) != 1):
        parser.error('invalid or duplicate motor/feedback ID in configuration')
    output = args.output or ROOT/'validation/j1_position'/f'{time.time_ns()}_{args.joint}_params.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    report = dict(joint=t.name,motor_type=t.motor_type,slave_id=t.slave_id,master_id=t.master_id,
                  readonly=True,enable_sent=False,parameter_writes_sent=False,flash_save_sent=False,
                  parameters={},errors={},tx_frames=[])
    sys.path.insert(0,str(find_driver_root()))
    from dm_canable2 import MotorControl
    class ReadOnlyControl(MotorControl):
        def send_std(self, arbitration_id, payload, *, pad_to_8=True):
            if not read_frame_allowed(t.slave_id,arbitration_id,payload):
                raise RuntimeError('non-read CAN frame blocked')
            report['tx_frames'].append(dict(can_id=arbitration_id,data=bytes(payload).hex()))
            return super().send_std(arbitration_id,payload,pad_to_8=pad_to_8)
    with output.open('x') as file:
        try:
            occupied = subprocess.run(['fuser',args.channel],capture_output=True,timeout=3)
            if occupied.returncode != 1:raise RuntimeError('serial device occupied or occupancy check failed')
            with ExitStack() as stack:
                device = stack.enter_context(open(args.channel,'rb',buffering=0))
                fcntl.flock(device.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
                control = ReadOnlyControl(channel=args.channel,auto_open=False)
                stack.callback(control.close);control.open()
                reader = IdProgrammer(control,timeout=.25,read_attempts=3)
                reader.read_pair(t.slave_id,t.master_id)
                report['state_before'] = reader.read_state(t.slave_id,t.master_id)
                for rid,name in REGISTERS.items():
                    try:
                        _,raw = reader.read_uint(t.slave_id,rid,t.master_id)
                        value = decode_parameter(rid,raw)
                        report['parameters'][name] = dict(rid=rid,value=value,raw_hex=f'{raw:08x}')
                        print(f'{name:12} RID={rid:2} value={value}',flush=True)
                    except (RuntimeError,ValueError) as error:
                        report['errors'][name] = str(error)
                        print(f'{name}: {error}',flush=True)
                report['state_after'] = reader.read_state(t.slave_id,t.master_id)
        except (Exception,KeyboardInterrupt) as error:
            report['errors']['session'] = str(error) or type(error).__name__
            print('Read stopped:',report['errors']['session'],flush=True)
        finally:
            report['complete'] = not report['errors']
            json.dump(report,file,indent=2);file.write('\n')
    print('Saved:',output,flush=True)
    return 0 if report['complete'] else 2


if __name__ == '__main__':raise SystemExit(main())
