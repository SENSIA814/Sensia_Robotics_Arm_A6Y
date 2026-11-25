#!/usr/bin/env python3
"""Recompute virtual states at recorded input/tick times, without ROS/network."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/a6_y_vr_teleop'))
from a6_y_vr_teleop.a5_kinematics import A5Kinematics
from a6_y_vr_teleop.virtual_arm import VirtualArm


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('record',type=Path)
    p.add_argument('--output',type=Path,default=ROOT/'validation/vr_rviz/replay_check.json')
    args=p.parse_args()
    arm=None;count=0;maximum=0.
    with args.record.open() as f:
        for line in f:
            row=json.loads(line)
            if row['kind']=='metadata':
                limits=row.get('limits',ROOT/'src/a6_y_controllers/config/a5_motion_limits.yaml')
                for name,path in [('urdf',row['urdf']),('limits',limits)]:
                    expected=row.get(name+'_sha256')
                    if expected and hashlib.sha256(Path(path).read_bytes()).hexdigest()!=expected:
                        p.error(name+' changed since recording; restore the recorded model/limits before replay')
                arm=VirtualArm(A5Kinematics(row['urdf'],limits,row['config']),row['config'])
            elif row['kind']=='input':
                assert arm is not None,'node metadata required'
                arm.receive(row['packet'].encode(),row['receive_ns'])
            elif row['kind']=='state':
                before=arm.q.copy()
                arm.tick(row['monotonic_ns'],row['dt'],enforce_budget=False)
                if row.get('reason') in ('compute_budget_exceeded','input_processing_budget_exceeded','input_queue_overload') and row['state']=='PAUSED':
                    arm.q=before;arm.pause(row['reason'])
                maximum=max(maximum,float(np.max(np.abs(arm.q-row['q']))));count+=1
    result=dict(passed=count>0 and maximum<1e-9,states=count,max_joint_difference=maximum,
                source=str(args.record),hardware_accessed=False,
                method='same recorded monotonic input/tick times; runtime budget failures replayed as recorded events')
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));sys.exit(0 if result['passed'] else 1)


if __name__=='__main__':main()
