"""Test per-motor VMAX decoding without accessing CAN."""
import math
from pathlib import Path
import sys
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'Motor Drive'))
from damiao_bus_scan import ScanTarget, apply_protocol_limits, load_targets
from dm_canable2 import Motor, DM_Motor_Type
from dm_canable2.core import parse_feedback


def test_measured_j3_vmax_does_not_change_same_type_j1_or_position(tmp_path):
    p=tmp_path/'motors.yaml'
    p.write_text('joints:\n  joint1: {motor_type: DM4340P, slave_id: 1, master_id: 17}\n'
                 '  joint3: {motor_type: DM4340P, slave_id: 3, master_id: 19, protocol_vmax: 20.0}\n')
    targets=load_targets(p)
    motors=[Motor(DM_Motor_Type.DM4340P,t.slave_id,t.master_id) for t in targets]
    for m,t in zip(motors,targets):apply_protocol_limits(m,t)
    before,after=[parse_feedback(bytes.fromhex('138000A008001E1D'),m.limit_param) for m in motors]
    assert math.isclose(after.dq,2*before.dq)
    assert after.q==before.q and after.tau==before.tau
    assert motors[0].limit_param==(12.5,10.,28.)
    assert motors[1].limit_param==(12.5,20.,28.)


@pytest.mark.parametrize('value',[0.,-1.,float('nan'),float('inf'),'bad'])
def test_invalid_protocol_range_is_rejected(value):
    with pytest.raises(ValueError):ScanTarget('joint3','DM4340P',3,19,value)
