"""One-shot register READ only; reject every non-read CAN frame."""
from pathlib import Path
import sys,json,datetime
import yaml
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'A6-Y-G/src/a6_y_ros2_control/tools'))
from read_motor_params import locked_control,find_driver_root
sys.path.insert(0,str(find_driver_root()))
from dm_canable2 import MotorControl,Motor,DM_Motor_Type,DM_variable
class Reader(MotorControl):
    def send_std(self,arbitration_id,payload,**kwargs):
        assert arbitration_id==0x7ff and len(payload)==8 and payload[2]==0x33, 'Refuse non-read frame'
        record['reads'].append(dict(id=arbitration_id,data=bytes(payload).hex()))
        return super().send_std(arbitration_id,payload,**kwargs)
roles=yaml.safe_load((root/'diagnostics/arm_roles.yaml').read_text())['roles']
record=dict(observed_at=datetime.datetime.now().astimezone().isoformat(),readonly=True,motor_writes_sent=0,arms={},reads=[])
output=Path(__file__).with_name('motor_registers.json')
for role,arm in roles.items():
    row=dict(usb_serial=arm['usb_serial'],port=arm['by_id'],motors=[]);record['arms'][role]=row
    with locked_control(Reader,channel=arm['by_id'],can_bitrate=1000000,tty_baudrate=115200) as bus:
        for i in range(1,8):
            m=Motor(DM_Motor_Type.DM4340P if i<=3 else DM_Motor_Type.DM4310,i,i+16);bus.addMotor(m)
            p={k:bus.read_motor_param(m,int(DM_variable[k])) for k in ('ESC_ID','MST_ID','CTRL_MODE','PMAX','VMAX','TMAX','MAX_SPD','ACC','DEC')}
            row['motors'].append(dict(motor_id=i,parameters=p))
            print(role,i,p,flush=True)
            output.write_text(json.dumps(record,ensure_ascii=False,indent=2))
