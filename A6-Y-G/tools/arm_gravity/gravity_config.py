"""Four-arm adapters for the existing A5 Pinocchio C++ gravity controller."""
from copy import deepcopy
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET
from zoneinfo import ZoneInfo

import numpy as np
import pinocchio as pin
import yaml

ROOT = Path(__file__).resolve().parents[2]
for sub in ('arm_diagnostics', 'arm_visualization', 'arm_moveit'):
    sys.path.insert(0, str(ROOT/'tools'/sub))
from model import JOINTS, load_model, load_roles, ROLE_ORDER
from motion_model import MOCK_Q

SOURCE = ROOT/'src/a6_y_controllers/config/a5_gravity_controllers.yaml'
PROFILE_DIR = Path(__file__).with_name('profiles')
CONTROLLER = 'a6_y_gravity_comp_controller'
VECTOR_KEYS = ('torque_scales', 'effort_signs', 'damping_gains', 'torque_limits', 'friction_compensations')
LIVE_KEYS = ('torque_scales', 'damping_gains', 'friction_compensations', 'friction_compensation_enabled')
# Protocol motor torque ceilings. These are not tuning defaults.
CAPS = [28., 28., 28., 10., 10., 10., 0.]
MOTOR_LIMITS_SOURCE = Path(__file__).with_name('motor_limits.json')


def base_parameters():
    p = deepcopy(yaml.safe_load(SOURCE.read_text())['/arm6/'+CONTROLLER]['ros__parameters'])
    # Gripper feedback contributes payload geometry, but the gripper is read-only.
    p['torque_limits'][6] = 0.
    return p


def sag_trial_parameters():
    """Uncalibrated small-step starting point; never saved or applied implicitly."""
    p = base_parameters()
    p['torque_scales'][1:4] = [1.2, 1.1, 1.2]
    p['friction_compensation_enabled'] = False
    return p


def validate_parameters(p):
    if p.get('joints') != list(JOINTS):
        raise ValueError('关节顺序必须与当前七轴模型一致')
    for key in VECTOR_KEYS:
        values = p.get(key)
        if not isinstance(values, list) or len(values) != 7 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
            raise ValueError(key+' 必须包含七个有限数值')
        if key == 'effort_signs':
            if any(v not in (-1., 1.) for v in values): raise ValueError('补偿方向只能为 +1 或 -1')
        elif any(v < 0 for v in values):
            raise ValueError(key+' 不能为负数')
    if any(x > 3 for x in p['torque_scales']) or any(x > 5 for x in p['damping_gains']):
        raise ValueError('补偿系数范围 0～3；阻尼范围 0～5')
    if any(v > cap for v,cap in zip(p['torque_limits'], CAPS)):
        raise ValueError('力矩上限超过电机协议范围')
    if any(v > cap for v,cap in zip(p['friction_compensations'], p['torque_limits'])):
        raise ValueError('摩擦补偿不能超过总力矩限幅')
    if any(p[k][6] != 0 for k in VECTOR_KEYS if k != 'effort_signs'):
        raise ValueError('夹爪保持只读，补偿、阻尼和力矩限幅必须为零')
    if type(p.get('friction_compensation_enabled')) is not bool:
        raise ValueError('摩擦补偿开关无效')
    return deepcopy(p)


def load_profile(role, roles=None, directory=PROFILE_DIR):
    roles = roles or load_roles()
    if role not in ROLE_ORDER: raise ValueError('未知机械臂')
    p = base_parameters()
    path = Path(directory)/(role+'.yaml')
    if path.exists():
        data = yaml.safe_load(path.read_text())
        if data.get('schema_version') != 1 or data.get('role') != role or str(data.get('usb_serial')) != roles[role]['usb_serial']:
            raise ValueError('重力参数的机械臂与 USB 身份不符')
        saved = data['parameters']
        for key in (*VECTOR_KEYS, 'friction_compensation_enabled'):
            p[key] = saved[key]
    return validate_parameters(p)


def save_profile(role, p, roles=None, directory=PROFILE_DIR):
    p = validate_parameters(p)
    roles = roles or load_roles()
    if role not in ROLE_ORDER: raise ValueError('未知机械臂')
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory/(role+'.yaml')
    data = dict(schema_version=1, role=role, usb_serial=roles[role]['usb_serial'],
                updated_at=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
                inherited_source=str(SOURCE), source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                parameters={key:p[key] for key in (*VECTOR_KEYS, 'friction_compensation_enabled')})
    with tempfile.NamedTemporaryFile('w', dir=directory, delete=False) as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        temp = Path(f.name)
    os.replace(temp, path)
    return path


def motor_limit_report(model):
    """One verified register source for decoding and feedback validity guards.

    VMAX is a protocol mapping range, not MAX_SPD or the home trajectory speed.
    Keep raw MAX_SPD/ACC/DEC for diagnosis; do not infer their units or write them.
    """
    data = json.loads(MOTOR_LIMITS_SOURCE.read_text())
    if data.get('schema_version') != 1 or data.get('readonly') is not True or data.get('motor_writes_sent') != 0:
        raise ValueError('电机参数记录的格式或只读来源无效')
    adapters = [a for a in data['arms'].values() if a['usb_serial'] == model['arm']['usb_serial']]
    if len(adapters) != 1: raise ValueError('缺少当前 USB 身份对应的电机参数记录')
    result = {}
    for motor in adapters[0]['motors']:
        i = motor['motor_id']; p = motor['parameters']
        if i not in range(1,8) or i in result or p['ESC_ID'] != i or p['MST_ID'] != i+16:
            raise ValueError('协议量程记录的电机身份不符')
        if any(not math.isfinite(p[k]) or p[k] <= 0 for k in ('PMAX','VMAX','TMAX')):
            raise ValueError('协议量程记录无效')
        if p['TMAX'] != ([28.]*3+[10.]*3+[5.])[i-1]:
            raise ValueError('记录中的 TMAX 与驱动电机类型不符')
        result[i] = deepcopy(p)
    if set(result) != set(range(1,8)): raise ValueError('七台电机的协议量程记录不完整')
    limits=[]
    for name in JOINTS[:6]:
        entry=model['joints'][name]
        scale=abs(float(entry['position_scale']))
        if not math.isfinite(scale) or scale <= 0: raise ValueError('关节速度换算比例无效')
        limits.append(result[entry['motor_id']]['VMAX']*scale)
    return dict(source=str(MOTOR_LIMITS_SOURCE),source_sha256=hashlib.sha256(MOTOR_LIMITS_SOURCE.read_bytes()).hexdigest(),
                observed_at=data['observed_at'],usb_serial=model['arm']['usb_serial'],
                policy='VMAX × abs(position_scale), feedback validity only',
                feedback_velocity_limits=limits,motors=result)


def protocol_ranges(model):
    return {i:{k:float(p[k]) for k in ('PMAX','VMAX','TMAX')}
            for i,p in motor_limit_report(model)['motors'].items()}


def control_description(model, mock, output, initial=None, disarm_only=False, position_output=False):
    robot = ET.fromstring(model['description'])
    system = ET.SubElement(robot, 'ros2_control', name='A6YGravitySystem', type='system')
    hardware = ET.SubElement(system, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'a6_y_ros2_control/A6YDamiaoSystem'
    def param(parent, key, value):
        ET.SubElement(parent, 'param', name=key).text = str(value).lower() if type(value) is bool else str(value)
    hp = dict(dry_run=mock, auto_enable=output and not disarm_only, can_channel=model['arm']['port'],
              can_bitrate=1000000, tty_baudrate=115200, control_mode='pos_vel' if position_output else 'mit',
              mit_kp=0., mit_kd=0., defer_enable_until_controller_ready=True,
              set_control_mode_on_configure=False, require_feedback_on_configure=True,
              allow_partial_feedback=False, disable_on_deactivate=True,
              command_timeout=.2, feedback_timeout=.5, receive_timeout=.001,
              max_start_delta=.1, status_request_period=.05, initial_feedback_attempts=60)
    for k,v in hp.items(): param(hardware,k,v)
    motors = yaml.safe_load((ROOT/'src/a6_y_controllers/config/a5_motors.yaml').read_text())['joints']
    ranges = protocol_ranges(model)
    initial = MOCK_Q if initial is None else initial
    for i,name in enumerate(JOINTS):
        entry = model['joints'][name]
        joint = ET.SubElement(system, 'joint', name=name)
        for interface in ('position','velocity','effort','motor_enable'):
            ET.SubElement(joint,'command_interface', name=interface)
        for interface in ('position','velocity','effort','motor_error','fault','has_feedback','feedback_age',
                          'enabled','position_age','position_valid','position_rejections','dry_run',
                          'bus_cycle_duration','commanded_effort'):
            state = ET.SubElement(joint,'state_interface', name=interface)
            if interface == 'position': param(state,'initial_value',float(initial[i]))
        params = deepcopy(motors[name])
        params.update(slave_id=entry['motor_id'], master_id=entry['motor_id']+16,
                      sign=entry['sign'], position_scale=entry['position_scale'], offset=entry['offset'],
                      min_position=entry['lower'], max_position=entry['upper'],
                      protocol_pmax=ranges[entry['motor_id']]['PMAX'],
                      protocol_vmax=ranges[entry['motor_id']]['VMAX'],
                      # Preserve measured coordinates, with no old low-pass lag.
                      feedback_position_jump=0., feedback_position_alpha=1.,
                      read_only=not output or i == 6)
        if position_output and i < 6:
            params['max_velocity'] = .1
        for k,v in params.items(): param(joint,k,v)
    return ET.tostring(robot, encoding='unicode')


def write_config(model, p, directory, namespace, manager, mock, output, initial=None, disarm_only=False):
    p = validate_parameters(p)
    path = Path(directory)/('control_'+manager+'.yaml')
    p.update(urdf_path=model['urdf_path'], enable_output=True, live_tuning_enabled=True,
             planned_home_enabled=True, home_mock_follow=mock,
             feedback_velocity_limits=motor_limit_report(model)['feedback_velocity_limits'],
             hardware_enable_interface='motor_enable', haptic_enable_output=False,
             haptic_effort_topic=namespace+'/unused_haptic_effort')
    data = {namespace+'/'+manager: {'ros__parameters': {
        'update_rate':125, 'robot_description':control_description(model,mock,output,initial,disarm_only,
            position_output=p.get('teaching_position_output',False)),
        'joint_state_broadcaster': {'type':'joint_state_broadcaster/JointStateBroadcaster'},
        CONTROLLER:{'type':'a6_y_controllers/A6YGravityCompController'}}},
        namespace+'/'+CONTROLLER:{'ros__parameters':p}}
    path.write_text(yaml.safe_dump(data,sort_keys=False))
    return path


class GravityModel:
    def __init__(self, model):
        self.model = pin.buildModelFromXML(model['description'], True)
        if self.model.nq != 7 or self.model.nv != 7: raise ValueError('需要 Pinocchio 4 的七轴 mimic 模型')
        self.data = self.model.createData()
        self.qids = [self.model.idx_qs[self.model.getJointId(n)] for n in JOINTS]
        self.vids = [self.model.idx_vs[self.model.getJointId(n)] for n in JOINTS]
    def gravity(self, q):
        q = np.asarray(q,float)
        if q.shape != (7,) or not np.isfinite(q).all(): raise ValueError('关节反馈无效')
        full = np.zeros(7)
        full[self.qids] = q
        return pin.computeGeneralizedGravity(self.model,self.data,full)[self.vids].copy()
