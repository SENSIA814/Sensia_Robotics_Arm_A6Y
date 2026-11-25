"""Source URDF and explicit, serial-bound display conversion; no device I/O."""
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/arm_diagnostics'))
from scanner import load_roles, load_position_scales, ROLE_ORDER

MODEL = ROOT / 'src/A5/urdf/A5.urdf'
MAPPING = Path(__file__).with_name('joint_mapping.yaml')
JOINTS = tuple([f'joint{i}' for i in range(1, 7)] + ['gripper_finger_joint1'])


def load_model(role, roles=None, mapping_path=MAPPING, urdf_path=MODEL):
    roles = roles if roles is not None else load_roles()
    if role not in ROLE_ORDER or role not in roles:
        raise ValueError('请明确选择运动／示教端的一条左臂或右臂')
    data = yaml.safe_load(Path(mapping_path).read_text(encoding='utf-8'))
    if data.get('schema_version') != 1:
        raise ValueError('不支持的关节映射版本')
    binding = data['roles'][role]
    if str(binding['usb_serial']) != roles[role]['usb_serial']:
        raise ValueError('显示映射的 USB 身份与机械臂角色不符')
    mapping = deepcopy(data['defaults'])
    for name, override in binding['overrides'].items():
        if name not in mapping:
            raise ValueError(f'未知关节 {name}')
        mapping[name].update(override)
    if set(mapping) != set(JOINTS):
        raise ValueError('必须配置六个关节和一个独立夹爪关节')
    source = Path(urdf_path).read_bytes()
    tree = ET.fromstring(source)
    if tree.findall('ros2_control') or tree.findall('gazebo'):
        raise ValueError('显示入口只接受纯 URDF 模型')
    joints = {j.attrib['name']: j for j in tree.findall('joint')}
    independent = {name for name, j in joints.items()
                   if j.attrib['type'] != 'fixed' and j.find('mimic') is None}
    if independent != set(JOINTS):
        raise ValueError('URDF 独立关节与显示映射不一致')
    scales = load_position_scales().get(roles[role]['usb_serial'], {})
    ids = []
    for name in JOINTS:
        entry = mapping[name]
        motor_id = entry['motor_id']
        if type(motor_id) is not int or motor_id not in range(1, 8):
            raise ValueError('电机 ID 必须为 1～7')
        ids.append(motor_id)
        if motor_id not in scales:
            raise ValueError(f'缺少 {role} ID {motor_id} 的实测 PMAX')
        for key in ('sign', 'position_scale', 'offset'):
            value = entry[key]
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError(f'{name}: {key} 必须为有限数值')
        if entry['sign'] not in (-1, 1) or entry['position_scale'] <= 0:
            raise ValueError(f'{name}: sign 必须为 ±1，position_scale 必须为正数')
        joint = joints[name]
        entry.update(name=name, type=joint.attrib['type'],
                     axis=[float(x) for x in joint.find('axis').attrib['xyz'].split()],
                     child=joint.find('child').attrib['link'],
                     origin=dict(joint.find('origin').attrib),
                     lower=float(joint.find('limit').attrib['lower']),
                     upper=float(joint.find('limit').attrib['upper']),
                     position_scale_evidence=scales[motor_id])
    if len(set(ids)) != 7:
        raise ValueError('七个独立关节必须使用不同电机 ID')
    # Resolve meshes from this source tree, never a stale installed overlay.
    for mesh in tree.iter('mesh'):
        uri = mesh.attrib['filename']
        prefix = 'package://a5_description/'
        if not uri.startswith(prefix):
            raise ValueError(f'非预期模型资源：{uri}')
        path = (ROOT / 'src/A5' / uri[len(prefix):]).resolve(strict=True)
        if not path.is_relative_to((ROOT / 'src/A5').resolve()):
            raise ValueError('模型资源超出 A5 目录')
        mesh.set('filename', path.as_uri())
    return {'role': role, 'arm': deepcopy(roles[role]), 'joints': mapping,
            'urdf_path': str(urdf_path), 'urdf_sha256': hashlib.sha256(source).hexdigest(),
            'description': ET.tostring(tree, encoding='unicode'),
            'mapping_path': str(mapping_path), 'mapping_source': data['source'],
            'mapping_verified': False}


def convert_position(motor_rad, entry):
    # Deliberately no clamping, wraparound, smoothing, or initial-pose subtraction.
    if not math.isfinite(motor_rad):
        raise ValueError('非有限的电机反馈')
    return entry['sign'] * entry['position_scale'] * motor_rad + entry['offset']


def out_of_limits(q, entry):
    # Half-step position quantization around a calibrated zero is not a limit fault.
    tolerance = entry['position_scale_evidence']['pmax_rad'] * 2 / 65535 * entry['position_scale']
    return q < entry['lower'] - tolerance or q > entry['upper'] + tolerance
