"""One source of model, mapping, kinematics and MoveIt planning limits."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/arm_visualization'))
from model import JOINTS, load_model, load_roles, ROLE_ORDER

ARM_JOINTS = JOINTS[:6]
GRIPPER_JOINTS = JOINTS[6:]
MAX_SPEED = .25  # joint rad/s, even at the UI's 100% setting
MAX_ACCEL = .5  # joint rad/s^2
GRIPPER_SPEED = .003  # single finger m/s
GRIPPER_ACCEL = .006
MOCK_Q = [0., 1.2, 1.2, 0., 0., 0., -.01]


def joint_speed(name):
    return MAX_SPEED if name in ARM_JOINTS else GRIPPER_SPEED


def joint_accel(name):
    return MAX_ACCEL if name in ARM_JOINTS else GRIPPER_ACCEL


def joint_tolerance(name, arm=.001, gripper=.00005):
    return arm if name in ARM_JOINTS else gripper


def bounded_start(model, measured):
    """Normalize only the planning copy; preserve/display raw encoder feedback."""
    q = np.asarray(measured, float).copy()
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError('起点反馈无效')
    warnings = []
    for i, name in enumerate(JOINTS):
        e = model['joints'][name]
        bounded = np.clip(q[i], e['lower'], e['upper'])
        residual = q[i] - bounded
        if abs(residual) > joint_tolerance(name, .001, .0005):
            raise ValueError(f'{name} 反馈超出模型范围 {residual:.6f}，请返回零位标定检查')
        if abs(residual) > 1e-9:
            unit, factor = ('rad', 1) if i < 6 else ('mm', 1000)
            digits = 6 if i < 6 else 3
            warnings.append(f'{name} 边界残差 {residual * factor:+.{digits}f} {unit}；原始反馈保留，建议复核零位')
            q[i] = bounded
    return q, '；'.join(warnings)


def planning_model(role, roles=None):
    model = load_model(role, roles)
    tree = ET.fromstring(model['description'])
    semantic = ET.Element('robot', name=tree.attrib['name'])
    group = ET.SubElement(semantic, 'group', name='arm')
    ET.SubElement(group, 'chain', base_link='base_link', tip_link='tool_tcp')
    group = ET.SubElement(semantic, 'group', name='gripper')
    for name in GRIPPER_JOINTS:
        ET.SubElement(group, 'joint', name=name)
        entry = model['joints'][name]
        span = entry['position_scale'] * entry['position_scale_evidence']['pmax_rad']
        entry['lower'] = max(entry['lower'], entry['offset'] - span)
        entry['upper'] = min(entry['upper'], entry['offset'] + span)
        if entry['lower'] >= entry['upper']:
            raise ValueError('夹爪电机映射与 URDF 范围没有交集')
    for joint in tree.findall('joint'):
        parent, child = joint.find('parent').attrib['link'], joint.find('child').attrib['link']
        ET.SubElement(semantic, 'disable_collisions', link1=parent, link2=child, reason='Adjacent')
        limit = joint.find('limit')
        if limit is not None:
            # CAD exports velocity=0. Set explicit planning values in this
            # session copy; leave geometry, position limits and source untouched.
            limit.set('velocity', str(MAX_SPEED if joint.attrib['name'] in ARM_JOINTS else .003))
            if joint.attrib['name'] in GRIPPER_JOINTS:
                entry = model['joints'][joint.attrib['name']]
                limit.set('lower', str(entry['lower']))
                limit.set('upper', str(entry['upper']))
    ET.SubElement(semantic, 'disable_collisions', link1='gripper_finger_link1',
                  link2='gripper_finger_link2', reason='Finger contact')
    model['description'] = ET.tostring(tree, encoding='unicode')
    model['semantic'] = ET.tostring(semantic, encoding='unicode')
    return model


def moveit_parameters(model):
    limits = {name: {'has_velocity_limits': True, 'max_velocity': joint_speed(name),
                     'has_acceleration_limits': True, 'max_acceleration': joint_accel(name)}
              for name in JOINTS}
    return {'robot_description': model['description'],
            'robot_description_semantic': model['semantic'],
            'robot_description_planning': {'joint_limits': limits},
            'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
            'ompl': {'planning_plugin': 'ompl_interface/OMPLPlanner',
                     'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                         'default_planner_request_adapters/FixWorkspaceBounds '
                                         'default_planner_request_adapters/FixStartStateBounds',
                     'start_state_max_bounds_error': .001,
                     'min_angle_change': .000001, 'path_tolerance': .00005, 'resample_dt': .05,
                     'planner_configs': {'RRTConnect': {'type': 'geometric::RRTConnect', 'range': .0}},
                     'arm': {'planner_configs': ['RRTConnect'], 'longest_valid_segment_fraction': .002},
                     'gripper': {'planner_configs': ['RRTConnect'], 'longest_valid_segment_fraction': .002}},
            'allow_trajectory_execution': False,
            'publish_robot_description': True, 'publish_robot_description_semantic': True,
            'planning_scene_monitor': {'publish_planning_scene': True,
                                       'publish_geometry_updates': True,
                                       'publish_state_updates': True, 'publish_transforms_updates': True}}


class Kinematics:
    """Pinocchio FK/Jacobian with bounded numerical pose IK for the six arm axes."""
    def __init__(self, model):
        self.model = pin.buildModelFromXML(model['description'], True)
        if self.model.nq != 7 or self.model.nv != 7:
            raise ValueError('需要 Pinocchio 4 的原生 mimic 支持及七个独立关节')
        self.data = self.model.createData()
        self.frame = self.model.getFrameId('tool_tcp')
        self.indices = [self.model.idx_qs[self.model.getJointId(name)] for name in JOINTS]
        self.lower = np.array([model['joints'][n]['lower'] for n in ARM_JOINTS])
        self.upper = np.array([model['joints'][n]['upper'] for n in ARM_JOINTS])

    def fk(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError('FK 需要七个有限关节位置')
        full = np.zeros(7)
        full[self.indices] = q
        pin.framesForwardKinematics(self.model, self.data, full)
        return self.data.oMf[self.frame].homogeneous.copy()

    def ik(self, target, seed, position_only=False):
        target, seed = np.asarray(target, float), np.asarray(seed, float)
        if target.shape != (4, 4) or not np.isfinite(target).all() or seed.shape != (7,):
            raise ValueError('末端目标或当前关节状态无效')
        if not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-5) or np.linalg.det(target[:3, :3]) < .999:
            raise ValueError('目标旋转矩阵无效')
        if np.linalg.norm(target[:3, 3]) > 1.2:
            raise ValueError('目标超出机械臂工作空间')
        reference = np.clip(seed[:6], self.lower, self.upper)
        def pose_error(q):
            pose = self.fk(np.r_[q, seed[6]])
            xyz = pose[:3, 3] - target[:3, 3]
            return xyz if position_only else np.r_[xyz, .2 * pin.log3(target[:3, :3] @ pose[:3, :3].T)]
        def error(q):
            # Small regularization selects a nearby solution for redundant position
            # goals. Starting inside the bounds avoids stagnation at a zero stop.
            return np.r_[pose_error(q), 1e-5 * (q - reference)]
        seeds = [np.clip(reference, self.lower + .001, self.upper - .001)]
        for shift in (.3, -.3, .8):
            seeds.append(np.clip(reference + [0., shift, shift, -shift, 0., 0.], self.lower + .001, self.upper - .001))
        for initial in seeds:
            result = least_squares(error, initial, bounds=(self.lower, self.upper), max_nfev=160,
                                   ftol=1e-9, xtol=1e-9, gtol=1e-10)
            e = pose_error(result.x)
            if np.linalg.norm(e[:3]) <= .0005 and (position_only or np.linalg.norm(e[3:]) / .2 <= .005):
                return np.r_[result.x, seed[6]]
        raise ValueError('目标不可达：请缩小拖动距离，或勾选“仅位置（姿态自由）”后重试')
