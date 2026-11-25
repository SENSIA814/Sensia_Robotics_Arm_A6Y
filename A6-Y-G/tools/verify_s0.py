#!/usr/bin/env python3
"""Check the S0 model contract and measured calibration records; no hardware IO."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import yaml

from calibration import field_issues, field_summary, positive

from derive_a5_tcp import DESCRIPTION, ROOT, URDF, check_tcp, derive, rotation, vector

CONFIG = ROOT / 'src/a6_y_controllers/config'


def load(path):
    return yaml.safe_load(path.read_text())




def xml_fk(robot, q_by_name):
    """Independent URDF chain multiplication, including fixed TCP and mimic."""
    transforms = {'base_link': np.eye(4)}
    remaining = list(robot.findall('joint'))
    while remaining:
        count = len(remaining)
        for joint in remaining[:]:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            origin = joint.find('origin')
            T = np.eye(4)
            T[:3, :3] = rotation(vector(origin, 'rpy', '0 0 0'))
            T[:3, 3] = vector(origin, 'xyz', '0 0 0')
            motion = np.eye(4)
            kind = joint.get('type')
            if kind != 'fixed':
                q = q_by_name.get(joint.get('name'), 0)
                mimic = joint.find('mimic')
                if mimic is not None:
                    q = q_by_name[mimic.get('joint')] * float(mimic.get('multiplier', '1')) + float(mimic.get('offset', '0'))
                axis = vector(joint.find('axis'), 'xyz', '1 0 0')
                if kind == 'prismatic':
                    motion[:3, 3] = axis * q
                elif kind == 'revolute':
                    x, y, z = axis
                    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                    motion[:3, :3] = np.eye(3) + np.sin(q) * cross + (1-np.cos(q)) * cross @ cross
                else:
                    raise ValueError(f'Unsupported S0 joint type: {kind}')
            transforms[joint.find('child').get('link')] = transforms[parent] @ T @ motion
            remaining.remove(joint)
        if len(remaining) == count:
            raise ValueError('Disconnected or cyclic URDF')
    return transforms




def verify(calibration_path):
    robot = ET.parse(URDF).getroot()
    conventions = load(DESCRIPTION / 'config/a5_model_conventions.yaml')
    motors = load(CONFIG / 'a5_motors.yaml')['joints']
    motion = load(CONFIG / 'a5_motion_limits.yaml')
    params = load(CONFIG / 'a5_gravity_controllers.yaml')['/arm6/a6_y_gravity_comp_controller']['ros__parameters']
    calibration = load(calibration_path)
    assert all(config['schema_version'] == 1 for config in (conventions, motion, calibration))
    names = conventions['independent_joints']
    assert names == list(motors) == list(motion['joint_limits']) == params['joints']
    assert list(calibration['joint_mapping']) == names
    assert conventions['base_frame'] == 'base_link' and conventions['tcp_frame'] == 'tool_tcp'
    assert conventions['wrist_frame'] == 'link6' and conventions['tcp_joint'] == 'tool_tcp_joint'
    assert conventions['model'] == 'urdf/A5.urdf' and conventions['tcp_axes'] == 'inherit_link6'
    assert conventions['units'] == {'length': 'm', 'angle': 'rad', 'arm_effort': 'Nm', 'gripper_effort': 'N'}
    mimic = conventions['mimic']
    actual_mimic = robot.find(f"joint[@name='{mimic['joint']}']/mimic")
    assert actual_mimic.get('joint') == mimic['source']
    assert float(actual_mimic.get('multiplier')) == mimic['multiplier'] == -1
    assert float(actual_mimic.get('offset')) == mimic['offset'] == 0
    gripper = conventions['gripper']; motor = motors[gripper['independent_coordinate']]
    assert gripper['open_fraction_zero_q_m'] == motor['max_position'] == 0
    assert gripper['open_fraction_one_q_m'] == motor['min_position'] == -0.03
    assert gripper['gap_change_per_abs_joint_displacement'] == 2
    tcp = derive(); check_tcp(robot, tcp)
    assert conventions['tcp_definition'] == tcp['method']
    model = pin.buildModelFromUrdf(URDF, True)
    assert model.nq == model.nv == 7
    assert np.allclose(model.gravity.linear, conventions['model_gravity_in_base_m_s2'])
    for i, name in enumerate(names):
        joint = robot.find(f"joint[@name='{name}']")
        cad = joint.find('limit'); runtime = motion['joint_limits'][name]; motor = motors[name]
        assert model.idx_qs[model.getJointId(name)] == i
        assert motor['sign'] in (-1, 1) and positive(motor.get('position_scale', 1))
        assert np.isfinite(motor['offset'])
        for key in ('min_position', 'max_position', 'max_velocity'):
            assert runtime[key] == motor[key], f'{name}: motion/driver mismatch: {key}'
        assert float(cad.get('lower')) <= runtime['min_position'] < runtime['max_position'] <= float(cad.get('upper'))
        assert positive(runtime['max_velocity']) and positive(runtime['max_effort'])
        assert runtime['max_effort'] == params['torque_limits'][i], f'{name}: effort cap mismatch'
        assert name in motion['cad_effort_differences']
    max_fk_error = 0.0
    rng = np.random.default_rng(20261006)
    for _ in range(32):
        q = np.array([rng.uniform(m['min_position'], m['max_position']) for m in motors.values()])
        reference = xml_fk(robot, dict(zip(names, q)))['tool_tcp']
        data = model.createData(); pin.framesForwardKinematics(model, data, q)
        actual = data.oMf[model.getFrameId('tool_tcp')].homogeneous
        max_fk_error = max(max_fk_error, float(np.max(np.abs(reference-actual))))
        for opening in (0.0, -0.015, -0.03):
            q[-1] = opening
            pin.framesForwardKinematics(model, data, q)
            assert np.allclose(reference, data.oMf[model.getFrameId('tool_tcp')].homogeneous, atol=1e-10)
    assert max_fk_error < 1e-10
    pending = field_issues(calibration, motors, model, names)
    evidence = field_summary(calibration)
    return {'software_passed': True, 'hardware_accessed': False,
            # Complete means the required checks are resolved under the recorded
            # user decisions; it does not imply all checks were measured.
            'field_calibration_complete': not pending, 'pending_field_checks': pending,
            'field_measurements_complete': not pending and not evidence['operator_confirmed'] and not evidence['skipped_by_user'],
            'calibration_evidence': evidence,
            'tcp_xyz_in_link6_m': tcp['xyz_m'], 'tcp_rpy_in_link6_rad': tcp['rpy_rad'],
            'opening_center_max_drift_m': tcp['opening_center_max_drift_m'],
            'independent_dof': model.nq, 'fk_tested_poses': 32, 'xml_pinocchio_fk_max_error': max_fk_error}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, default=CONFIG / 'a5_calibration.yaml')
    parser.add_argument('--require-field-calibration', action='store_true')
    args = parser.parse_args()
    out = ROOT / 'validation/s0_check.json'; out.parent.mkdir(exist_ok=True)
    try:
        result = verify(args.calibration)
    except Exception as error:
        out.write_text(json.dumps({'software_passed': False, 'hardware_accessed': False,
                                   'error': f'{type(error).__name__}: {error}'}, indent=2) + '\n')
        raise
    out.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if args.require_field_calibration and not result['field_calibration_complete']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
