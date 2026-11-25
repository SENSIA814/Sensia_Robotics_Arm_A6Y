#!/usr/bin/env python3
"""Validate the installed A5 model, seven-motor mapping and coupled gravity."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

root = Path(__file__).resolve().parents[1]
shares = {p: Path(get_package_share_directory(p)) for p in
          ('a5_description', 'a6_y_controllers', 'a6_y_ros2_control')}
for p in shares.values():
    assert p.is_relative_to(root / 'install'), f'External overlay leaked into test: {p}'
model_path = shares['a5_description'] / 'urdf/A5.urdf'
robot = ET.parse(model_path).getroot()
arm = [f'joint{i}' for i in range(1, 7)]
actuated = arm + ['gripper_finger_joint1']
assert len(robot.findall('link')) == 10
assert [j.get('name') for j in robot.findall('joint')] == actuated + ['gripper_finger_joint2', 'tool_tcp_joint']
tcp_joint = robot.find("joint[@name='tool_tcp_joint']")
assert tcp_joint.get('type') == 'fixed' and tcp_joint.find('parent').get('link') == 'link6'
assert tcp_joint.find('child').get('link') == 'tool_tcp'
assert len(robot.find("link[@name='tool_tcp']")) == 0, 'TCP must not add mass or geometry'
for name in arm:
    assert robot.find(f"joint[@name='{name}']").get('type') == 'revolute'
for name in ('gripper_finger_joint1', 'gripper_finger_joint2'):
    assert robot.find(f"joint[@name='{name}']").get('type') == 'prismatic'
mimic = robot.find("joint[@name='gripper_finger_joint2']/mimic")
assert mimic.attrib == {'joint': 'gripper_finger_joint1', 'multiplier': '-1', 'offset': '0'}
meshes = sorted({m.get('filename') for m in robot.findall('.//mesh')})
assert len(meshes) == 9
for uri in meshes:
    assert uri.startswith('package://a5_description/')
    assert (shares['a5_description'] / uri.removeprefix('package://a5_description/')).is_file(), uri
payload_mass = sum(float(robot.find(f"link[@name='{name}']/inertial/mass").get('value'))
                   for name in ('link6', 'gripper_finger_link1', 'gripper_finger_link2'))
assert payload_mass > 0.5
config_dir = shares['a6_y_controllers'] / 'config'
motors = yaml.safe_load((config_dir / 'a5_motors.yaml').read_text())['joints']
params = yaml.safe_load((config_dir / 'a5_gravity_controllers.yaml').read_text())[
    '/arm6/a6_y_gravity_comp_controller']['ros__parameters']
assert params['joints'] == list(motors) == actuated
for key in ('torque_scales', 'effort_signs', 'torque_limits', 'damping_gains',
            'friction_compensations', 'friction_velocity_deadbands',
            'friction_velocity_scales', 'haptic_effort_limits'):
    assert len(params[key]) == 7 and np.isfinite(params[key]).all()
assert not params['haptic_enable_output'] and not params['friction_pulse_enabled']
assert np.all(np.array(params['friction_compensations']) >= 0)
assert np.all(np.array(params['friction_compensations']) <= np.array(params['torque_limits']))
assert np.all(np.array(params['friction_velocity_deadbands']) > 0)
assert np.all(np.array(params['friction_velocity_scales']) > 0)
assert np.isfinite(params['friction_ramp_duration']) and params['friction_ramp_duration'] > 0
assert params['friction_compensations'][-1] == 0
assert params['torque_scales'][-1] == params['damping_gains'][-1] == 0
assert motors['joint3']['sign'] == 1 and motors['joint3']['min_position'] == 0
finger = motors['gripper_finger_joint1']
assert (finger['motor_type'], finger['slave_id'], finger['master_id']) == ('DM3507', 7, 23)
assert finger['sign'] == 1 and finger['position_scale'] == 0.0239
assert finger['min_position'] == -0.03 and finger['max_position'] == 0
# The documented old motor-angle mapping must preserve physical finger motion.
angle = -0.6
assert finger['sign'] * angle * finger['position_scale'] < 0
assert np.isclose(30 * finger['position_scale'], 0.717)
xml = xacro.process_file(str(config_dir / 'a5.urdf.xacro'), mappings={
    'model_file': str(model_path), 'motor_config_file': str(config_dir / 'a5_motors.yaml'),
    'dry_run': 'true', 'auto_enable': 'false'}).toxml()
hardware = ET.fromstring(xml).find('ros2_control')
assert [j.get('name') for j in hardware.findall('joint')] == actuated
assert sum(len(j.findall('command_interface')) for j in hardware.findall('joint')) == 28
for j in hardware.findall('joint'):
    actual = {p.get('name'): p.text for p in j.findall('param')}
    expected = motors[j.get('name')]
    for key in ('slave_id', 'master_id'):
        assert int(actual[key]) == expected[key]
    for key in ('sign', 'offset', 'min_position', 'max_position', 'max_velocity'):
        assert float(actual[key]) == expected[key]
    assert float(actual['position_scale']) == expected.get('position_scale', 1)

# Native mimic model has 7 independent coordinates. The independently parsed
# 8-DoF tree provides a reference with explicit q_finger2 = -q_finger1.
model = pin.buildModelFromUrdf(model_path, True)
full_model = pin.buildModelFromUrdf(model_path, False)
assert model.nq == model.nv == 7 and full_model.nq == full_model.nv == 8
rng = np.random.default_rng(20261006)
max_error = 0.0
max_mimic_error = 0.0
max_j6_gravity = 0.0
for _ in range(32):
    q = np.array([rng.uniform(m['min_position'], m['max_position']) for m in motors.values()])
    gravity = pin.computeGeneralizedGravity(model, model.createData(), q)
    full_q = np.r_[q, -q[-1]]
    full_g = pin.computeGeneralizedGravity(full_model, full_model.createData(), full_q)
    projected = np.r_[full_g[:6], full_g[6] - full_g[7]]
    max_mimic_error = max(max_mimic_error, float(np.max(np.abs(gravity - projected))))
    max_j6_gravity = max(max_j6_gravity, float(abs(gravity[5])))
    for i in range(7):
        step = np.zeros(7); step[i] = 1e-6
        plus = q + step; minus = q - step
        gradient = (pin.computePotentialEnergy(full_model, full_model.createData(), np.r_[plus, -plus[-1]])
                    - pin.computePotentialEnergy(full_model, full_model.createData(), np.r_[minus, -minus[-1]])) / 2e-6
        max_error = max(max_error, float(abs(gravity[i] - gradient)))
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    wrist = data.oMf[model.getFrameId('link6')]
    for name, position in (('gripper_finger_joint1', q[-1]), ('gripper_finger_joint2', -q[-1])):
        joint = robot.find(f"joint[@name='{name}']")
        origin = np.fromstring(joint.find('origin').get('xyz'), sep=' ')
        child = data.oMf[model.getFrameId(joint.find('child').get('link'))]
        actual = wrist.rotation.T @ (child.translation - wrist.translation)
        assert np.allclose(actual, origin + [0, position, 0], atol=1e-10)
assert max_error < 1e-5 and max_mimic_error < 1e-10
assert max_j6_gravity > 1e-4, 'J6 must include the real wrist/gripper payload'
g0 = pin.computeGeneralizedGravity(model, model.createData(), np.zeros(7))
result = {'passed': True, 'hardware_accessed': False, 'packages': list(shares),
          'links': 10, 'urdf_joints': 9, 'independent_joints': 7, 'mesh_files': len(meshes),
          'command_interfaces': 28, 'payload_mass_kg': payload_mass,
          'pinocchio_version': pin.__version__, 'gravity_at_model_zero': g0.tolist(),
          'torque_scales': params['torque_scales'], 'tested_poses': 32,
          'gravity_gradient_max_error': max_error, 'mimic_projection_max_error': max_mimic_error,
          'max_sampled_j6_gravity_nm': max_j6_gravity}
(root / 'validation').mkdir(exist_ok=True)
(root / 'validation/model_check.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
