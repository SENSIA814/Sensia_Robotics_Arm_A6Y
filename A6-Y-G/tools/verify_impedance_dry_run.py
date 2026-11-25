#!/usr/bin/env python3
"""Verify impedance plugin, launch, target capture and inherited gravity without accessing CAN.

This is interface validation, not a dynamic simulation of hand pushing.
"""
from pathlib import Path
import json
import math
import os
import signal
import subprocess
import time
import numpy as np
import pinocchio as pin
import yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState
from tf2_msgs.msg import TFMessage

root = Path(__file__).resolve().parents[1]
validation = root / 'validation'
validation.mkdir(exist_ok=True)
os.environ['ROS_DOMAIN_ID'] = os.environ.get('A6Y_TEST_ROS_DOMAIN_ID', '193')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = str(validation / 'ros_logs')
log_path = validation / 'impedance_dry_run_launch.log'
report = {'passed': False, 'hardware_accessed': False, 'dry_run': True, 'auto_enable': False}
node = None
process = None
try:
    rclpy.init()
    node = rclpy.create_node('a5_impedance_verifier')
    samples = []
    diagnostic = []
    transforms = {}
    def accept_transforms(message):
        for transform in message.transforms:
            transforms[transform.child_frame_id] = transform
    node.create_subscription(JointState, '/arm6/joint_states', samples.append, 10)
    node.create_subscription(DynamicJointState, '/arm6/dynamic_joint_states', diagnostic.append, 10)
    node.create_subscription(TFMessage, '/tf', accept_transforms, 10)
    node.create_subscription(TFMessage, '/tf_static', accept_transforms,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = node.create_client(ListControllers, '/arm6/controller_manager/list_controllers')
    param_client = node.create_client(GetParameters, '/arm6/a6_y_cartesian_impedance_controller/get_parameters')
    with log_path.open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'a6_y_controllers', 'a5_cartesian_impedance.launch.py',
                                    'dry_run:=true', 'auto_enable:=false', 'can_channel:=/dev/a6y_no_device'], stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, cwd=root)
        deadline = time.monotonic() + 30
        active = {}
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f'Launch exited early: {process.returncode}; see {log_path}')
            rclpy.spin_once(node, timeout_sec=0.1)
            if client.wait_for_service(timeout_sec=0.1):
                future = client.call_async(ListControllers.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=1)
                if future.done() and future.result() is not None:
                    active = {c.name: c for c in future.result().controller}
            if (all(name in active and active[name].state == 'active' for name in
                    ['joint_state_broadcaster', 'a6_y_cartesian_impedance_controller']) and samples and diagnostic
                    and {'gripper_finger_link1', 'gripper_finger_link2', 'tool_tcp'} <= transforms.keys()
                    and len(samples[-1].effort) == 7 and max(abs(v) for v in samples[-1].effort) > 0):
                break
        else:
            raise RuntimeError('Controllers or arm and gripper state topics did not become ready')
        expected = [f'joint{i}' for i in range(1, 7)] + ['gripper_finger_joint1']
        state = samples[-1]
        assert len(state.name) == 7 and set(state.name) == set(expected), state.name
        assert len(state.position) == 7 and all(math.isfinite(q) for q in state.position)
        positions = dict(zip(state.name, state.position))
        controller = active['a6_y_cartesian_impedance_controller']
        assert len(controller.claimed_interfaces) == 28
        enabled = {}
        for name, values in zip(diagnostic[-1].joint_names, diagnostic[-1].interface_values):
            mapping = dict(zip(values.interface_names, values.values))
            enabled[name] = mapping['enabled']
            assert mapping['position_valid'] == 1 and mapping['position_age'] == 0
            assert mapping['position_rejections'] == 0 and mapping['dry_run'] == 1
        assert set(enabled) == set(expected) and all(v == 0 for v in enabled.values()), enabled
        assert param_client.wait_for_service(timeout_sec=3)
        request = GetParameters.Request()
        request.names = ['urdf_path', 'torque_scales', 'damping_gains', 'torque_limits',
                         'cartesian_stiffness', 'cartesian_damping', 'tcp_frame']
        future = param_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        response = future.result()
        assert response is not None
        urdf = response.values[0].string_value
        assert Path(urdf).is_relative_to(root / 'install')
        assert urdf.endswith('A5.urdf')
        assert len(response.values[1].double_array_value) == 7
        assert response.values[1].double_array_value[-1] == 0.0
        assert response.values[2].double_array_value[-1] == 0.0
        assert len(state.velocity) == len(state.effort) == 7
        ordered = [state.name.index(name) for name in expected]
        q = np.array([state.position[i] for i in ordered])
        dq = np.array([state.velocity[i] for i in ordered])
        efforts = np.array([state.effort[i] for i in ordered])
        model = pin.buildModelFromUrdf(urdf, True)
        tcp = transforms['tool_tcp']
        assert tcp.header.frame_id == 'link6'
        origin = model.frames[model.getFrameId('tool_tcp')].placement
        assert np.allclose([tcp.transform.translation.x, tcp.transform.translation.y,
                            tcp.transform.translation.z], origin.translation, atol=1e-10)
        assert np.allclose([tcp.transform.rotation.x, tcp.transform.rotation.y,
                            tcp.transform.rotation.z, tcp.transform.rotation.w], [0, 0, 0, 1])
        gravity = pin.computeGeneralizedGravity(model, model.createData(), q)
        scales = np.array(response.values[1].double_array_value)
        damping = np.array(response.values[2].double_array_value)
        limits = np.array(response.values[3].double_array_value)
        gravity_source = yaml.safe_load((root / 'src/a6_y_controllers/config/a5_gravity_controllers.yaml').read_text())[
            '/arm6/a6_y_gravity_comp_controller']['ros__parameters']
        assert list(scales) == gravity_source['torque_scales']
        assert list(damping) == gravity_source['damping_gains']
        assert list(limits) == gravity_source['torque_limits']
        assert response.values[6].string_value == 'tool_tcp'
        impedance_source = yaml.safe_load((root / 'src/a6_y_controllers/config/a5_impedance_controllers.yaml').read_text())[
            '/arm6/a6_y_cartesian_impedance_controller']['ros__parameters']
        assert list(response.values[4].double_array_value) == impedance_source['cartesian_stiffness']
        assert list(response.values[5].double_array_value) == impedance_source['cartesian_damping']
        assert np.all(np.abs(dq) < 1e-10)
        # Dry-run returns positions unchanged: capture has no spring error.
        assert 'Captured fixed TCP target' in log_path.read_text()
        assert 'Implicit TCP impedance' in log_path.read_text()
        assert 'limit=none:none' in log_path.read_text()
        assert 'task_Fz_scaled_N=' in log_path.read_text()
        expected_efforts = np.clip(scales * gravity - damping * dq, -limits, limits)
        assert np.allclose(efforts, expected_efforts, atol=1e-8), (efforts, expected_efforts)
        assert efforts[-1] == 0.0
        report.update({'passed': True, 'controllers': {k: v.state for k, v in active.items()},
                       'joint_names': state.name, 'positions_by_joint': positions, 'claimed_interfaces': list(controller.claimed_interfaces),
                       'enabled': enabled, 'loaded_urdf_relative': str(Path(urdf).relative_to(root)),
                       'torque_scales': list(response.values[1].double_array_value),
                       'cartesian_stiffness': list(response.values[4].double_array_value),
                       'cartesian_damping': list(response.values[5].double_array_value),
                       'target_captured': True,
                       'implementation': 'implicit_step',
                       'efforts_by_joint': dict(zip(expected, efforts.tolist())),
                       'tf_child_frames': sorted(transforms)})
        # Spawner services report active just before their own processes exit.
        finish_until = time.monotonic() + 1.0
        while time.monotonic() < finish_until:
            rclpy.spin_once(node, timeout_sec=0.1)
finally:
    if process is not None and process.poll() is None:
        # Let launch forward SIGINT once to its children; signalling the whole
        # group here would deliver it twice and interrupt shutdown handlers.
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
                report['passed'] = False
        report['launch_exit_code'] = process.returncode
        report['passed'] = report['passed'] and process.returncode == 0
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    (validation / 'impedance_dry_run_check.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
assert report['passed']
