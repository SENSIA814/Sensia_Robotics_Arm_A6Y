"""Power teaching: six original arm axes plus role-aware gripper following."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml
from dataclasses import asdict
from a6_y_teleop.gripper_haptics import GripperHapticConfig

from gravity_config import ROOT, JOINTS, CONTROLLER, write_config, load_profile, base_parameters, protocol_ranges
from teaching_gripper import (GRIPPER_JOINT, controlled_joints, gripper_profile, gripper_mapping,
                              TEACHING_GRIPPER_STATUS_PERIOD_S)

HOME = 'arm_controller'
FOLLOW = 'direct_position_controller'
CONTROL_HZ = 250
TARGET_HZ = 100
STARTUP_BLEND_SECONDS = 1.2
CAN_BITRATE = 1000000
TIMING_INTERFACES = ('control_time', 'control_cycle_count', 'control_period',
                     'control_overrun_count', 'read_duration', 'write_duration',
                     'command_count', 'feedback_count', 'sent_position', 'sent_motor_position',
                     'sent_velocity_limit', 'send_begin_time', 'send_complete_time',
                     'feedback_receive_time', 'sent_mode', 'motor_position', 'motor_velocity')
VELOCITIES = [7.5, 7.5, 7.5, 5.0, 5.0, 5.0]
HOME_TOLERANCE = .03  # move_to_group_state.py's original goal tolerance
HOME_TIMEOUT = 55.  # Includes the original client's discovery and result waits.
POSITION_LIMIT_POLICY = 'protocol_feedback_bounds_and_measured_inset_gripper_targets'


def position_bounds(model):
    """Decoded feedback/startup/hold use motor protocol coordinate bounds.

    Match native to_motor_position exactly. Gripper calibration is used for
    percentage goals only; crossing a recorded endpoint is not a feedback fault.
    PMAX is an encoding boundary, not a mechanical travel or percentage goal.
    Never move or re-zero an encoder to fit that model at startup.
    """
    ranges = protocol_ranges(model)
    bounds = {}
    for name in JOINTS:
        entry = model['joints'][name]
        scale,offset,sign = (float(entry[k]) for k in ('position_scale','offset','sign'))
        if not all(math.isfinite(v) for v in (scale,offset,sign)) or scale <= 0 or sign not in (-1.,1.):
            raise ValueError(name+' 电机坐标换算无效')
        radius = ranges[entry['motor_id']]['PMAX'] * scale
        bounds[name] = (offset-radius,offset+radius)
    # PMAX is an encoder codec range, not a usable gripper opening interval.
    gripper = gripper_profile(model)
    bounds[GRIPPER_JOINT] = (gripper['hardware_min_position_m'], gripper['hardware_max_position_m'])
    return bounds


def port_parameters(role, roles=None):
    # Motion arms have no gravity profile dependency in the teaching workflow.
    return load_profile(role, roles) if role.startswith('teaching_') else base_parameters()


def write_port_control(model, parameters, folder, namespace, manager, mock, output,
                       initial=None, disarm_only=False):
    path = write_config(model, parameters, folder, namespace, manager, mock, output, initial, disarm_only)
    data = yaml.safe_load(path.read_text())
    cm = data[namespace+'/'+manager]['ros__parameters']
    cm['update_rate'] = CONTROL_HZ
    robot = ET.fromstring(cm['robot_description'])
    hardware = robot.find('ros2_control/hardware')
    teacher = model['role'].startswith('teaching_')
    hardware.find("param[@name='control_mode']").text = 'mit' if teacher else 'pos_vel'
    hardware.find("param[@name='can_bitrate']").text = str(CAN_BITRATE)
    if teacher:
        # Read-only connection stage polls at 10 ms. Once active, J7 returns
        # feedback with its zero-stiffness MIT command at the native 250 Hz rate.
        hardware.find("param[@name='status_request_period']").text = str(TEACHING_GRIPPER_STATUS_PERIOD_S)
    # Diagnostic threshold only; never changes watchdogs or motor control.
    ET.SubElement(hardware, 'param', name='diagnostic_update_rate').text = str(CONTROL_HZ)
    ET.SubElement(hardware, 'param', name='controller_mode_switching').text = 'true'
    bounds = position_bounds(model)
    gripper = gripper_profile(model)
    # Teaching's generated URDF must admit the same decoded feedback as the
    # driver, including holds outside the calibration and inset command interval.
    # Only limit metadata changes; coordinates, geometry and six-axis tuning do not.
    low, high = bounds[GRIPPER_JOINT]
    robot.find(f"joint[@name='{GRIPPER_JOINT}']/limit").attrib.update(lower=str(low), upper=str(high))
    for joint in robot.findall('joint'):
        mimic = joint.find('mimic')
        if mimic is not None and mimic.get('joint') == GRIPPER_JOINT:
            factor, offset = float(mimic.get('multiplier', '1')), float(mimic.get('offset', '0'))
            ends = [factor*q+offset for q in (low, high)]
            joint.find('limit').attrib.update(lower=str(min(ends)), upper=str(max(ends)))
    for joint in robot.findall('ros2_control/joint'):
        low,high = bounds[joint.get('name')]
        joint.find("param[@name='min_position']").text = str(low)
        joint.find("param[@name='max_position']").text = str(high)
        if joint.get('name') == GRIPPER_JOINT:
            joint.find("param[@name='read_only']").text = str(not output).lower()
            joint.find("param[@name='max_velocity']").text = str(gripper['max_velocity_m_s'])
            if teacher:
                ET.SubElement(joint, 'param', name='mit_kp').text = '0.0'
                ET.SubElement(joint, 'param', name='mit_kd').text = '0.0'
        for name in TIMING_INTERFACES:
            ET.SubElement(joint, 'state_interface', name=name)
    for joint, speed in zip(robot.findall('ros2_control/joint')[:6], VELOCITIES):
        joint.find("param[@name='max_velocity']").text = str(speed)
    cm['robot_description'] = ET.tostring(robot, encoding='unicode')
    cm[HOME] = {'type':'joint_trajectory_controller/JointTrajectoryController'}
    cm[FOLLOW] = {'type':'position_controllers/JointGroupPositionController'}
    data[namespace+'/'+HOME] = {'ros__parameters':{
        'joints':list(controlled_joints(model)), 'command_interfaces':['position'],
        'state_interfaces':['position','velocity'],
        # Left wrist VMAX=50 encodes zero velocity as +/-0.01221 rad/s.
        'constraints':{'stopped_velocity_tolerance':.03, 'goal_time':5.0}}}
    data[namespace+'/'+HOME]['ros__parameters']['constraints'][GRIPPER_JOINT] = {
        'trajectory': .001, 'goal': .0005}
    data[namespace+'/'+FOLLOW] = {'ros__parameters':{'joints':list(controlled_joints(model))}}
    if teacher:
        gravity = data[namespace+'/'+CONTROLLER]['ros__parameters']
        gravity.update(planned_home_enabled=False, home_mock_follow=False,
                       teaching_follow_enabled=False, teaching_position_output=False)
        haptic = GripperHapticConfig()
        # The MIT codec truncates to a 12-bit integer. Negative values may be
        # rounded away from zero by one LSB; reserve that step in the native cap.
        torque_lsb = 2*protocol_ranges(model)[7]['TMAX']/4095
        limit = (haptic.max_teacher_torque-torque_lsb) / model['joints'][GRIPPER_JOINT]['position_scale']
        gravity['torque_limits'][6] = limit
        gravity.update(haptic_enable_output=True,
            haptic_effort_topic=namespace+'/gripper_haptics/effort',
            haptic_output_enable_topic=namespace+'/gripper_haptics/enable',
            haptic_effort_limits=[0.]*6+[limit],
            haptic_command_timeout=haptic.timeout, haptic_output_enable_timeout=haptic.timeout,
            haptic_activation_ramp_duration=.1)
    else:
        del cm[CONTROLLER]
        del data[namespace+'/'+CONTROLLER]
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return path


def write_mapper_config(folder, leader, follower, frequency=TARGET_HZ):
    if frequency != TARGET_HZ:
        raise ValueError(f'当前动力示教目标发送频率固定为 {TARGET_HZ} Hz')
    source = ROOT/'src/a6_y_teleop/config/teleop.yaml'
    config = yaml.safe_load(source.read_text())
    ns = follower.namespace+'/upstream_teleop'
    config['teleop'].update(control_hz=float(frequency),
        leader_joint_state_topic=leader.namespace+'/joint_states',
        follower_joint_state_topic=follower.namespace+'/joint_states',
        command_topic=follower.namespace+'/'+FOLLOW+'/commands',
        **{key:ns+'/'+topic for key,topic in (
            ('enable_topic','enable'),('emergency_stop_topic','emergency_stop'),
            ('raw_action_topic','raw_action'),('safe_action_topic','safe_action'),('status_topic','status'))})
    mapping = gripper_mapping(leader.model, follower.model)
    config['joints'] = {k:v[:6] for k,v in config['joints'].items()}
    for key, value in dict(names=GRIPPER_JOINT, leader_names=GRIPPER_JOINT,
                           leader_joint_index=6, sign=mapping['sign'],
                           scale=mapping['scale'], offset=mapping['offset']).items():
        config['joints'][key].append(value)
    bounds = position_bounds(follower.model)
    config['limits'].update(
        step_reference='previous_command',
        slew_into_bounds_joints=[GRIPPER_JOINT],
        min_position=[bounds[n][0] for n in JOINTS[:6]] + [mapping['follower']['open_position_m']],
        max_position=[bounds[n][1] for n in JOINTS[:6]] + [mapping['follower']['closed_position_m']],
        limit_margin=[0.]*7)
    config['smoothing']['soft_start_duration'] = STARTUP_BLEND_SECONDS
    for section in ('limits','prediction'):
        for key,value in config[section].items():
            if isinstance(value,list) and key not in ('min_position', 'max_position', 'limit_margin', 'slew_into_bounds_joints'):
                config[section][key] = value[:6] + [mapping['follower']['max_velocity_m_s'] if key == 'max_velocity' else 0.]
    config['recording']['action_space'] = 'joint_position_6rad_gripper_1m'
    config['recording']['gripper_mapping'] = mapping
    config['gripper_haptics'] = dict(
        parameters=asdict(GripperHapticConfig(
            leader_scale=leader.model['joints'][GRIPPER_JOINT]['position_scale'],
            follower_scale=follower.model['joints'][GRIPPER_JOINT]['position_scale'],
            follower_open=mapping['follower']['open_position_m'],
            follower_closed=mapping['follower']['closed_position_m'])),
        effort_topic=leader.namespace+'/gripper_haptics/effort',
        gate_topic=leader.namespace+'/gripper_haptics/enable',
        status_topic=ns+'/gripper_haptics',
        leader_feedback_topic=leader.namespace+'/dynamic_joint_states',
        follower_feedback_topic=follower.namespace+'/dynamic_joint_states')
    path = Path(folder)/(follower.model['role']+'_teleop.yaml')
    path.write_text(yaml.safe_dump(config,sort_keys=False))
    return path, config
