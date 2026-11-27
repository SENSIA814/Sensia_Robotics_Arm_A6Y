"""Archive teleop configuration adapted only for this rig's roles and six axes."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml

from gravity_config import ROOT, JOINTS, CONTROLLER, write_config, load_profile, base_parameters, protocol_ranges

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
POSITION_LIMIT_POLICY = 'scanned_motor_PMAX_in_joint_coordinates'


def position_bounds(model):
    """Power teaching uses the scanned motor ranges requested by the operator.

    Match native to_motor_position exactly; URDF limits remain model/planning
    metadata. Never move or re-zero an encoder to fit that model at startup.
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
    # Diagnostic threshold only; never changes watchdogs or motor control.
    ET.SubElement(hardware, 'param', name='diagnostic_update_rate').text = str(CONTROL_HZ)
    ET.SubElement(hardware, 'param', name='controller_mode_switching').text = 'true'
    bounds = position_bounds(model)
    for joint in robot.findall('ros2_control/joint'):
        low,high = bounds[joint.get('name')]
        joint.find("param[@name='min_position']").text = str(low)
        joint.find("param[@name='max_position']").text = str(high)
        for name in TIMING_INTERFACES:
            ET.SubElement(joint, 'state_interface', name=name)
    for joint, speed in zip(robot.findall('ros2_control/joint')[:6], VELOCITIES):
        joint.find("param[@name='max_velocity']").text = str(speed)
    cm['robot_description'] = ET.tostring(robot, encoding='unicode')
    cm[HOME] = {'type':'joint_trajectory_controller/JointTrajectoryController'}
    cm[FOLLOW] = {'type':'position_controllers/JointGroupPositionController'}
    data[namespace+'/'+HOME] = {'ros__parameters':{
        'joints':list(JOINTS[:6]), 'command_interfaces':['position'],
        'state_interfaces':['position','velocity'],
        # Left wrist VMAX=50 encodes zero velocity as +/-0.01221 rad/s.
        'constraints':{'stopped_velocity_tolerance':.03, 'goal_time':5.0}}}
    data[namespace+'/'+FOLLOW] = {'ros__parameters':{'joints':list(JOINTS[:6])}}
    if teacher:
        gravity = data[namespace+'/'+CONTROLLER]['ros__parameters']
        gravity.update(planned_home_enabled=False, home_mock_follow=False,
                       teaching_follow_enabled=False, teaching_position_output=False)
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
    config['joints'] = {k:v[:6] for k,v in config['joints'].items()}
    bounds = position_bounds(follower.model)
    config['limits'].update(
        step_reference='previous_command',
        min_position=[bounds[n][0] for n in JOINTS[:6]],
        max_position=[bounds[n][1] for n in JOINTS[:6]],
        limit_margin=[0.]*6)
    config['smoothing']['soft_start_duration'] = STARTUP_BLEND_SECONDS
    for section in ('limits','prediction'):
        for key,value in config[section].items():
            if isinstance(value,list):config[section][key] = value[:6]
    config['recording']['action_space'] = 'joint_position_6d'
    path = Path(folder)/(follower.model['role']+'_teleop.yaml')
    path.write_text(yaml.safe_dump(config,sort_keys=False))
    return path, config
