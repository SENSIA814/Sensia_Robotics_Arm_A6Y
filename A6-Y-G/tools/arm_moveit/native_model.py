"""Model/identity adapters around the user's unmodified MoveIt controller config."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from motion_model import ROOT, JOINTS, MOCK_Q
from model import load_model
from gripper_policy import load_limits

UPSTREAM = Path(__file__).with_name('ros2_stack') / 'src'
CONFIG = UPSTREAM / 'A6_Y_Moveitconfig/config'
COLLISION_POLICY = Path(__file__).with_name('collision_policy.yaml')


def collision_profiles():
    config = yaml.safe_load(COLLISION_POLICY.read_text())
    if config.get('schema_version') != 1 or config.get('default_profile') not in config.get('profiles', {}):
        raise ValueError('碰撞配置格式无效')
    return config


def resolved_collision_policy(role, collision_profile=None):
    profiles = collision_profiles()
    selected = collision_profile or profiles['default_profile']
    if selected not in profiles['profiles']:
        raise ValueError('未知碰撞配置：' + str(selected))
    policy = deepcopy(profiles['profiles'][selected])
    extra = policy.pop('role_disabled_pairs', {}).get(role, [])
    policy['disabled_pairs'] += extra
    if extra:
        policy['description'] = ('忽略 ' + '、'.join('↔'.join(pair) for pair in policy['disabled_pairs']) +
                                 ' 整对自碰撞；其他自碰撞和环境碰撞仍检查')
    policy.update(profile=selected, source=str(COLLISION_POLICY), role=role, role_extra_pairs=extra)
    return policy


def native_model(role, roles=None, collision_profile=None):
    model = load_model(role, roles)
    model['gripper_limits'] = load_limits(role, model['joints'][JOINTS[6]])
    limits = yaml.safe_load((CONFIG / 'joint_limits.yaml').read_text())
    robot = ET.fromstring(model['description'])
    for joint in robot.findall('joint'):
        if joint.get('name') in limits['joint_limits']:
            joint.find('limit').set('velocity', str(limits['joint_limits'][joint.get('name')]['max_velocity']))
    semantic = ET.parse(CONFIG / 'y1_6.srdf').getroot()
    semantic.set('name', robot.get('name'))
    # Keep the archive intact. Adapt names/poses, then apply the separately
    # recorded, user-requested bare-arm self-collision policy to this SRDF only.
    for element in semantic.iter():
        for key, value in list(element.attrib.items()):
            if key in ('base_link', 'tip_link', 'parent_link', 'child_link', 'link1', 'link2'):
                element.set(key, 'tool_tcp' if value == 'tcp_link' else value.rstrip('.'))
    for state in semantic.findall('group_state'):
        if state.get('name') == 'ready':
            semantic.remove(state)  # This pose belonged to the other URDF axes.
        elif state.get('group') == 'gripper':
            endpoint = 'open_position_m' if state.get('name') == 'open' else 'closed_position_m'
            state.find('joint').set('value', str(model['gripper_limits'][endpoint]))
    policy = resolved_collision_policy(role, collision_profile)
    links = {link.get('name') for link in robot.findall('link')}
    existing = {frozenset((p.get('link1'), p.get('link2'))) for p in semantic.findall('disable_collisions')}
    for pair in policy['disabled_pairs']:
        if not isinstance(pair, list) or len(pair) != 2 or pair[0] == pair[1] or not set(pair) <= links:
            raise ValueError('裸机碰撞配置包含无效连杆对')
        if frozenset(pair) not in existing:
            ET.SubElement(semantic, 'disable_collisions', link1=pair[0], link2=pair[1],
                          reason='UserConfirmedDecorativeShellsAbsent')
            existing.add(frozenset(pair))
    model.update(description=ET.tostring(robot, encoding='unicode'),
                 semantic=ET.tostring(semantic, encoding='unicode'), native_limits=limits,
                 collision_policy=policy)
    return model


def control_description(model, mock, enabled, initial=None):
    robot = ET.fromstring(model['description'])
    system = ET.SubElement(robot, 'ros2_control', name='A6YArmSystem', type='system')
    hardware = ET.SubElement(system, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'a6_y_ros2_control/A6YDamiaoSystem'
    original = yaml.safe_load((CONFIG / 'a6_y_damiao_motors.yaml').read_text())['a6_y_damiao_system']['ros__parameters']
    params = deepcopy(original['defaults'])
    params.update(dry_run=mock, auto_enable=enabled, can_channel=model['arm']['port'],
                  can_bitrate=original['can']['can_bitrate'], tty_baudrate=original['can']['tty_baudrate'],
                  # Exactly the real_moveit_bringup launch overrides in the ZIP.
                  command_timeout=0., max_start_delta=.1, feedback_timeout=0.,
                  control_mode='pos_vel', set_control_mode_on_configure=False)
    def param(parent, name, value):
        ET.SubElement(parent, 'param', name=name).text = str(value).lower() if isinstance(value, bool) else str(value)
    for name, value in params.items():
        param(hardware, name, value)
    current_motors = yaml.safe_load((ROOT / 'src/a6_y_controllers/config/a5_motors.yaml').read_text())['joints']
    initial = MOCK_Q if initial is None else initial
    for index, name in enumerate(JOINTS):
        entry = model['joints'][name]
        joint = ET.SubElement(system, 'joint', name=name)
        for interface in ('position', 'velocity', 'effort'):
            ET.SubElement(joint, 'command_interface', name=interface)
        for interface in ('position', 'velocity', 'effort', 'motor_error', 'fault', 'has_feedback', 'feedback_age', 'enabled'):
            state = ET.SubElement(joint, 'state_interface', name=interface)
            if interface == 'position':
                param(state, 'initial_value', float(initial[index]))
        joint_params = dict(current_motors[name])
        joint_params.update(slave_id=entry['motor_id'], master_id=entry['motor_id'] + 16,
                            sign=entry['sign'], offset=entry['offset'], position_scale=entry['position_scale'],
                            min_position=entry['lower'], max_position=entry['upper'],
                            protocol_pmax=entry['position_scale_evidence']['pmax_rad'], read_only=not enabled)
        for key, value in joint_params.items():
            param(joint, key, value)
    return ET.tostring(robot, encoding='unicode')


def write_moveit_config(model, folder):
    from moveit_configs_utils import MoveItConfigsBuilder
    folder = Path(folder)
    urdf, srdf = folder / 'robot.urdf', folder / 'robot.srdf'
    urdf.write_text(model['description'])
    srdf.write_text(model['semantic'])
    builder = (MoveItConfigsBuilder('y1_6', package_name='A6_Y_Moveitconfig')
               .robot_description(file_path=str(urdf))
               .robot_description_semantic(file_path=str(srdf))
               .robot_description_kinematics(file_path=str(CONFIG / 'kinematics.yaml'))
               .joint_limits(file_path=str(CONFIG / 'joint_limits.yaml'))
               .trajectory_execution(file_path=str(CONFIG / 'moveit_controllers.yaml'))
               .planning_pipelines(pipelines=['ompl']))
    params = builder.to_moveit_configs().to_dict()
    params.update(allow_trajectory_execution=True, publish_robot_description=True,
                  publish_robot_description_semantic=True)
    path = folder / 'move_group.yaml'
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    return path, params


def write_controller_config(model, folder, namespace, mock, enabled, initial=None, manager_name='controller_manager'):
    original = yaml.safe_load((CONFIG / 'ros2_controllers.yaml').read_text())
    manager = deepcopy(original['controller_manager']['ros__parameters'])
    manager['robot_description'] = control_description(model, mock, enabled, initial)
    result = {namespace + '/' + manager_name: {'ros__parameters': manager}}
    for name in ('arm_controller', 'gripper_controller'):
        result[namespace + '/' + name] = deepcopy(original[name])
    result[namespace + '/joint_state_broadcaster'] = {'ros__parameters': {'use_local_topics': False}}
    path = Path(folder) / ('controllers_active.yaml' if enabled else 'controllers_readonly.yaml')
    path.write_text(yaml.safe_dump(result))
    return path
