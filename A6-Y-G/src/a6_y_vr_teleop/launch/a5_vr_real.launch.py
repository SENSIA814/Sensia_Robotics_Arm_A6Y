"""A5 external-reference controller. Default physical mode is readonly."""
import os
import fcntl
import hashlib
import json
import subprocess
import time
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from a6_y_vr_teleop.a5_kinematics import A5Kinematics, JOINTS
from a6_y_vr_teleop.commissioning import (profile_paths, commissioning_issues, reference_limit_issues,
                                        commissioning_profile, commissioning_stage_issues, sha256)
from a6_y_vr_teleop.position_profile import position_paths, position_profile, position_issues


def setup(context):
    backend = LaunchConfiguration('control_backend').perform(context)
    if backend not in ('impedance', 'position'): raise ValueError('control_backend must be impedance/position')
    position = backend == 'position'
    motion_range=LaunchConfiguration('motion_range').perform(context)
    if motion_range not in ('trial','full') or (motion_range=='full' and not position):
        raise ValueError('motion_range=full is only supported by position control')
    controller_name = 'a6_y_vr_position_controller' if position else 'a6_y_joint_impedance_controller'
    controller_key = '/arm6/'+controller_name
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('readonly', 'mock', 'commission', 'execute'): raise ValueError('mode must be readonly/mock/commission/execute')
    mock_pose_arg=LaunchConfiguration('mock_initial_positions').perform(context)
    if mock_pose_arg and mode!='mock': raise ValueError('mock_initial_positions is only allowed in isolated mock mode')
    stage = LaunchConfiguration('stage').perform(context)
    if mode == 'commission' and stage == 'none': stage = 'hold'
    if stage not in ('none', 'hold', 'follow') or (stage != 'none' and mode not in ('commission', 'mock')):
        raise ValueError('stage=hold/follow is only valid for commission or isolated mock')
    trial_arg = LaunchConfiguration('operator_trial').perform(context).lower()
    if trial_arg not in ('true', 'false'): raise ValueError('operator_trial must be true/false')
    operator_trial = trial_arg == 'true'
    if operator_trial and (mode != 'commission' or stage != 'follow'):
        raise ValueError('operator_trial is only available in mode:=commission stage:=follow')
    zero_arg = LaunchConfiguration('zero_collision_tolerance').perform(context).lower()
    if zero_arg not in ('true', 'false'): raise ValueError('zero_collision_tolerance must be true/false')
    zero_tolerance = zero_arg == 'true'
    if zero_tolerance and not (operator_trial or (mode == 'mock' and stage == 'follow')):
        raise ValueError('zero_collision_tolerance requires restricted operator trial or isolated mock follow')
    if position and (mode == 'execute' or (mode == 'commission' and stage == 'follow' and not operator_trial)):
        raise ValueError('Position backend currently supports readonly, mock and restricted commission; follow requires operator_trial:=true')
    domain = os.environ.get('ROS_DOMAIN_ID')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not domain or domain == '84':
        raise RuntimeError('set explicit ROS_DOMAIN_ID (not virtual domain 84) and ROS_LOCALHOST_ONLY=1')
    if mode == 'mock' and domain != '86': raise RuntimeError('mock requires isolated ROS_DOMAIN_ID=86')
    # Also guard direct ros2 launch usage, not just the shell wrapper.
    import rclpy
    check_context = rclpy.Context(); rclpy.init(args=[], context=check_context)
    check_node = rclpy.create_node('vr_launch_graph_check', context=check_context)
    check_executor = rclpy.executors.SingleThreadedExecutor(context=check_context)
    check_executor.add_node(check_node)
    try:
        until = time.monotonic()+1.2
        while time.monotonic() < until:
            check_executor.spin_once(timeout_sec=.1)
            if ('controller_manager', '/arm6') in check_node.get_node_names_and_namespaces():
                raise RuntimeError('Existing /arm6/controller_manager; stop the old launch first')
    finally:
        check_executor.shutdown(); check_node.destroy_node(); rclpy.shutdown(context=check_context)
    paths, manifest = profile_paths()
    if position: paths = position_paths(paths,motion_range)
    share = Path(get_package_share_directory('a6_y_vr_teleop'))
    cfg = yaml.safe_load(paths['bridge'].read_text())
    if cfg.get('zero_pose_collision_tolerance', {}).get('enabled', False):
        raise ValueError('activate zero-pose collision tolerance through the restricted launch argument only')
    kin = A5Kinematics(paths['urdf'], paths['limits'], cfg)
    controllers = yaml.safe_load(paths['controller'].read_text())
    params = controllers[controller_key]['ros__parameters']
    limit_issues = reference_limit_issues(params, kin)
    if limit_issues: raise RuntimeError('\n'.join(limit_issues))
    manifest_data = yaml.safe_load(manifest.read_text())
    issues = position_issues(paths, kin, params, manifest_data,motion_range) if position else commissioning_issues(manifest_data, paths, kin, cfg)
    if position and issues: raise RuntimeError('Position configuration:\n'+'\n'.join(issues))
    if mode == 'execute' and issues: raise RuntimeError('Physical execute is not commissioned:\n'+'\n'.join(issues))
    if mode == 'commission' and not position:
        stage_issues = commissioning_stage_issues(stage, manifest_data, paths, kin, cfg,
                                                 operator_trial=operator_trial)
        if stage_issues: raise RuntimeError('Commissioning '+stage+' prerequisites:\n'+'\n'.join(stage_issues))
    params.update(urdf_path=str(paths['urdf']), vr_allow_prepare=mode != 'readonly', vr_allow_arm=mode != 'readonly')
    if position:
        params, cfg = position_profile(stage, params, cfg,motion_range)
        params['vr_allow_arm'] = mode != 'readonly' and stage != 'hold'
    elif stage != 'none':
        params, cfg = commissioning_profile(stage, params, cfg)
    controllers[controller_key]['ros__parameters'] = params
    if zero_tolerance: cfg['zero_pose_collision_tolerance']['enabled'] = True
    params['vr_allow_test_input'] = mode == 'mock'
    record = LaunchConfiguration('record_file').perform(context)
    if stage != 'none' and not record:
        record = str(Path.cwd()/'validation/vr_commission'/f'{time.time_ns()}_{mode}_{stage}.jsonl')
    if record:
        record = str(Path(record).resolve())
        if Path(record).exists(): raise RuntimeError('record_file already exists: '+record)
        Path(record).parent.mkdir(parents=True, exist_ok=True)
    metadata = dict(launch_mode=mode, commissioning_stage=stage, controller_parameters=params,
                    control_backend=backend, controller_name=controller_name,
                    motion_range=motion_range,
                    operator_trial=operator_trial,
                    zero_collision_tolerance=zero_tolerance,
                    unverified_prerequisites=(['position HOLD/follow/stop physical validation pending',
                        'Quest axis mapping and workspace calibration pending',
                        'native motor current limits not verified by effort-controller torque caps'] if position else
                        commissioning_stage_issues(stage, manifest_data, paths, kin, cfg) if operator_trial else []),
                    sha256={key: sha256(path) for key, path in paths.items()})
    print(f'Commissioning stage={stage}; ARM allowed={params["vr_allow_arm"]}; record={record}', flush=True)
    print(f'A5 VR {mode}: commissioning gaps={len(issues)}; starts DISABLED; initial PREPARE/ARM are manual', flush=True)
    channel = LaunchConfiguration('can_channel').perform(context)
    serial_lock = None
    if mode == 'mock': channel = '/dev/THIS_IS_A_MOCK_NO_CAN'
    else:
        real_channel = str(Path(channel).resolve())
        lock_name = hashlib.sha256(real_channel.encode()).hexdigest()[:16]
        serial_lock = open('/tmp/a5-vr-serial-'+lock_name+'.lock', 'w')
        fcntl.flock(serial_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        occupied = subprocess.run(['fuser', real_channel], capture_output=True, timeout=3)
        if occupied.returncode == 0:
            serial_lock.close()
            raise RuntimeError('Serial device is already open: '+real_channel)
    description = xacro.process_file(str(paths['hardware']), mappings=dict(
        model_file=str(paths['urdf']), motor_config_file=str(paths['motors']),
        dry_run='true' if mode == 'mock' else 'false', auto_enable='false' if mode == 'readonly' else 'true',
        can_channel=channel, can_bitrate='1000000', tty_baudrate='115200',
        command_timeout='0.05', feedback_timeout='0.5')).toxml()
    if position:
        root = ET.fromstring(description)
        root.find("ros2_control/hardware/param[@name='control_mode']").text = 'pos_vel'
        description = ET.tostring(root, encoding='unicode')
    if mode == 'mock':
        root = ET.fromstring(description)
        # Mock initial state only. Physical launch never sends or assumes this pose.
        mock_pose=yaml.safe_load(mock_pose_arg) if mock_pose_arg else [0., 1., 1., 0., .4, 0., -.015]
        if not isinstance(mock_pose,list) or not kin.feedback_in_bounds(mock_pose):
            raise ValueError('mock_initial_positions requires seven finite in-bounds joint coordinates')
        for name, value in zip(JOINTS, mock_pose):
            root.find(f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param").text = str(value)
        description = ET.tostring(root, encoding='unicode')
    with tempfile.NamedTemporaryFile(mode='w', prefix='a5_vr_', suffix='.yaml', delete=False) as out:
        yaml.safe_dump(controllers, out); pinned = Path(out.name)
    with tempfile.NamedTemporaryFile(mode='w', prefix='a5_vr_bridge_', suffix='.yaml', delete=False) as out:
        yaml.safe_dump(cfg, out); bridge_config = Path(out.name)
    def cleanup(_event, _context):
        pinned.unlink(missing_ok=True)
        bridge_config.unlink(missing_ok=True)
        if serial_lock is not None: serial_lock.close()
        return []
    def failed(event, _context):
        return [EmitEvent(event=Shutdown(reason='VR controller spawner failed'))] if event.returncode else []
    manager = Node(package='controller_manager', executable='ros2_control_node', namespace='arm6',
        parameters=[dict(robot_description=ParameterValue(description, value_type=str)), str(pinned)], output='screen',
        on_exit=EmitEvent(event=Shutdown(reason='A5 controller manager exited')))
    spawners = [Node(package='controller_manager', executable='spawner', namespace='arm6', output='screen',
        arguments=[name, '--controller-manager', '/arm6/controller_manager', '--param-file', str(pinned)])
        for name in ('joint_state_broadcaster', controller_name)]
    # In physical HOLD the bridge is preview-only; the C++ gate independently rejects ARM.
    bridge_mode = 'mock' if mode == 'mock' else ('readonly' if mode == 'readonly' or stage == 'hold' else 'execute')
    return [RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        *[RegisterEventHandler(OnProcessExit(target_action=s, on_exit=failed)) for s in spawners],
        manager, *spawners,
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='arm6',
             parameters=[dict(robot_description=paths['urdf'].read_text(), publish_frequency=125.)]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='arm6/vr', name='reference_model',
             parameters=[dict(robot_description=paths['urdf'].read_text(), frame_prefix='vr_ref/', publish_frequency=125.)],
             remappings=[('joint_states', '/arm6/vr/reference_joint_states')]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='vr_reference_base',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'vr_ref/base_link']),
        Node(package='a6_y_vr_teleop', executable='real_bridge', namespace='arm6/vr', output='screen',
             parameters=[dict(mode=bridge_mode, udp_port=15154 if mode == 'mock' else 15153,
                 controller_prefix=controller_key,
                 config_path=str(bridge_config), urdf_path=str(paths['urdf']), limits_path=str(paths['limits']),
                 record_file=record, run_metadata=json.dumps(metadata))]),
        Node(package='rviz2', executable='rviz2', namespace='arm6', name='vr_rviz',
             arguments=['-d', str(share/'config/a5_vr_real.rviz')], condition=IfCondition(LaunchConfiguration('use_rviz')))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='readonly'),
        DeclareLaunchArgument('control_backend', default_value='impedance'),
        DeclareLaunchArgument('motion_range', default_value='trial'),
        DeclareLaunchArgument('stage', default_value='none'),
        DeclareLaunchArgument('operator_trial', default_value='false'),
        DeclareLaunchArgument('zero_collision_tolerance', default_value='false'),
        DeclareLaunchArgument('can_channel', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('mock_initial_positions', default_value=''),
        DeclareLaunchArgument('record_file', default_value=''), OpaqueFunction(function=setup)])
