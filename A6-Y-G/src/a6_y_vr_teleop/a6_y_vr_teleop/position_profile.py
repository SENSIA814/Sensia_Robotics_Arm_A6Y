"""Native-position trial configuration, independent of effort commissioning."""
from copy import deepcopy
import numpy as np
import yaml
from .a5_kinematics import JOINTS
from .commissioning import reference_limit_issues
from .field_calibration import field_issues

CONTROLLER = 'a6_y_vr_position_controller'


def position_paths(paths, motion_range='trial'):
    if motion_range not in ('trial','full'): raise ValueError('invalid motion_range')
    result = dict(paths)
    root = paths['controller'].parent
    suffix='_full' if motion_range=='full' else ''
    result.update(controller=root/f'a5_vr_position{suffix}_controllers.yaml',
                  motors=root/f'a5_vr_position{suffix}_motors.yaml',
                  original_motors=root/'a5_motors.yaml')
    if motion_range == 'full':
        result['limits'] = root/'a5_vr_position_full_limits.yaml'
    # Gravity snapshots are not position-control evidence.
    result.pop('gravity', None)
    return result


def position_profile(stage, controller, bridge, motion_range='trial'):
    if motion_range not in ('trial','full'): raise ValueError('invalid motion_range')
    params, cfg = deepcopy(controller), deepcopy(bridge)
    params['vr_allow_arm'] = stage != 'hold'
    cfg.update(max_velocity=[.03]*6+[.1], target_linear_speed=.005,
               rotation_enabled=False, local_workspace_radius=.005, max_joint_lead=.01,gripper_control=False,
               geometry_checking=True,hold_on_ik_failure=False,limit_reference_lead=False,bounded_ik=False,
               joint_goal_horizon=0.)
    if motion_range=='full':
        # 10x the operator's previous full-profile TCP and arm speed caps.
        cfg.update(max_velocity=[1.]*6+[.003],target_linear_speed=1.8,target_angular_speed=7.5,
                   rotation_enabled=True,local_workspace_radius=None,max_joint_lead=.03,
                   gripper_control=True,gripper_speed=.003,gripper_max_lead=.004,
                   stop_on_compute_overrun=False,stop_on_ik_stall=False,
                   geometry_checking=False,hold_on_ik_failure=True,limit_reference_lead=False,bounded_ik=True,
                   joint_goal_horizon=.25)
        # A 1 rad/s output with 3 rad/s² acceleration needs >0.1747 rad of
        # goal distance at 125 Hz. These are future IK goals, not motor steps;
        # C++ alone generates the velocity/acceleration-bounded output.
        # User-requested Piper-style position mode: do not depend on the
        # uncalibrated CAD collision model or its folded ±2-degree exception.
        if 'zero_pose_collision_tolerance' in cfg:
            cfg['zero_pose_collision_tolerance']['enabled'] = False
    return params, cfg


def position_issues(paths, kin, params, manifest, motion_range='trial'):
    """Check mapping and both speed/position boundaries, never inherit HOLD proof."""
    issues = reference_limit_issues(params, kin)
    if motion_range not in ('trial','full'): return ['invalid motion_range']
    full=motion_range=='full'
    try:
        motors = yaml.safe_load(paths['motors'].read_text())['joints']
        original = yaml.safe_load(paths['original_motors'].read_text())['joints']
        limits = yaml.safe_load(paths['limits'].read_text())['joint_limits'] if full else None
        calibration = yaml.safe_load(paths['calibration'].read_text())
        issues += field_issues(calibration, motors, kin.model, JOINTS, include_geometry=False)
        if params.get('reference_mode') != 'external_vr': issues.append('position requires external_vr')
        for i, name in enumerate(JOINTS):
            motor, source = motors[name], original[name]
            for key in ('motor_type', 'slave_id', 'master_id', 'sign', 'offset', 'position_scale'):
                if motor.get(key, 1. if key == 'position_scale' else None) != source.get(key, 1. if key == 'position_scale' else None):
                    issues.append('position motor mapping changed: '+name+'/'+key)
            if i == 6:
                if full:
                    if motor.get('read_only') is not False or motor['max_velocity'] != .003 or motor['min_position'] != kin.lower[6] or motor['max_position'] != kin.upper[6]+kin.zero_tolerance[6]:
                        issues.append('full position gripper driver configuration mismatch')
                elif motor.get('read_only') is not True: issues.append('position gripper must remain read_only')
                continue
            if motor.get('read_only', False): issues.append('position arm joint is read_only: '+name)
            lo = kin.lower[i] - (kin.zero_tolerance[i] if kin.lower[i] == 0 else 0.)
            hi = kin.upper[i] + (kin.zero_tolerance[i] if kin.upper[i] == 0 else 0.)
            if motor['min_position'] != lo or motor['max_position'] != hi:
                issues.append('position driver/gate bounds mismatch: '+name)
            speeds = [params['vr_reference_velocity'][i], params['position_velocity_limit'][i],
                      motor['max_velocity'], params['vr_actual_velocity'][i]]
            if not np.isfinite(speeds).all() or not 0 < speeds[0] <= speeds[1] <= speeds[2] <= speeds[3] <= (1.25 if full else .1):
                issues.append('position speed limits inconsistent: '+name)
            if full and (limits[name]['max_velocity'] != 1. or motor['max_velocity'] != 1.):
                issues.append('full position IK/driver speed mismatch: '+name)
        expected = dict(vr_reference_velocity=[.03]*6, vr_reference_acceleration=[.1]*6,
                        position_velocity_limit=[.03]*6, vr_actual_velocity=[.1]*6,
                        vr_session_reference_travel=float(np.deg2rad(1.)),
                        vr_session_actual_travel=float(np.deg2rad(2.)), vr_tracking_error=.015)
        if full:
            expected.update(vr_reference_velocity=[1.]*6,vr_reference_acceleration=[3.]*6,
                position_velocity_limit=[1.]*6,vr_actual_velocity=[1.25]*6,
                vr_session_reference_travel=0.,vr_session_actual_travel=0.,vr_tracking_error=.05,
                vr_gripper_control=True,vr_gripper_speed=.003,vr_gripper_acceleration=.01,
                vr_gripper_actual_speed=.01,vr_gripper_tracking_error=.004,
                vr_auto_rearm_after_release=True,vr_trajectory_goals=True,vr_tracking_governor=True)
        for key, value in expected.items():
            if not np.array_equal(params.get(key), value): issues.append('position trial constraint changed: '+key)
        if not manifest.get('independent_stop', {}).get('device_operator_confirmed'):
            issues.append('independent power cutoff confirmation missing')
    except (KeyError, TypeError, ValueError, OSError, IndexError) as error:
        issues.append('malformed position configuration: '+str(error))
    return issues
