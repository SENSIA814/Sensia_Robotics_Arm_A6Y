"""Fail-closed, file-based physical commissioning checks; no hardware actions."""
import hashlib
from copy import deepcopy
from pathlib import Path
import numpy as np
import yaml
from .a5_kinematics import JOINTS
from .field_calibration import field_issues, positive


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reference_limit_issues(controller, kin):
    """Keep controller and IK acceptance identical, including in readonly/mock."""
    issues = []
    expected = dict(vr_lower=kin.lower[:6], vr_upper=kin.upper[:6],
                    vr_zero_tolerance=kin.zero_tolerance,
                    vr_gripper_lower=kin.lower[6], vr_gripper_upper=kin.upper[6],
                    vr_feedback_tolerance=0.)
    for key, value in expected.items():
        if not np.array_equal(controller.get(key), value):
            issues.append('VR limit mismatch: '+key)
    return issues


def commissioning_issues(manifest, paths, kin, bridge_config):
    issues = []
    try:
        motors = yaml.safe_load(paths['motors'].read_text())['joints']
        calibration = yaml.safe_load(paths['calibration'].read_text())
        issues.extend(field_issues(calibration, motors, kin.model, JOINTS))
        limits = yaml.safe_load(paths['limits'].read_text())['joint_limits']
        gravity = yaml.safe_load(paths['gravity'].read_text())['/arm6/a6_y_gravity_comp_controller']['ros__parameters']
        controller = yaml.safe_load(paths['controller'].read_text())['/arm6/a6_y_joint_impedance_controller']['ros__parameters']
        issues.extend(reference_limit_issues(controller, kin))
        for i, joint in enumerate(JOINTS):
            if limits[joint]['max_effort'] != gravity['torque_limits'][i]: issues.append('torque cap mismatch: '+joint)
        for key in ('torque_scales', 'torque_limits', 'effort_signs', 'damping_gains'):
            if controller[key] != gravity[key]: issues.append('gravity snapshot mismatch: '+key)
        if any(controller['joint_integral_gains']): issues.append('VR first-stage Ki must be zero')
        if manifest.get('status') != 'verified': issues.append('commissioning status pending')
        for name, path in paths.items():
            if manifest.get('sha256', {}).get(name) != sha256(path): issues.append('unverified snapshot: '+name)
        for name in ('axis_mapping', 'independent_stop', 'motor_command_loss', 'supported_enable', 'actual_quest'):
            item = manifest.get(name, {})
            if item.get('status') != 'verified' or not item.get('date') or not item.get('operator') or not item.get('evidence'):
                issues.append(name+': dated physical evidence pending')
        stop = manifest.get('stopping', {})
        trials = stop.get('trials', [])
        if (stop.get('status') != 'verified' or not positive(stop.get('max_distance_m')) or
                not positive(stop.get('max_joint_travel_rad')) or not positive(stop.get('max_stop_time_s'))):
            issues.append('stopping: limits and supported-stop trials pending')
        elif len(trials) < 3 or not {'release', 'input_loss', 'bridge_loss'} <= {t.get('cause') for t in trials}:
            issues.append('stopping: release/input_loss/bridge_loss evidence required')
        else:
            for trial in trials:
                if trial.get('hardware_accessed') is not True or not trial.get('evidence'):
                    issues.append('stopping: physical evidence required'); break
                for metric, bound in (('distance_m', 'max_distance_m'), ('joint_travel_rad', 'max_joint_travel_rad'), ('stop_time_s', 'max_stop_time_s')):
                    value = trial.get(metric)
                    if not isinstance(value, (float, int)) or isinstance(value, bool) or not np.isfinite(value) or not 0 <= value <= stop[bound]:
                        issues.append('stopping: invalid/exceeded '+metric)
        geo = manifest.get('geometry', {})
        uncertainty = geo.get('model_and_tracking_margin_m')
        if geo.get('status') != 'verified' or not positive(uncertainty) or not geo.get('evidence'):
            issues.append('geometry: measured model/obstacle/clearance evidence pending')
        elif positive(stop.get('max_distance_m')) and bridge_config['mesh_clearance_m'] < uncertainty+stop['max_distance_m']:
            issues.append('geometry: clearance smaller than uncertainty plus measured stopping travel')
        lo, hi, floor = (bridge_config.get(k) for k in ('workspace_lower', 'workspace_upper', 'floor_z_m'))
        if (np.shape(lo) != (3,) or np.shape(hi) != (3,) or not np.isfinite([lo, hi]).all() or
                np.any(np.asarray(lo) >= hi) or floor is None or not np.isfinite(floor)):
            issues.append('geometry: measured workspace/floor not configured')
    except (KeyError, TypeError, ValueError, OSError) as error:
        issues.append('malformed commissioning/configuration: '+str(error))
    return issues


def commissioning_profile(stage, controller, bridge):
    """Fixed, narrower trial constraints; hardware never accepts synthetic input."""
    if stage not in ('hold', 'follow'): raise ValueError('commissioning stage must be hold/follow')
    params, cfg = deepcopy(controller), deepcopy(bridge)
    params.update(vr_allow_arm=stage == 'follow', vr_allow_test_input=False, vr_ramp_gravity_on_prepare=True,
                  impedance_ramp_duration=max(2., float(controller['impedance_ramp_duration'])),
                  vr_session_reference_travel=float(np.deg2rad(1.)),
                  vr_session_actual_travel=float(np.deg2rad(2.)),
                  vr_reference_velocity=[.03]*6, vr_reference_acceleration=[.1]*6,
                  vr_actual_velocity=[.1]*6, vr_tracking_error=.015)
    cfg.update(max_velocity=[.03]*6+[.1], target_linear_speed=.005,
               rotation_enabled=False, local_workspace_radius=.005, max_joint_lead=.01)
    return params, cfg


def commissioning_stage_issues(stage, manifest, paths, kin, bridge_config, *, operator_trial=False):
    """Hold can produce its own evidence; follow can produce stopping evidence.

    An explicitly requested follow trial may defer remaining acceptance evidence.
    It still requires confirmed mapping, a supported HOLD result, and an operator-
    confirmed independent power cutoff. The full execute gate remains unchanged.
    """
    if stage not in ('hold', 'follow'): return ['invalid commissioning stage']
    if operator_trial and stage != 'follow': return ['operator_trial requires follow stage']
    issues = []
    try:
        motors = yaml.safe_load(paths['motors'].read_text())['joints']
        calibration = yaml.safe_load(paths['calibration'].read_text())
        issues.extend(field_issues(calibration, motors, kin.model, JOINTS,
                                  include_geometry=stage == 'follow' and not operator_trial))
        limits = yaml.safe_load(paths['limits'].read_text())['joint_limits']
        gravity = yaml.safe_load(paths['gravity'].read_text())['/arm6/a6_y_gravity_comp_controller']['ros__parameters']
        controller = yaml.safe_load(paths['controller'].read_text())['/arm6/a6_y_joint_impedance_controller']['ros__parameters']
        issues.extend(reference_limit_issues(controller, kin))
        if controller.get('reference_mode') != 'external_vr': issues.append('commissioning requires external_vr')
        if any(controller['joint_integral_gains']): issues.append('VR first-stage Ki must be zero')
        for i, joint in enumerate(JOINTS):
            if limits[joint]['max_effort'] != gravity['torque_limits'][i]: issues.append('torque cap mismatch: '+joint)
        for key in ('torque_scales', 'torque_limits', 'effort_signs', 'damping_gains'):
            if controller[key] != gravity[key]: issues.append('gravity snapshot mismatch: '+key)
        if stage == 'hold': return issues
        record = manifest.get('supported_enable', {})
        if record.get('status') != 'verified' or not all(record.get(k) for k in ('date', 'operator', 'evidence')):
            issues.append('supported_enable: physical evidence required before follow trial')
        if operator_trial:
            if not manifest.get('independent_stop', {}).get('device_operator_confirmed'):
                issues.append('independent power cutoff: operator confirmation required for trial')
            return issues
        for name in ('independent_stop', 'motor_command_loss', 'axis_mapping', 'actual_quest'):
            record = manifest.get(name, {})
            if record.get('status') != 'verified' or not all(record.get(k) for k in ('date', 'operator', 'evidence')):
                issues.append(name+': physical evidence required before follow trial')
        stop = manifest.get('stopping', {})
        # Limits are chosen before a trial. Measured stop results are its output.
        if not all(positive(stop.get(k)) for k in ('max_distance_m', 'max_joint_travel_rad', 'max_stop_time_s')):
            issues.append('stopping: acceptance limits must be set before follow trial')
        geo = manifest.get('geometry', {})
        margin = geo.get('model_and_tracking_margin_m')
        if geo.get('status') != 'verified' or not positive(margin) or not geo.get('evidence'):
            issues.append('geometry: measured clearance evidence required before follow trial')
        elif positive(stop.get('max_distance_m')) and bridge_config['mesh_clearance_m'] < margin+stop['max_distance_m']:
            issues.append('geometry: clearance smaller than uncertainty plus allowed stopping travel')
        lo, hi, floor = (bridge_config.get(k) for k in ('workspace_lower', 'workspace_upper', 'floor_z_m'))
        if (np.shape(lo) != (3,) or np.shape(hi) != (3,) or not np.isfinite([lo, hi]).all()
                or np.any(np.asarray(lo) >= hi) or floor is None or not np.isfinite(floor)):
            issues.append('geometry: workspace/floor required before follow trial')
    except (KeyError, TypeError, ValueError, OSError) as error:
        issues.append('malformed commissioning/configuration: '+str(error))
    return issues


def profile_paths():
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    ctrl = Path(get_package_share_directory('a6_y_controllers'))/'config'
    vr = Path(get_package_share_directory('a6_y_vr_teleop'))/'config'
    model = Path(get_package_share_directory('a5_description'))
    paths = dict(urdf=model/'urdf/A5.urdf', motors=ctrl/'a5_motors.yaml',
                 calibration=ctrl/'a5_calibration.yaml', limits=ctrl/'a5_motion_limits.yaml',
                 gravity=ctrl/'a5_gravity_controllers.yaml', controller=ctrl/'a5_vr_joint_impedance_controllers.yaml',
                 bridge=vr/'a5_vr_real.yaml', hardware=ctrl/'a5.urdf.xacro',
                 controller_binary=Path(get_package_prefix('a6_y_controllers'))/'lib/liba6_y_gravity_comp_controller.so',
                 driver_binary=Path(get_package_prefix('a6_y_ros2_control'))/'lib/liba6_y_damiao_system.so')
    for module in sorted(Path(__file__).parent.glob('*.py')): paths['python_'+module.stem] = module
    paths['launch'] = vr.parent/'launch/a5_vr_real.launch.py'
    for mesh in sorted((model/'meshes').glob('*.STL')): paths['mesh_'+mesh.stem] = mesh
    return paths, vr/'a5_vr_commissioning.yaml'
