"""Serial-bound measured gripper travel and inset percentage goals for teaching.

Calibration is in motor radians. Metres remain the existing model coordinate,
not a measured jaw gap. Independent MoveIt/model configuration is untouched.
"""
import hashlib
import math
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[2]
CALIBRATION = Path(__file__).with_name('teaching_gripper_limits.yaml')

ARM_JOINTS = tuple(f'joint{i}' for i in range(1, 7))
GRIPPER_JOINT = 'gripper_finger_joint1'
# 2026-11-27 19:58 log: 80 mm/s leaves ~0.08-0.17 s limiter delay in fast closing.
# Next trial: ~0.40 s full usable stroke, 4.184 rad/s at the motor.
# Model displacement, not measured jaw gap; physical validation still required.
TEACHING_GRIPPER_SPEED_M_S = .100
TEACHING_GRIPPER_STATUS_PERIOD_S = .010  # Read-only leader J7; quantized by 4 ms loop.
STARTUP_ZERO_TOLERANCE_M = .0001  # 0.1 mm: holding/feedback only, not target travel.


def controlled_joints(model):
    role = model['role']
    if role in ('motion_left', 'motion_right', 'teaching_left', 'teaching_right'):
        return ARM_JOINTS + (GRIPPER_JOINT,)
    raise ValueError('未知夹爪示教角色：' + str(role))


def gripper_profile(model, max_velocity_m_s=None, calibration_path=None):
    """Validate measured endpoints/identity before any teaching controller starts."""
    controlled_joints(model)  # Validate the role even for a read-only leader.
    if max_velocity_m_s is None:
        # Teaching owns its follow speed; independent MoveIt keeps its own speed.
        max_velocity_m_s = TEACHING_GRIPPER_SPEED_M_S
    joint = model['joints'][GRIPPER_JOINT]
    low, high = float(joint['lower']), float(joint['upper'])
    scale, offset, sign = (float(joint[key]) for key in ('position_scale', 'offset', 'sign'))
    pmax = float(joint['position_scale_evidence']['pmax_rad'])
    values = (low, high, scale, offset, sign, pmax, max_velocity_m_s)
    if (not all(math.isfinite(x) for x in values) or not low < high or scale <= 0
            or sign not in (-1., 1.) or pmax <= 0 or max_velocity_m_s <= 0):
        raise ValueError('夹爪模型范围、坐标换算或速度限制无效')
    if joint.get('type') != 'prismatic':
        raise ValueError('夹爪跟随要求米制单指移动关节')
    path = Path(calibration_path) if calibration_path is not None else CALIBRATION
    source = path.read_bytes()
    config = yaml.safe_load(source)
    if not isinstance(config, dict) or config.get('schema_version') != 1:
        raise ValueError('夹爪实测端点配置版本无效')
    saved = config.get('roles', {}).get(model['role'])
    if not isinstance(saved, dict) or str(saved.get('usb_serial')) != model['arm']['usb_serial']:
        raise ValueError('夹爪实测端点的角色与 USB 身份不符')
    if joint.get('motor_id') != 7 or saved.get('motor_id') != 7:
        raise ValueError('夹爪实测端点必须对应 J7')
    names = ('closed_motor_rad', 'open_motor_rad', 'pmax_rad')
    values = [saved.get(n) for n in names] + [config.get(n) for n in
              ('endpoint_margin_motor_rad', 'feedback_tolerance_motor_rad')]
    if any(type(v) not in (float, int) or not math.isfinite(v) for v in values):
        raise ValueError('夹爪实测端点、量程和余量必须是有限数值')
    closed_rad, open_rad, recorded_pmax, margin_rad, tolerance_rad = values
    if not math.isclose(pmax, recorded_pmax, rel_tol=1e-6, abs_tol=1e-6):
        raise ValueError('夹爪 PMAX 与端点采集时不一致，需要重新核对')
    if not -pmax < open_rad < closed_rad < pmax:
        raise ValueError('夹爪实测电机端点顺序或协议范围无效')
    if not 0 < tolerance_rad < margin_rad or 2*margin_rad >= closed_rad-open_rad:
        raise ValueError('夹爪内收余量或反馈容差无效')
    measured_open = sign*scale*open_rad+offset
    measured_closed = sign*scale*closed_rad+offset
    opened = sign*scale*(open_rad+margin_rad)+offset
    closed = sign*scale*(closed_rad-margin_rad)+offset
    if not measured_open < opened < closed < measured_closed:
        raise ValueError('夹爪模型方向与实测张开方向不一致，请核对关节映射')
    encoder_lsb = pmax * 2 / 65535 * scale
    tolerance = max(encoder_lsb, tolerance_rad*scale)
    if tolerance >= margin_rad*scale:
        raise ValueError('夹爪编码分辨率不足以区分预留余量')
    return dict(role=model['role'], usb_serial=model['arm']['usb_serial'],
                joint=GRIPPER_JOINT, position_unit='m (single-finger joint)',
                open_position_m=opened, closed_position_m=closed,
                model_min_position_m=low, model_max_position_m=high,
                measured_open_position_m=measured_open, measured_closed_position_m=measured_closed,
                measured_open_motor_rad=open_rad, measured_closed_motor_rad=closed_rad,
                open_motor_rad=open_rad+margin_rad, closed_motor_rad=closed_rad-margin_rad,
                endpoint_margin_motor_rad=margin_rad, endpoint_margin_model_m=margin_rad*scale,
                min_position_m=measured_open-tolerance, max_position_m=measured_closed+tolerance,
                # Calibration defines percentage goals, not a feedback interlock.
                # Admit decoded positions outside those endpoints for startup/hold;
                # normal mapped goals still use the inset open/closed values above.
                hardware_min_position_m=offset-pmax*scale, hardware_max_position_m=offset+pmax*scale,
                feedback_position_policy='protocol_range_only', measured_endpoint_guard_enabled=False,
                encoder_lsb_m=encoder_lsb, startup_zero_tolerance_m=STARTUP_ZERO_TOLERANCE_M,
                feedback_tolerance_m=tolerance, max_velocity_m_s=float(max_velocity_m_s),
                endpoint_source=str(path), endpoint_source_sha256=hashlib.sha256(source).hexdigest(),
                calibration_id=config['calibration_id'], measured_at=config['measured_at'],
                endpoint_calibration_verified=True, physical_jaw_gap_measured=False,
                endpoint_note='Operator-positioned disabled J7 endpoints; inset targets; no repeatability or jaw-gap calibration.')


def opening_percent(profile, position_m):
    """The same 0--100% saturation as the affine mapper's final target clamp."""
    if not math.isfinite(position_m):
        raise ValueError('夹爪开度反馈无效')
    fraction = ((position_m-profile['closed_position_m']) /
                (profile['open_position_m']-profile['closed_position_m']))
    return 100*min(1., max(0., fraction))


def gripper_mapping(leader_model, follower_model):
    """Affine opening-fraction mapping; mapper limits clamp the final goal.

    u=(q_leader-closed_leader)/(open_leader-closed_leader),
    q_goal=closed_follower+u*(open_follower-closed_follower).
    This supports differing software endpoints without mixing raw encoder units.
    """
    if not leader_model['role'].startswith('teaching_') or not follower_model['role'].startswith('motion_'):
        raise ValueError('夹爪跟随必须由示教端映射到运动端')
    leader, follower = gripper_profile(leader_model), gripper_profile(follower_model)
    scale = ((follower['open_position_m']-follower['closed_position_m']) /
             (leader['open_position_m']-leader['closed_position_m']))
    offset = follower['closed_position_m'] - scale*leader['closed_position_m']
    return dict(sign=1., scale=scale, offset=offset, leader=leader, follower=follower)
