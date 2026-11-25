"""Gripper goal limits and normalized opening; no device access or URDF edits."""
import math
from pathlib import Path
import yaml

CONFIG = Path(__file__).with_name('gripper_limits.yaml')


def load_limits(role, joint):
    config = yaml.safe_load(CONFIG.read_text())
    if config.get('schema_version') != 1:
        raise ValueError('夹爪电子限位配置版本无效')
    limits = dict(config['defaults'])
    limits.update(config.get('role_overrides', {}).get(role, {}))
    for key in ('closed_margin_mm', 'open_margin_mm'):
        value = limits.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('夹爪端点余量必须为有限正数：'+key)
    opened = joint['lower'] + limits['open_margin_mm']/1000
    closed = joint['upper'] - limits['closed_margin_mm']/1000
    if not opened < closed:
        raise ValueError('夹爪电子限位余量过大，没有剩余行程')
    limits.update(open_position_m=opened, closed_position_m=closed, source=str(CONFIG))
    return limits


def position_from_percent(limits, percent):
    percent = float(percent)
    if not math.isfinite(percent) or not 0 <= percent <= 100:
        raise ValueError('夹爪开度需在 0～100%')
    return limits['closed_position_m'] + percent/100*(limits['open_position_m']-limits['closed_position_m'])


def percent_from_position(limits, position):
    if position is None:
        return None
    return 100*(limits['closed_position_m']-position)/(limits['closed_position_m']-limits['open_position_m'])


def validate_trajectory(limits, trajectory, start):
    """Reject an out-of-limit MoveIt path; allow recovery from an outer start.

    Do not clamp feedback or rewrite the native trajectory. These are goal/path
    limits, not a hardware stop or a physical end-stop calibration.
    """
    from motion_model import JOINTS
    names = list(trajectory.joint_names)
    if names != [JOINTS[6]] or not trajectory.points:
        raise ValueError('夹爪轨迹关节或轨迹点无效')
    values = [p.positions[0] for p in trajectory.points if len(p.positions) == 1]
    if len(values) != len(trajectory.points) or not all(math.isfinite(v) for v in values):
        raise ValueError('夹爪轨迹包含无效位置')
    lo, hi, eps = limits['open_position_m'], limits['closed_position_m'], 1e-8
    if not lo-eps <= values[-1] <= hi+eps:
        raise ValueError('夹爪规划终点超出电子限位，请重新规划')
    # Starting outside the electronic range is possible with read-only feedback
    # or after manual teaching. It may only move inward until back inside.
    lower, upper = min(lo, start), max(hi, start)
    for value in values:
        if not lower-eps <= value <= upper+eps:
            raise ValueError('夹爪规划路径超出电子限位，本次动作已取消')
        lower, upper = min(lo, value), max(hi, value)
