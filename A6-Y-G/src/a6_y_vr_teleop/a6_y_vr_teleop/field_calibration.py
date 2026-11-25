"""Measured calibration checks and explicitly recorded operator decisions."""
from datetime import date
import numpy as np
import pinocchio as pin


def positive(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and np.isfinite(value) and value > 0


def has_operator_confirmation(calibration):
    record = calibration.get('operator_confirmation', {})
    if not isinstance(record, dict): return False
    try:
        date.fromisoformat(record.get('date', ''))
    except (TypeError, ValueError):
        return False
    statement = record.get('statement')
    return record.get('source') == 'user_message' and isinstance(statement, str) and bool(statement.strip())


def field_summary(calibration):
    """Expose declared evidence types so accepted omissions remain visible."""
    records = {'base_installation': calibration['base_installation'],
               **{'joint_mapping.'+name: record for name, record in calibration['joint_mapping'].items()},
               'gripper_gap': calibration['gripper_gap'], 'fk_spot_checks': calibration['fk_spot_checks']}
    return dict(operator_confirmation=calibration.get('operator_confirmation'),
                operator_confirmed=[name for name, record in records.items() if record['status'] == 'operator_confirmed'],
                skipped_by_user=[name+'.zero_scale_test' for name, record in records.items()
                                 if record['status'] == 'operator_confirmed' and record.get('zero_scale_test') == 'skipped_by_user'])


def field_issues(calibration, motors, model, names, *, include_geometry=True):
    issues = []
    confirmed = has_operator_confirmation(calibration)
    base = calibration['base_installation']
    measured = base.get('measured_gravity_in_base_m_s2')
    tolerance = base.get('max_gravity_error_m_s2')
    if base['status'] == 'operator_confirmed':
        direction = np.asarray(base.get('gravity_direction_in_base'))
        gravity = np.asarray(model.gravity.linear)
        magnitude = np.linalg.norm(gravity)
        valid = (confirmed and bool(base.get('description')) and direction.shape == (3,)
                 and np.isfinite(direction).all() and np.isfinite(magnitude) and magnitude > 0
                 and np.allclose(direction, gravity/magnitude, atol=1e-9, rtol=0.))
    else:
        valid = (base['status'] == 'verified' and bool(base.get('description')) and positive(tolerance)
                 and measured is not None and np.asarray(measured).shape == (3,)
                 and np.isfinite(measured).all()
                 and np.linalg.norm(np.asarray(measured) - model.gravity.linear) <= tolerance)
    if not valid:
        issues.append('base_installation: gravity measurement or operator-confirmed orientation pending/inconsistent')
    for name in names:
        record = calibration['joint_mapping'][name]
        samples = record.get('samples', [])
        mapping_confirmed = record.get('motor_id_verified') is True and record.get('direction_verified') is True
        if record['status'] == 'operator_confirmed':
            # A user-authorized omission is not a successful measured calibration.
            valid = confirmed and mapping_confirmed and record.get('zero_scale_test') == 'skipped_by_user'
        else:
            valid = (record['status'] == 'verified' and mapping_confirmed
                     and positive(record.get('max_error')) and len(samples) >= 2)
        if valid and record['status'] == 'verified':
            m = motors[name]
            pairs = np.array([[s['motor_position'], s['reference_model_position']] for s in samples], dtype=float)
            predicted = m['sign'] * m.get('position_scale', 1) * pairs[:, 0] + m['offset']
            valid = (np.isfinite(pairs).all() and np.ptp(pairs[:, 0]) > 1e-6
                     and np.max(np.abs(predicted - pairs[:, 1])) <= record['max_error'])
        if not valid:
            issues.append(f'{name}: ID/direction and zero/scale measurement or explicit user decision incomplete/inconsistent')
    if not include_geometry: return issues
    gap = calibration['gripper_gap']
    samples = gap.get('samples', [])
    valid = (gap['status'] == 'verified' and positive(gap.get('max_error_m'))
             and isinstance(gap.get('closed_gap_m'), (int, float))
             and np.isfinite(gap['closed_gap_m']) and gap['closed_gap_m'] >= 0 and len(samples) >= 3)
    if valid:
        m = motors[names[-1]]
        values = np.array([[s['motor_position_rad'], s['measured_gap_m']] for s in samples], dtype=float)
        q = m['sign'] * m['position_scale'] * values[:, 0] + m['offset']
        predicted = gap['closed_gap_m'] - 2 * q
        valid = (np.isfinite(values).all() and len(np.unique(values[:, 0])) >= 3
                 and np.all(values[:, 1] >= 0) and np.all(q >= m['min_position']-1e-8)
                 and np.all(q <= m['max_position']+1e-8)
                 and np.max(np.abs(predicted-values[:, 1])) <= gap['max_error_m'])
    if not valid:
        issues.append('gripper_gap: three measured openings and closed gap pending or inconsistent')
    fk = calibration['fk_spot_checks']
    samples = fk.get('samples', [])
    valid = (fk['status'] == 'verified' and positive(fk.get('max_position_error_m'))
             and positive(fk.get('max_orientation_error_rad')) and len(samples) >= 3)
    if valid:
        frame = model.getFrameId('tool_tcp')
        distinct = set()
        for sample in samples:
            q = np.asarray(sample['q'], dtype=float)
            xyz = np.asarray(sample['measured_tcp_xyz_m'], dtype=float)
            rpy = np.asarray(sample['measured_tcp_rpy_rad'], dtype=float)
            if q.shape != (7,) or xyz.shape != (3,) or rpy.shape != (3,) or not all(np.isfinite(v).all() for v in (q, xyz, rpy)):
                valid = False
                break
            distinct.add(tuple(q))
            data = model.createData(); pin.framesForwardKinematics(model, data, q)
            actual = data.oMf[frame]
            if (np.linalg.norm(actual.translation-xyz) > fk['max_position_error_m']
                    or np.linalg.norm(pin.log3(actual.rotation.T @ pin.rpy.rpyToMatrix(rpy))) > fk['max_orientation_error_rad']):
                valid = False
        valid = valid and len(distinct) >= 3
    if not valid:
        issues.append('fk_spot_checks: three measured TCP poses pending or inconsistent')
    return issues
