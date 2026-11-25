"""Sample-based commissioning metrics; never a source of synthetic evidence."""
import numpy as np


def disarm_metrics(states, minimum_duration=.25):
    """DISARM must remain disabled, including the cycle after driver cleanup."""
    if len(states) < 2:
        return dict(passed=False, error='insufficient post-DISARM samples')
    stamps = np.asarray([s['monotonic_ns'] for s in states], dtype=float)
    duration = float((stamps[-1]-stamps[0])/1e9)
    valid = bool(np.isfinite(stamps).all() and np.all(np.diff(stamps) > 0)
                 and np.max(np.diff(stamps)) <= 100_000_000)
    disabled = all(s['state'] == 0 for s in states)
    return dict(passed=bool(valid and disabled and duration >= minimum_duration),
                duration_s=duration, remained_disabled=disabled, samples=len(states),
                final_state=states[-1]['state'], final_reason=states[-1]['reason'])


def hold_metrics(states, minimum_duration=10.):
    if len(states) < 2: return dict(passed=False, error='insufficient HOLD samples')
    stamps = np.asarray([s['monotonic_ns'] for s in states], dtype=float)
    q = np.asarray([s['actual'][:6] for s in states], dtype=float)
    v = np.asarray([s['velocity'][:6] for s in states], dtype=float)
    ref = np.asarray([s['reference'] for s in states], dtype=float)
    ages = np.asarray([s['position_age']+s['feedback_age'] for s in states], dtype=float)
    if not all(np.isfinite(x).all() for x in (stamps, q, v, ref, ages)):
        return dict(passed=False, error='nonfinite HOLD samples')
    duration = float((stamps[-1]-stamps[0])/1e9)
    drift, speed = float(np.max(np.abs(q-ref[0]))), float(np.max(np.abs(v)))
    fixed = bool(np.max(np.abs(ref-ref[0])) <= 1e-9)
    fresh = bool(np.all((ages >= 0) & (ages <= .05)) and np.all(np.diff(stamps) > 0)
                 and np.max(np.diff(stamps)) <= 100_000_000)
    states_valid = all(s['state'] == 1 and s['feedback_valid'] and not s['follow_permitted'] for s in states)
    saturation = any(any(s['saturated']) for s in states)
    return dict(passed=bool(duration >= minimum_duration and fixed and fresh and states_valid
                            and drift <= np.deg2rad(1.) and speed <= .03 and not saturation),
                samples=len(states), duration_s=duration, max_position_error_rad=drift,
                max_speed_rad_s=speed, max_feedback_age_s=float(np.max(ages)),
                reference_fixed=fixed, feedback_fresh=fresh, hold_only=states_valid,
                saturation_observed=saturation,
                criteria=dict(minimum_duration_s=minimum_duration,
                              max_position_error_rad=float(np.deg2rad(1.)), max_speed_rad_s=.03))
