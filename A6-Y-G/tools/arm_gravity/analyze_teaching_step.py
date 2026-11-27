#!/usr/bin/env python3
"""Offline manual-step diagnostics. Reads logs only; no ROS or motor access."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def number(value):
    try:
        return float(value)
    except (ValueError, TypeError):
        return float('nan')


def series(t, q):
    t, q = np.asarray(t), np.asarray(q)
    valid = np.isfinite(t) & np.isfinite(q) & (t > 0)
    t, q = t[valid], q[valid]
    order = np.argsort(t, kind='stable')
    t, q = t[order], q[order]
    # Keep the last snapshot for a repeated native send/feedback timestamp.
    unique = np.r_[np.diff(t) > 0, True] if len(t) else np.array([], dtype=bool)
    return t[unique], q[unique]


def asof(t, source_t, values, max_age=.1):
    source_t, values = series(source_t, values)
    if not len(source_t):
        return np.full(len(t), np.nan), np.full(len(t), np.nan)
    indices = np.searchsorted(source_t, t, side='right') - 1
    age = t - source_t[np.maximum(indices, 0)]
    valid = (indices >= 0) & (age >= 0) & (age <= max_age)
    return np.where(valid, values[np.maximum(indices, 0)], np.nan), np.where(valid, age, np.nan)


def crossing(t, normalized, threshold, start):
    indices = np.flatnonzero((t >= start) & (normalized >= threshold))
    return float(t[indices[0]]) if len(indices) else None


def transition(t, q, start, baseline_end):
    before = q[(t >= baseline_end - .6) & (t < baseline_end)]
    final = q[t >= t[-1] - 1.]
    if not len(before) or not len(final):
        return None
    initial, target = float(np.median(before)), float(np.median(final))
    amplitude = target - initial
    if abs(amplitude) < .015:
        return None
    normalized = (q - initial) / amplitude
    a, b = crossing(t, normalized, .1, start), crossing(t, normalized, .9, start)
    return dict(initial=initial, final=target, amplitude=amplitude, t10=a, t90=b,
                rise_s=b-a if a is not None and b is not None else None)


def response_metrics(t, teacher, target, actual, raw=None, safe=None):
    """One manual movement followed by a hold; t is host monotonic seconds."""
    t, teacher, target, actual = [np.asarray(x, dtype=float) for x in (t, teacher, target, actual)]
    result = dict(status='insufficient_data', flags=[])
    valid = np.isfinite(t) & np.isfinite(teacher) & np.isfinite(target) & np.isfinite(actual)
    result['invalid_samples'] = int(np.sum(~valid))
    t, teacher, target, actual = [x[valid] for x in (t, teacher, target, actual)]
    raw = np.asarray(raw)[valid] if raw is not None else None
    safe = np.asarray(safe)[valid] if safe is not None else None
    if len(t) < 50 or t[-1] - t[0] < 2.5:
        return result
    result['max_sample_gap_s'] = float(np.max(np.diff(t)))
    result['median_sample_interval_s'] = float(np.median(np.diff(t)))
    if result['max_sample_gap_s'] > .02 or result['invalid_samples'] > 0:
        result['flags'].append('sample_gaps_or_invalid_feedback')
    baseline = teacher[t < t[0] + .6]
    initial, final = float(np.median(baseline)), float(np.median(teacher[t >= t[-1]-1.]))
    amplitude = final-initial
    excursion = float(np.max(np.abs(teacher-initial)))
    result.update(teacher_amplitude_rad=amplitude, teacher_excursion_rad=excursion)
    if abs(amplitude) < .03:
        result['status'] = 'no_excitation' if excursion < .03 else 'returned_or_multiple_movements'
        return result
    if np.ptp(baseline) > max(.01, abs(amplitude)*.05):
        result['status'] = 'initial_teacher_not_stable'
        return result
    normalized = (teacher-initial)/amplitude
    onset = crossing(t, normalized, .05, t[0]+.6)
    if onset is None:
        return result
    base = (t >= onset-.6) & (t < onset)
    if np.ptp(actual[base]) > max(.015, abs(amplitude)*.05):
        result['flags'].append('initial_follower_not_stable')
    band = max(.01, abs(amplitude)*.02)
    result.update(analysis_band_rad=band, input_onset_s=onset-float(t[0]))
    tail = t >= t[-1]-1.
    if np.ptp(teacher[tail]) > 2*band:
        result['status'] = 'teacher_not_held'
        return result
    # Require a single direction. Quantization/noise below 5% or .015 rad is ignored.
    moving = normalized[t >= onset]
    reversal = float(np.max(np.maximum.accumulate(moving)-moving))*abs(amplitude)
    if reversal > max(.015, abs(amplitude)*.1):
        result['status'] = 'reversal_or_multiple_movements'
        return result
    teacher_transition = transition(t, teacher, onset-.05, onset)
    sent_transition = transition(t, target, onset-.05, onset)
    if not teacher_transition or not sent_transition:
        result['status'] = 'target_not_excited'
        return result
    for label, values in (('teacher', teacher_transition), ('sent', sent_transition)):
        result[label+'_rise_10_90_s'] = values['rise_s']
    for name, values in (('raw', raw), ('safe', safe)):
        if values is not None and np.isfinite(values).all():
            change = transition(t, values, onset-.05, onset)
            if change:
                result[name+'_rise_10_90_s'] = change['rise_s']
                if teacher_transition['t10'] is not None and change['t10'] is not None:
                    result['teacher_to_'+name+'_t10_s'] = change['t10']-teacher_transition['t10']
    q0, q1 = sent_transition['initial'], sent_transition['final']
    step = q1-q0
    response = (actual-q0)/step
    y10, y90 = crossing(t,response,.1,onset-.05), crossing(t,response,.9,onset-.05)
    result.update(sent_amplitude_rad=step, actual_rise_10_90_s=y90-y10 if y90 is not None and y10 is not None else None,
                  sent_to_actual_t10_s=y10-sent_transition['t10'] if y10 is not None and sent_transition['t10'] is not None else None,
                  teacher_to_actual_t10_s=y10-teacher_transition['t10'] if y10 is not None and teacher_transition['t10'] is not None else None)
    if result['sent_to_actual_t10_s'] is not None and abs(result['sent_to_actual_t10_s']) < 1e-9:
        result['flags'].append('response_below_observation_resolution')
    elif result['sent_to_actual_t10_s'] is not None and result['sent_to_actual_t10_s'] < 0:
        result['flags'].append('negative_observed_delay_requires_review')
    # Final command must itself have stopped. Otherwise an apparent settling
    # time conflates controller response with a still-moving target.
    held = np.abs(target-q1) <= band
    indices = np.flatnonzero((t >= onset) & ~held)
    held_index = indices[-1]+1 if len(indices) else np.searchsorted(t,onset)
    held_time = float(t[held_index]) if held_index < len(t) else None
    result['target_hold_duration_s'] = float(t[-1]-held_time) if held_time is not None else 0.
    active = t >= onset
    error = target[active]-actual[active]
    result.update(steady_error_rad=float(np.median(target[tail]-actual[tail])),
                  peak_abs_error_rad=float(np.max(np.abs(error))), rms_error_rad=float(np.sqrt(np.mean(error**2))),
                  overshoot_percent=max(0., float(np.max(response[active])-1.)*100.))
    inside = (np.abs(actual-q1) <= band) & held & active
    last_outside = np.flatnonzero(active & ~inside)
    settled_index = last_outside[-1]+1 if len(last_outside) else np.searchsorted(t,onset)
    settled = settled_index < len(t) and t[-1]-t[settled_index] >= .5
    result['settling_from_teacher_onset_s'] = float(t[settled_index]-onset) if settled else None
    result['settling_after_target_hold_s'] = float(max(0.,t[settled_index]-held_time)) if settled and held_time is not None else None
    result['settling_from_sent_t10_s'] = float(t[settled_index]-sent_transition['t10']) if settled and sent_transition['t10'] is not None else None
    result['input_shape'] = 'step_like' if max(teacher_transition['rise_s'] or 0., sent_transition['rise_s'] or 0.) <= .05 else 'ramp_or_limited'
    # Separate spaced movements cannot be reliably fit as one step. Detect a
    # long stationary plateau inside the 10-90% transition.
    middle = (normalized > .1) & (normalized < .9) & active
    plateau_start = None
    for k in np.flatnonzero(middle):
        if plateau_start is None or abs(teacher[k]-teacher[plateau_start]) > band:
            plateau_start = k
        elif t[k]-t[plateau_start] > .75:
            result['flags'].append('intermediate_pause_or_multiple_moves')
            break
    result['status'] = 'measured' if settled else 'not_settled'
    if result['target_hold_duration_s'] < 1.:
        result['status'] = 'target_not_held_long_enough'
    result['parameter_fit_allowed'] = (result['status'] == 'measured' and not result['flags'] and result['input_shape'] == 'step_like')
    return result


def load_arms(path):
    fields = ['receive_wall','receive_monotonic','control_time','dry_run'] + [f'{name}_j{i}'
        for i in range(1,8) for name in ('position','sent_position','feedback_receive_time',
            'send_complete_time','sent_mode','command_count','position_valid','position_age','fault','enabled')]
    rows = {}
    with path.open() as stream:
        for row in csv.DictReader(stream):
            rows.setdefault(row['role'],[]).append([number(row.get(name)) for name in fields])
    return {role:dict(zip(fields,np.array(values).T)) for role,values in rows.items()}


def load_targets(path):
    rows = {}
    clock_bad = False
    with path.open() as stream:
        for row in csv.DictReader(stream):
            ros, wall, mono = [number(row[x]) for x in ('source_ros_time','receive_wall','receive_monotonic')]
            clock_bad |= not np.isfinite(ros) or not -.02 <= wall-ros <= .5
            rows.setdefault((row['follower'],row['kind']),[]).append([ros+mono-wall] +
                [number(row.get(f'q_j{i}')) for i in range(1,7)])
    return {key:np.array(values).T for key,values in rows.items()}, bool(clock_bad)


def clean_json(value):
    if isinstance(value, dict):return {k:clean_json(v) for k,v in value.items()}
    if isinstance(value, list):return [clean_json(v) for v in value]
    if isinstance(value, float) and not np.isfinite(value):return None
    return value


def analyze_folder(folder):
    folder = Path(folder)
    quality = json.loads((folder/'step_quality.json').read_text())
    arms = load_arms(folder/'step_arms.csv')
    targets, bad_clock = load_targets(folder/'step_targets.csv')
    results = []
    fields = ['leader','follower','joint','time_monotonic','teacher_q','raw_q','safe_q',
              'sent_q','actual_q','sent_error','teacher_error','teacher_age_s','sent_age_s','raw_age_s','safe_age_s']
    with (folder/'step_tracking.csv').open('w',newline='') as stream:
        writer = csv.writer(stream);writer.writerow(fields)
        for leader,follower in quality['pairs']:
            for joint in range(1,8):
                result = dict(leader=leader,follower=follower,joint=joint,status='missing_data',flags=[])
                if joint == 7:
                    controlled = follower in arms and np.any(arms[follower]['command_count_j7'] > 0)
                    result['status'] = 'gripper_recorded_not_analyzed' if controlled else 'read_only_gripper'
                    result['position_unit'] = 'm_single_finger'
                    if controlled:
                        result['flags'].append('J7 raw targets and feedback retained; radian step thresholds do not apply')
                elif leader in arms and follower in arms:
                    a,b = arms[leader],arms[follower]
                    key = lambda name:f'{name}_j{joint}'
                    good = (b[key('position_valid')] == 1) & (b[key('fault')] == 0) & (b[key('enabled')] == 1) & (b[key('position_age')] < .1)
                    t,actual = series(b[key('feedback_receive_time')],np.where(good,b[key('position')],np.nan))
                    teacher_good = (a[key('position_valid')] == 1) & (a[key('fault')] == 0) & (a[key('position_age')] < .1)
                    teacher,ta = asof(t,a[key('feedback_receive_time')],np.where(teacher_good,a[key('position')],np.nan))
                    sent,sa = asof(t,b[key('send_complete_time')],np.where(b[key('sent_mode')]==2,b[key('sent_position')],np.nan))
                    mapped = {}
                    for kind in ('raw','safe'):
                        source = targets.get((follower,kind))
                        mapped[kind] = asof(t,source[0],source[joint]) if source is not None and not bad_clock else (np.full(len(t),np.nan),np.full(len(t),np.nan))
                    raw,ra = mapped['raw'];safe,fa = mapped['safe']
                    result.update(response_metrics(t,teacher,sent,actual,raw,safe))
                    if bad_clock:result['flags'].append('ROS_system_clock_mismatch_or_stale_target')
                    for role in (leader,follower):
                        cycles = quality.get('cycles',{}).get(role,{})
                        if cycles.get('skipped',0) or cycles.get('resets',0):
                            result['flags'].append(role+':native_cycles_missing_or_reset')
                    if np.any(b['dry_run'] != 0):result['flags'].append('dry_run_no_motor_dynamics')
                    if quality.get('devices',{}).get(follower,'').startswith('/dev/pts/'):
                        result['flags'].append('virtual_motor_no_physical_dynamics')
                    if not quality.get('complete') or any(quality.get('queue_dropped',{}).values()):
                        result['flags'].append('incomplete_or_dropped_log')
                    result['parameter_fit_allowed'] = bool(result.get('parameter_fit_allowed') and not result['flags'])
                    for values in zip(t,teacher,raw,safe,sent,actual,sent-actual,teacher-actual,ta,sa,ra,fa):
                        writer.writerow([leader,follower,joint]+[format(x,'.9f') if np.isfinite(x) else '' for x in values])
                results.append(result)
    report = clean_json(dict(schema=1, results=results,
        definitions=dict(response_delay='10% actual crossing minus 10% target/teacher crossing; observed pipeline delay, not pure transport latency',
            rise_time='10% to 90% of final sent target change',
            settling='Within ±max(0.01 rad,2% teacher amplitude) of final target for >=0.5 s and through end of recording',
            steady_error='Median sent target minus actual over last 1 s',
            input_shape='Manual movement and original follower-relative limiter may create a ramp; not an ideal motor step',
            alignment='As-of native steady timestamps; raw/safe ROS times converted using same-host callback wall/monotonic offset',
            missing='Blank/null means unavailable or criterion not reached, never zero delay'),
        quality_file='step_quality.json', tracking_file='step_tracking.csv'))
    (folder/'step_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
    columns = ['leader','follower','joint','status','flags'] + sorted({k for r in results for k in r}-{'leader','follower','joint','status','flags'})
    with (folder/'step_summary.csv').open('w',newline='') as stream:
        writer = csv.DictWriter(stream,columns);writer.writeheader()
        for result in report['results']:
            writer.writerow(dict(result,flags=';'.join(result['flags'])))
    labels=dict(measured='已测得',not_settled='未稳定',no_excitation='未激励',read_only_gripper='夹爪只读',
                gripper_recorded_not_analyzed='夹爪已记录（米），不套用六轴角度阶跃阈值',
        insufficient_data='数据不足',missing_data='缺少数据',returned_or_multiple_movements='回到起点/多次移动',
        initial_teacher_not_stable='初始示教姿态未静止',teacher_not_held='示教端未保持静止',
        reversal_or_multiple_movements='反向/多次移动',target_not_excited='下发目标变化不足',
        target_not_held_long_enough='目标保持时间不足')
    lines=['手动阶跃采集分析（只读，不调参）',
        '时间单位 s；误差单位 rad。— 表示无法计算或尚未达到条件，不表示 0。',
        '响应延迟为目标/反馈的 10% 交叉时间差；不是电机内部控制周期或纯 CAN 传输延迟。',
        '稳定时间从示教开始变化算起，要求进入 ±max(0.01 rad,2%幅值) 并持续至结束至少 0.5 s。',
        '手动移动或限步形成斜坡时，仅供跟随响应参考；有采样缺口时需结合原始日志复核。','']
    for r in report['results']:
        def show(key):
            value=r.get(key)
            if isinstance(value,(int,float)) and key.endswith('_s') and abs(value)<1e-9:
                return f"同一采样间隔内（约 {r.get('median_sample_interval_s',0):.4f} s）"
            return f'{value:.4f}' if isinstance(value,(int,float)) else '—'
        lines.append(f"{r['leader']} → {r['follower']} J{r['joint']}：{labels.get(r['status'],r['status'])}")
        if 'actual_rise_10_90_s' in r:
            lines.append(f"  示教→实际延迟 {show('teacher_to_actual_t10_s')}；下发→实际延迟 {show('sent_to_actual_t10_s')}；"
                         f"上升时间 {show('actual_rise_10_90_s')}；稳定时间 {show('settling_from_teacher_onset_s')}；"
                         f"稳态误差 {show('steady_error_rad')}；超调 {show('overshoot_percent')}%。")
            lines.append('  输入形状：'+('近似阶跃' if r.get('input_shape')=='step_like' else '斜坡/限步输入'))
        if r['flags']:lines.append('  数据标记：'+'; '.join(r['flags']))
    (folder/'step_summary.txt').write_text('\n'.join(lines)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path,help='Teaching session containing step_*.csv')
    args = parser.parse_args()
    report = analyze_folder(args.folder)
    print(json.dumps(report,ensure_ascii=False,indent=2))
