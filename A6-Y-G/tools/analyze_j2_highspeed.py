#!/usr/bin/env python3
"""Export and compare J2 high-speed trials without hardware access."""
import argparse
import csv
import json
import math
from pathlib import Path
import re


def endpoint_metrics(rows, move_seconds):
    """First 1.5 s after the planned endpoint, including settling transients."""
    if not rows:
        return {}
    origin = rows[0]['t_ns']
    window = [r for r in rows if move_seconds <= (r['t_ns']-origin)*1e-9 <= move_seconds+1.5]
    if not window:
        return {}
    position = [math.degrees(r['actual']) for r in window]
    effort = [r['j2_hardware']['effort'] for r in window]
    return dict(window_coverage_s=(window[-1]['t_ns']-window[0]['t_ns'])*1e-9,
        position_span_deg=max(position)-min(position), effort_peak_nm=max(map(abs, effort)),
        effort_span_nm=max(effort)-min(effort),
        reference_span_deg=math.degrees(max(r['reference'] for r in window)-min(r['reference'] for r in window)))


def operator_acceptance(run, observations):
    values = [r['acceptable'] for r in observations if 'acceptable' in r and
              (r.get('run') == run or run in r.get('runs', []))]
    return all(v is True for v in values) if values else None


def summarize(path, criteria, observations=()):
    report = json.loads(path.read_text())
    metrics = report.get('metrics', {})
    result = {k: report.get(k) for k in ('mode', 'passed', 'disarm_confirmed', 'error',
        'start_degrees', 'end_degrees', 'move_seconds', 'planned_peak_speed_deg_s')}
    result.update(metrics)
    result['run'] = path.parent.name
    match = re.search(r'_kp([0-9]+)_', path.parent.name)
    result['kp_apr'] = int(match[1]) if match else None
    native_file = path.parent/'native_parameters.json'
    result['native_parameters'] = json.loads(native_file.read_text()) if native_file.exists() else {}
    holds = metrics.get('holds', {})
    target = report.get('planned_peak_speed_deg_s', 0.)
    peak = metrics.get('encoder_peak_speed_deg_s', 0.)
    lower, upper = criteria['peak_speed_fraction']
    checks = dict(
        complete_and_disarmed=bool(report.get('passed') and report.get('disarm_confirmed')),
        speed=target > 0 and lower*target <= peak <= upper*target,
        plan_error=metrics.get('plan_error_max_deg', math.inf) <= criteria['plan_error_max_deg'],
        holds=len(holds) == 2 and all(h['span_deg'] <= criteria['hold_last_second_span_deg'] and
            abs(h['mean_error_deg']) <= criteria['hold_mean_error_deg'] for h in holds.values()),
        overshoot=metrics.get('overshoot_deg', math.inf) <= criteria['overshoot_deg'])
    result['criteria_checks'] = checks
    direction_rows = {'outbound': [], 'return': []}
    with (path.parent/'trace.csv').open('w') as out:
        writer = csv.writer(out)
        writer.writerow(['time_s', 'phase', 'reason'] + [f'{key}_j{i}_{unit}'
            for key, unit in [('plan', 'deg'), ('goal', 'deg'), ('sent', 'deg'), ('actual', 'deg'), ('effort', 'Nm')]
            for i in range(1, 7)])
        origin = None
        for line in (path.parent/'trace.jsonl').open():
            row = json.loads(line)
            if 't_ns' not in row:
                continue
            if origin is None:
                origin = row['t_ns']
            values = [(row['t_ns']-origin)*1e-9, row['phase'], row['reason']]
            for key in ('plan_all', 'goal_all', 'reference_all', 'actual_all'):
                values.extend(math.degrees(v) for v in row.get(key, [math.nan]*6)[:6])
            values.extend(row['hardware'].get(f'joint{i}', {}).get('effort') for i in range(1, 7))
            writer.writerow(values)
            for direction in direction_rows:
                if row['phase'] == direction or row['phase'].startswith(direction+'_'):
                    direction_rows[direction].append(row)
    import numpy as np
    result['direction_metrics'] = {}
    result['endpoints'] = {}
    for direction, rows in direction_rows.items():
        if len(rows) < 3:
            continue
        t = np.array([(r['t_ns']-rows[0]['t_ns'])*1e-9 for r in rows])
        actual = np.array([r['actual'] for r in rows])
        plan = np.array([r['plan'] for r in rows])
        velocity = (np.interp(t+.1,t,actual)-np.interp(t-.1,t,actual))/.2
        result['direction_metrics'][direction] = dict(
            encoder_peak_speed_deg_s=float(np.rad2deg(np.max(np.abs(velocity)))),
            plan_error_max_deg=float(np.rad2deg(np.max(np.abs(plan-actual)))),
            effort_peak_nm=max(abs(r['j2_hardware'].get('effort',0.)) for r in rows))
        result['endpoints'][direction] = endpoint_metrics(rows, report['move_seconds'])
    if 'endpoint_span_deg' in criteria:
        checks['endpoints'] = len(result['endpoints']) == 2 and all(
            m.get('window_coverage_s', 0.) >= 1.4 and
            m.get('position_span_deg', math.inf) <= criteria['endpoint_span_deg']
            for m in result['endpoints'].values())
    checks['both_direction_speeds'] = len(result['direction_metrics']) == 2 and all(
        lower*target <= m['encoder_peak_speed_deg_s'] <= upper*target
        for m in result['direction_metrics'].values())
    result['performance_target_met'] = all(checks.values())
    result['operator_acceptable'] = operator_acceptance(result['run'], observations)
    result['accepted'] = result['performance_target_met'] and result['operator_acceptable'] is True
    return result


def plot_comparison(session, summaries):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for speed in sorted(set(r['planned_peak_speed_deg_s'] for r in summaries)):
        group = [r for r in summaries if r['passed'] and r['planned_peak_speed_deg_s'] == speed]
        if not group:
            continue
        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        for index, r in enumerate(group):
            rows = [json.loads(line) for line in (session/r['run']/'trace.jsonl').open()]
            rows = [x for x in rows if 't_ns' in x and x['phase'] in ('outbound', 'outbound_settle', 'outbound_hold')]
            if not rows:
                continue
            t = np.array([(x['t_ns']-rows[0]['t_ns'])*1e-9 for x in rows])
            plan, actual = [np.rad2deg([x[k] for x in rows]) for k in ('plan', 'actual')]
            velocity = (np.interp(t+.1, t, actual)-np.interp(t-.1, t, actual))/.2
            label = f"KP_APR={r['kp_apr']}" + (' repeat' if 'repeat' in r['run'] else '')
            if r['native_parameters'].get('KP_ASR') is not None:
                label += f"; ASR={r['native_parameters']['KP_ASR']:.5g}"
            if index == 0:
                axes[0].plot(t, plan, 'k--', label='Plan')
            axes[0].plot(t, actual, label=label)
            axes[1].plot(t, plan-actual, label=label)
            axes[2].plot(t, velocity, label=label)
        axes[0].set_title(f'J2: staged position trials; planned peak {speed:g} deg/s')
        axes[0].set_ylabel('Angle (deg)'); axes[1].set_ylabel('Plan - encoder (deg)')
        axes[2].set_ylabel('Encoder speed (deg/s)'); axes[2].set_xlabel('Time from outbound start (s)')
        for ax in axes:
            ax.grid(alpha=.3); ax.legend(ncol=3)
        fig.tight_layout(); fig.savefig(session/f'comparison_{speed:g}dps.png', dpi=160); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path); parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    criteria = json.loads((args.session/'protocol.json').read_text())['criteria']
    observation_file = args.session/'operator_observations.json'
    observations = json.loads(observation_file.read_text()) if observation_file.exists() else []
    summaries = [summarize(path, criteria, observations) for path in sorted(args.session.glob('real_*/report.json'), key=lambda p: p.stat().st_mtime_ns)]
    (args.session/'comparison.json').write_text(json.dumps(summaries, indent=2)+'\n')
    if args.plot:
        plot_comparison(args.session, summaries)
    for r in summaries:
        print(r['run'], json.dumps({k: r.get(k) for k in ('passed', 'error', 'encoder_peak_speed_deg_s',
            'plan_error_max_deg', 'effort_peak_nm', 'tracking_limited_fraction', 'performance_target_met')}))


if __name__ == '__main__':
    main()
