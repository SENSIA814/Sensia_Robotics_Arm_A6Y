#!/usr/bin/env python3
"""Compare recorded J1 trials; this script never connects to a robot."""
import argparse
import json
from pathlib import Path
import re

import numpy as np


def analyze(directory):
    report = json.loads((directory / 'report.json').read_text())
    rows = [json.loads(line) for line in (directory / 'trace.jsonl').read_text().splitlines()]
    rows = [r for r in rows if 'phase' in r and r['phase'] not in ('initial', 'prepare', 'disarm')]
    if not rows:
        return {'run': directory.name, 'passed': False, 'error': report.get('error', 'no motion samples')}
    t = np.array([r['t_ns'] for r in rows], dtype=np.float64) * 1e-9
    actual, plan, sent = [np.array([r[key] for r in rows]) for key in ('actual', 'plan', 'reference')]
    phases = np.array([r['phase'] for r in rows])
    moving = np.isin(phases, ['outbound', 'return'])
    if not moving.any():
        return {'run': directory.name, 'passed': False, 'error': report.get('error', 'no outbound/return samples')}
    total_error = np.rad2deg(plan - actual)
    motor_error = np.rad2deg(sent - actual)
    anchor = report['anchor_positions'][0]
    delta = np.deg2rad(report['delta_degrees'])
    sign = np.sign(delta)
    holds = {}
    for name, target in [('outbound_hold', anchor + delta), ('return_hold', anchor)]:
        indices = np.flatnonzero(phases == name)
        if not len(indices):
            continue
        indices = indices[t[indices] >= t[indices[-1]] - 1.]
        q = actual[indices]
        holds[name] = dict(span_deg=float(np.rad2deg(np.ptp(q))),
                           std_deg=float(np.rad2deg(np.std(q))),
                           mean_error_deg=float(np.rad2deg(np.mean(q) - target)))
    outgoing = np.isin(phases, ['outbound', 'outbound_settle', 'outbound_hold'])
    returning = np.isin(phases, ['return', 'return_settle', 'return_hold'])
    overshoot = max(0., float(np.max(sign * (actual[outgoing] - anchor - delta))) if outgoing.any() else 0.,
                    float(np.max(-sign * (actual[returning] - anchor))) if returning.any() else 0.)
    # Centered encoder difference avoids treating quantized velocity feedback
    # spikes as sustainable speed; require both endpoints inside a moving phase.
    window = .2
    velocity = (np.interp(t + window/2, t, actual) - np.interp(t - window/2, t, actual)) / window
    interiors = moving.copy()
    for name in ['outbound', 'return']:
        idx = np.flatnonzero(phases == name)
        if len(idx):
            interiors[idx] &= (t[idx] >= t[idx[0]] + window/2) & (t[idx] <= t[idx[-1]] - window/2)
    effort = np.array([r.get('j1_hardware', {}).get('effort', np.nan) for r in rows])
    gains = interiors & (np.abs(sent - actual) > .012) & (np.abs(velocity) > .02)
    relative = np.rad2deg(np.abs(actual - anchor))
    result = dict(run=directory.name, passed=report['passed'], disarm_confirmed=report['disarm_confirmed'],
                  move_seconds=report['move_seconds'], planned_peak_speed_deg_s=report['planned_peak_speed_deg_s'],
                  plan_error_max_deg=float(np.max(np.abs(total_error))),
                  plan_error_rms_deg=float(np.sqrt(np.mean(total_error[moving]**2))),
                  plan_error_p95_deg=float(np.percentile(np.abs(total_error[moving]), 95)),
                  motor_error_max_deg=float(np.max(np.abs(motor_error))),
                  motor_error_rms_deg=float(np.sqrt(np.mean(motor_error[moving]**2))),
                  tracking_limited_fraction=report.get('metrics', {}).get('tracking_limited_fraction'),
                  encoder_peak_speed_deg_s=float(np.max(np.abs(np.rad2deg(velocity[interiors])))),
                  overshoot_deg=float(np.rad2deg(overshoot)),
                  holds=holds, actual_travel_deg=float(np.max(relative)),
                  final_error_deg=report.get('metrics', {}).get('final_error_from_start_deg'),
                  other_joint_motion_deg=report.get('metrics', {}).get('max_other_joint_motion_deg'))
    if np.any(gains):
        result['effective_velocity_per_position_error_s_inv'] = float(np.median(velocity[gains] / (sent[gains] - actual[gains])))
    if np.any(np.isfinite(effort)):
        result['effort_peak_nm'] = float(np.nanmax(np.abs(effort)))
        result['effort_rms_nm'] = float(np.sqrt(np.nanmean(effort**2)))
    # Explicit trial criteria, not a manufacturer specification or a proof
    # of stability at untested payloads, gains, speeds, and robot postures.
    result['smooth_trial'] = bool(report['passed'] and report['disarm_confirmed'] and
        len(holds) == 2 and max(h['span_deg'] for h in holds.values()) <= .2 and
        result['overshoot_deg'] <= .3 and abs(result['final_error_deg']) <= .2)
    result['tracking_target_met'] = result['smooth_trial'] and result['plan_error_max_deg'] <= 2.
    return result


def plot_trials(session, summaries):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for duration in sorted({s['move_seconds'] for s in summaries if s.get('passed')}):
        group = [s for s in summaries if s.get('passed') and s['move_seconds'] == duration]
        group.sort(key=lambda s: int(re.search(r'kp(\d+)', s['run'])[1]))
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        for i, summary in enumerate(group):
            directory = session / summary['run']
            report = json.loads((directory / 'report.json').read_text())
            rows = [json.loads(line) for line in (directory / 'trace.jsonl').read_text().splitlines()]
            rows = [r for r in rows if r.get('phase') in ('outbound', 'outbound_settle', 'outbound_hold')]
            t = np.array([(r['t_ns'] - rows[0]['t_ns']) * 1e-9 for r in rows])
            q, plan = [np.rad2deg([r[k] - report['anchor_positions'][0] for r in rows]) for k in ('actual', 'plan')]
            gain = re.search(r'kp(\d+)', summary['run'])[1]
            label = f"KP_APR={gain}" + (' repeat' if 'repeat' in summary['run'] else '')
            if i == 0:
                axes[0].plot(t, plan, 'k--', linewidth=2, label='Planned trajectory')
            axes[0].plot(t, q, label=label)
            axes[1].plot(t, plan - q, label=label)
        axes[0].set_ylabel('J1 relative angle (deg)')
        axes[0].set_title(f'J1 hardware tuning: 30 deg outbound in {duration:g} seconds')
        axes[1].set_ylabel('Plan - encoder (deg)')
        axes[1].set_xlabel('Time from outbound start (s)')
        for ax in axes:
            ax.grid(alpha=.25)
            ax.legend(ncol=3, fontsize=8)
        fig.tight_layout()
        fig.savefig(session / f'comparison_{duration:g}s.png', dpi=170)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--require-tracking', action='store_true', help='Return nonzero if any trial misses the 2 degree target.')
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    summaries = [analyze(path.parent) for path in sorted(args.session.glob('*/report.json'))]
    (args.session / 'comparison.json').write_text(json.dumps(summaries, indent=2) + '\n')
    if args.plot:
        plot_trials(args.session, summaries)
    print('run,peak_plan_error_deg,rms_plan_error_deg,limited_pct,encoder_peak_deg_s,overshoot_deg,hold_span_deg,smooth,target')
    for s in summaries:
        if 'plan_error_max_deg' not in s:
            print(s)
            continue
        hold = max(h['span_deg'] for h in s['holds'].values()) if s['holds'] else float('nan')
        print(f"{s['run']},{s['plan_error_max_deg']:.4f},{s['plan_error_rms_deg']:.4f},"
              f"{100*s['tracking_limited_fraction']:.2f},{s['encoder_peak_speed_deg_s']:.3f},"
              f"{s['overshoot_deg']:.4f},{hold:.4f},{s['smooth_trial']},{s['tracking_target_met']}")
    return int(args.require_tracking and (not summaries or not all(s.get('tracking_target_met', False) for s in summaries)))


if __name__ == '__main__':
    raise SystemExit(main())
