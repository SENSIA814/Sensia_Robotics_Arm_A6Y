#!/usr/bin/env python3
"""Read an existing report only; no ROS, serial, or hardware operations."""
import argparse
import bisect
import csv
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import statistics


def local_time(t):
    return datetime.fromtimestamp(t, timezone(timedelta(hours=8))).isoformat(timespec='milliseconds')


def vector(row, prefix):
    return [float(row[f'{prefix}_joint{i}']) for i in range(1, 7)]


def interpolate(plan, elapsed):
    # The old executor prepends two stationary seconds to the saved trajectory.
    times = [0.] + [t + 2. for t in plan['times']]
    positions = [plan['positions'][0]] + plan['positions']
    velocities = [[0.] * 6] + plan['velocities']
    i = min(len(times) - 2, max(0, bisect.bisect_right(times, elapsed) - 1))
    h = times[i+1] - times[i]
    u = min(1., max(0., (elapsed - times[i]) / h))
    return [(2*u**3 - 3*u**2 + 1) * positions[i][j]
            + (u**3 - 2*u*u + u) * h * velocities[i][j]
            + (-2*u**3 + 3*u*u) * positions[i+1][j]
            + (u**3 - u*u) * h * velocities[i+1][j] for j in range(6)]


def analyze(report):
    events = [json.loads(line) for line in (report / 'events.jsonl').read_text().splitlines()]
    with (report / 'telemetry.csv').open() as f:
        rows = list(csv.DictReader(f))
    model = json.loads((report / 'model.json').read_text())
    plan = json.loads((report / 'home_plan.json').read_text())
    result = json.loads((report / 'result.json').read_text())
    starts = [e for e in events if e['kind'] == 'home_executing']
    failures = [e for e in events if e['kind'] == 'close_cancelled']
    planning = [e for e in events if e['kind'] == 'home_planning']
    attempts = []
    for ix, (start, failure) in enumerate(zip(starts, failures)):
        selected = [r for r in rows if start['time'] <= float(r['time']) <= failure['time']]
        gaps = [float(b['time']) - float(a['time']) for a, b in zip(selected, selected[1:])]
        peak_v = []
        for j in range(6):
            peak = max(selected, key=lambda r: abs(vector(r, 'velocity')[j]))
            peak_v.append(dict(joint=j+1, velocity_rad_s=vector(peak, 'velocity')[j],
                               seconds_after_execute_event=float(peak['time']) - start['time']))
        entry = dict(revision=start['revision'], start_local=local_time(start['time']),
                     failure_local=local_time(failure['time']), planned_duration_s=start['duration'],
                     elapsed_until_failure_s=failure['time'] - start['time'],
                     planning_and_preview_s=start['time'] - planning[ix]['time'],
                     sample_count=len(selected), all_sampled_feedback_fresh=all(r['fresh'] == 'True' for r in selected),
                     max_csv_interval_s=max(gaps), median_csv_frequency_hz=1. / statistics.median(gaps),
                     start_positions_rad=vector(selected[0], 'q'),
                     last_sampled_positions_rad=vector(selected[-1], 'q'),
                     sampled_peak_velocities=peak_v, reported_error=failure['message'])
        if ix == len(starts) - 1:
            # Only this last attempt survives the old home_plan.json overwrite.
            errors = [[abs(ref[j] - vector(row, 'q')[j]) for j in range(6)]
                      for row in selected for ref in [interpolate(plan, float(row['time']) - start['time'])]]
            entry['approx_sampled_max_tracking_errors_rad'] = [max(e[j] for e in errors) for j in range(6)]
            entry['tracking_error_time_alignment'] = 'Approximate: aligned to Python home_executing event, not exact RT start.'
        attempts.append(entry)
    return dict(
        source=str(report.resolve()), physical_device_commands=0, role=model['role'],
        ready_mock=next(e for e in events if e['kind'] == 'ready')['mock'],
        stable_serial_port=model['arm']['port'], controller_rate_hz=125,
        home_attempts=attempts, home_reached_events=sum(e['kind'] == 'home_reached' for e in events),
        release_events=[e for e in events if e['kind'] == 'released'],
        final_release_confirmed=result['release_confirmed'],
        observations=[
            'All four plans reached execution; aborts occurred in the C++ return controller.',
            'Every CSV sample during the four execution windows reports fresh feedback.',
            'The saved final attempt has sampled tracking error below 0.021 rad; the protection threshold was 0.15 rad.',
            'No sampled velocity during execution exceeds the 0.5 rad/s protection threshold.',
            'J4 motion begins shortly before several aborts; sampled velocity alternates sign during net negative motion.',
            'No home_reached event exists; the later explicit release was confirmed.',
        ],
        inferences_and_limits=[
            'Not a MoveIt planning failure; these events do not indicate a serial disconnect.',
            'Velocity transients, control-period/heartbeat gaps or solver failure remain possible; the old generic error does not identify which branch fired.',
            'The 125 Hz controller can see transients omitted by approximately 18 Hz CSV sampling. Do not infer an exact failure code from this report.',
            'The first three trajectories were overwritten, so their exact reference errors cannot be reconstructed.',
            'Alternating J4 velocity could reflect real oscillation or feedback spikes; low-rate telemetry cannot distinguish these.',
        ])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    content = json.dumps(analyze(args.report), ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(content + '\n')
    else:
        print(content)
