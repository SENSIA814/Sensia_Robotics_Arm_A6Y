#!/usr/bin/python3
"""Run the real controller in a mock-interface dynamical plant; never open hardware."""
import argparse
import csv
import hashlib
import json
import math
import re
import os
from pathlib import Path
import subprocess
from datetime import datetime, timezone


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=root / 'validation/joint_spring_strengthening')
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    binary = root / 'build/a6_y_controllers/test_a5_joint_impedance_controller'
    if not binary.is_file():
        raise SystemExit('Build a6_y_controllers with BUILD_TESTING enabled first.')
    env = dict(os.environ, ROS_DOMAIN_ID='194', ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(out / 'ros_logs'), A6Y_RETURN_CSV=str(out / 'trajectories.csv'),
               A6Y_LARGE_RETURN_CSV=str(out / 'large_push_trajectories.csv'))
    with (out / 'simulation.log').open('w') as log:
        result = subprocess.run([str(binary), '--gtest_filter=JointControllerTest.FrictionReleaseAtRecordedPose:JointControllerTest.RepeatedLargePushesWithStrongerResistance:JointControllerTest.SmallReleaseWithStrongerAssumedResistance'],
                                cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
    if result.returncode:
        raise SystemExit(f'Simulation failed; see {out / "simulation.log"}')
    log_text = (out / 'simulation.log').read_text()
    strong_small = [dict(joint=int(j), onset_1mrad_seconds=float(t), error_fraction_at_3s=float(f), final_error_deg=float(e))
        for j,t,f,e in re.findall(r'small_strong_friction joint=(\d+) onset_1mrad_s=([\deE.+-]+) fraction_at_3s=([\deE.+-]+) final_error_deg=([\deE.+-]+)', log_text)]
    assert len(strong_small) == 4 and all(c['error_fraction_at_3s'] < .5 for c in strong_small)
    small_peaks = {int(case): float(peak) for case, peak in re.findall(
        r'friction_release profile=configured case=(\d+)[^\n]*?peak_speed=([\deE.+-]+)', log_text)}
    large_peaks = {(int(case), int(round_)): float(peak) for case, round_, peak in re.findall(
        r'large_release case=(\d+)[^\n]*?round=(\d+)[^\n]*?peak_speed=([\deE.+-]+)', log_text)}
    def return_metrics(rows):
        axes = [j for j in range(1, 7) if abs(rows[0][f'error{j}']) > .01]
        fractions = [(r['time'], max(abs(r[f'error{j}'] / rows[0][f'error{j}']) for j in axes)) for r in rows]
        worst_after_three = max(f for t, f in fractions if t >= 3)
        outside = [t for t, f in fractions if f >= .5]
        return dict(max_error_fraction_after_3s=worst_after_three,
                    half_error_settling_seconds=(max(outside) + rows[0]['dt']) if outside else 0)
    groups = {}
    with (out / 'trajectories.csv').open() as stream:
        for row in csv.DictReader(stream):
            row = {key: value if key == 'profile' else float(value) for key, value in row.items()}
            groups.setdefault(int(row['case_id']), []).append(row)
    cases = []
    for case_id, rows in groups.items():
        first = rows[0]
        late = [r for r in rows if r['time'] >= 6.0]
        error = max(abs(r[f'error{j}']) for r in late for j in range(1, 7))
        speed = max(abs(r[f'speed{j}']) for r in late for j in range(1, 7))
        timing = return_metrics(rows)
        if first['profile'] == 'configured':
            assert timing['max_error_fraction_after_3s'] < .5, (case_id, timing)
            assert small_peaks[case_id] < 2.0, (case_id, small_peaks[case_id])
            assert error < math.radians(2), (case_id, error)
            assert speed < (.04 if first['quantized'] else .03), (case_id, speed)
        else:
            assert error > math.radians(5), (case_id, 'old-gain negative control no longer reproduces')
        cases.append(dict(case_id=case_id, **timing, **{k: first[k] for k in (
            'profile', 'pose', 'axis', 'dt', 'delay', 'friction_scale', 'mass_scale', 'quantized')},
            late_peak_error_deg=math.degrees(error), late_peak_speed_rad_s=speed,
            final_peak_error_deg=math.degrees(max(abs(rows[-1][f'error{j}']) for j in range(1, 7))),
            sampled_peak_speed_rad_s=max(abs(r[f'speed{j}']) for r in rows for j in range(1, 7)),
            peak_speed_rad_s=small_peaks.get(case_id) ))
    configured = [c for c in cases if c['profile'] == 'configured']
    assert len(configured) == 108 and len(cases) == 110
    large_groups = {}
    with (out / 'large_push_trajectories.csv').open() as stream:
        for row in csv.DictReader(stream):
            row = {key: float(value) for key, value in row.items()}
            large_groups.setdefault((int(row['case_id']), int(row['round'])), []).append(row)
    large_cases = []
    for key, rows in large_groups.items():
        late = [r for r in rows if r['time'] >= 16]
        error = max(abs(r[f'error{j}']) for r in late for j in range(1, 7))
        speed = max(abs(r[f'speed{j}']) for r in late for j in range(1, 7))
        assert error < math.radians(2) and speed < .03, (key, error, speed)
        outside = [r['time'] for r in rows if max(abs(r[f'error{j}']) for j in range(1, 7)) >= math.radians(2)]
        timing = return_metrics(rows)
        assert timing['max_error_fraction_after_3s'] < .5, (key, timing)
        assert large_peaks[key] < 3.5, (key, large_peaks[key])
        large_cases.append(dict(**timing, case_id=key[0], round=key[1], axis=rows[0]['axis'],
            feedback_delay_seconds=rows[0]['dt'] * rows[0]['delay'], friction_scale=rows[0]['friction_scale'],
            late_peak_error_deg=math.degrees(error), late_peak_speed_rad_s=speed,
            final_peak_error_deg=math.degrees(max(abs(rows[-1][f'error{j}']) for j in range(1, 7))),
            settling_to_2deg_seconds=(max(outside) + .02) if outside else 0,
            sampled_peak_speed_rad_s=max(abs(r[f'speed{j}']) for r in rows for j in range(1, 7)),
            peak_speed_rad_s=large_peaks[key]))
    assert len(large_cases) == 84
    sources = ['src/a6_y_controllers/config/a5_joint_impedance_controllers.yaml',
               'src/a6_y_controllers/src/A6_y_joint_impedance_controller.cpp',
               'src/a6_y_controllers/include/a6_y_controllers/A6_y_joint_impedance_controller.hpp',
               'src/a6_y_controllers/include/a6_y_controllers/bounded_joint_impedance.hpp',
               'src/a6_y_controllers/test/test_a5_joint_impedance_controller.cpp', 'src/A5/urdf/A5.urdf']
    report = dict(timestamp_utc=datetime.now(timezone.utc).isoformat(), passed=True,
        hardware_started=False, hardware_validated=False, controller='actual C++ plugin with mock hardware interfaces',
        configured_cases=len(configured), negative_control_cases=2,
        strong_small_release=dict(cases=strong_small, release_angle_deg=5,
            target_rad=[-.09747,.59987,.30842,.33169,.10586,.03147],
            assumed_coulomb_stiction_nm=[1.,1.6,2.2,.25,.25,.2],
            period_seconds=.02, feedback_delay_seconds=0,
            note='Four additional zero-initial-integral releases; same strong assumed friction as large-push test. Not friction identification. Onset means displacement above 0.001 rad.'),
        criteria=dict(late_window_seconds=[6, 8], max_angle_error_deg=2,
                      max_speed_rad_s=.03, quantized_max_speed_rad_s=.04,
                      by_3s_initially_displaced_axes_enter_and_remain_below_fraction=.5,
                      small_release_peak_speed_regression_limit_rad_s=2.0,
                      speed_peak_source='C++ 1 ms integration substeps (log)'),
        assumptions=dict(integration_step_seconds=.001, release_displacement_rad=.15,
            plant='independent 8-coordinate URDF; fixed fingers; full 6-axis inertia and Coriolis',
            gravity='ideal cancellation plus constant residual in sensitivity cases',
            residual_torque_nm='[r,-r,r,0.2r,-0.2r,0.1r], r=+0.1 or -0.1',
            coulomb_stiction_nm=[.55,.25,.85,.08,.10,.05], friction_scales=[0,.5,1,1.5],
            friction_identified_from_hardware=False, inertia_scales=[.7,1,1.3],
            viscous_damping_nms_rad=[0,.03,.05,.08],
            feedback_and_command_quantization='nearest-step approximation of configured MIT ranges',
            coverage='selected combinations, not the Cartesian product; see test scenario table'),
        worst_error_fraction_after_3s=max(c['max_error_fraction_after_3s'] for c in configured),
        worst_half_error_settling_seconds=max(c['half_error_settling_seconds'] for c in configured),
        worst_late_error_deg=max(c['late_peak_error_deg'] for c in configured),
        worst_late_speed_rad_s=max(c['late_peak_speed_rad_s'] for c in configured),
        source_sha256={p: hashlib.sha256((root / p).read_bytes()).hexdigest() for p in sources}, cases=cases,
        repeated_pushes=dict(cases=large_cases, count=len(large_cases),
            worst_error_fraction_after_3s=max(c['max_error_fraction_after_3s'] for c in large_cases),
            worst_half_error_settling_seconds=max(c['half_error_settling_seconds'] for c in large_cases),
            worst_late_error_deg=max(c['late_peak_error_deg'] for c in large_cases),
            worst_settling_to_2deg_seconds=max(c['settling_to_2deg_seconds'] for c in large_cases),
            maximum_transient_speed_rad_s=max(c['peak_speed_rad_s'] for c in large_cases),
            late_window_seconds=[16,20], controller_period_seconds=.02, peak_speed_regression_limit_rad_s=3.5,
            target_radians=[-.04520,.53769,.36183,.14019,.11387,.10853],
            release_deflections_rad=[.817,.25,.31,.48,.475,.4044],
            scenario='six individual axes and all six together; opposite-side hold for 5 s, release for 20 s, twice in opposite directions',
            assumed_coulomb_stiction_nm=[1.,1.6,2.2,.25,.25,.20], friction_scales=[0,1,1.25],
            no_friction_case_has_zero_viscous_damping=True,
            note='Stronger resistance is a sensitivity hypothesis consistent with observed stalling, not identified motor friction. No speed-limit guarantee.'))
    (out / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    os.environ.setdefault('MPLCONFIGDIR', str(out / 'matplotlib_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.4), layout='constrained')
    for plot, axis in zip(axes[:2], [1, 5]):
        for profile, color in [('baseline', '#a83c32'), ('configured', '#16756b')]:
            rows = next(rows for rows in groups.values() if rows[0]['profile'] == profile
                        and rows[0]['pose'] == 0 and rows[0]['axis'] == axis and rows[0]['dt'] == .02
                        and rows[0]['delay'] == 0 and rows[0]['friction_scale'] == 1)
            plot.plot([r['time'] for r in rows], [math.degrees(r[f'error{axis}']) for r in rows],
                      color=color, label=profile, linewidth=2)
        plot.axvline(3, color='#555555', linestyle=':', label='3 s')
        plot.axhspan(-2, 2, alpha=.08, color='#16756b')
        plot.set(title=f'Joint {axis}: release from 8.59 deg', xlabel='Time after release (s)', ylabel='Angle error (deg)')
        plot.grid(alpha=.2); plot.legend()
    maxima = [max(c['late_peak_error_deg'] for c in configured if c['axis'] == j) for j in range(1, 7)]
    axes[2].bar([f'J{j}' for j in range(1, 7)], maxima, color='#16756b')
    axes[2].axhline(2, color='#a83c32', linestyle='--', label='2 deg criterion')
    axes[2].set(title='Worst residual across 108 cases', ylabel='Peak error, 6-8 s (deg)', xlabel='Initially displaced joint')
    axes[2].set_ylim(0, 2.4); axes[2].legend(); axes[2].grid(axis='y', alpha=.2)
    fig.suptitle('Joint return simulation | assumed friction, not hardware identification', fontsize=13)
    fig.savefig(out / 'comparison.png', dpi=180)
    fig.savefig(out / 'comparison.svg')
    large_fig, large_axes = plt.subplots(1, 3, figsize=(14, 4.4), layout='constrained')
    for plot, axis in zip(large_axes[:2], [3, 6]):
        for round_index, style in [(0, '-'), (1, '--')]:
            rows = next(rows for (case_id, round_number), rows in large_groups.items()
                        if round_number == round_index and rows[0]['axis'] == axis
                        and rows[0]['delay'] == 1 and rows[0]['friction_scale'] == 1)
            plot.plot([r['time'] for r in rows], [math.degrees(r[f'error{axis}']) for r in rows],
                      linestyle=style, linewidth=2, label=f'Release {round_index + 1}')
        plot.axvline(3, color='#555555', linestyle=':', label='3 s')
        plot.axhspan(-2, 2, alpha=.08, color='#16756b')
        plot.set(title=f'Joint {axis}: repeated opposite pushes', xlabel='Time after release (s)', ylabel='Angle error (deg)')
        plot.grid(alpha=.2); plot.legend()
    maxima = [max(c['late_peak_error_deg'] for c in large_cases if c['axis'] == j) for j in range(1,8)]
    large_axes[2].bar([f'J{j}' for j in range(1,7)] + ['All'], maxima, color='#16756b')
    large_axes[2].axhline(2, color='#a83c32', linestyle='--', label='2 deg criterion')
    large_axes[2].set(title='Worst residual across 84 releases', ylabel='Peak error, 16-20 s (deg)', ylim=(0,2.4))
    large_axes[2].legend(); large_axes[2].grid(axis='y', alpha=.2)
    large_fig.suptitle('Large-push recovery | stronger assumed resistance; 20 ms feedback delay', fontsize=13)
    large_fig.savefig(out / 'large_push_recovery.png', dpi=180)
    large_fig.savefig(out / 'large_push_recovery.svg')
    print(json.dumps({'large_push_cases': len(large_cases),
                      'worst_large_push_residual_deg': report['repeated_pushes']['worst_late_error_deg'],
                      'worst_large_push_settling_seconds': report['repeated_pushes']['worst_settling_to_2deg_seconds']}, indent=2))
    print(json.dumps({k: report[k] for k in ('passed', 'configured_cases', 'worst_late_error_deg',
                                           'worst_late_speed_rad_s', 'hardware_validated')}, indent=2))
    print(f'Artifacts: {out}')


if __name__ == '__main__':
    main()
