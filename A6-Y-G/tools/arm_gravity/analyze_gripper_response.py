#!/usr/bin/env python3
"""Offline J7 timing/torque analysis; never imports or opens a motor driver.

Best-shift lag is a waveform-fit metric, NOT a CAN/transport delay. Motor torque
is decoded from the prismatic effort with the configured joint position scale.
It is neither a force-sensor reading nor calibrated jaw/contact force.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def lag_fit(t, source, response, maximum):
    # Fixed common window for every candidate avoids shrinking-window bias.
    dt = float(np.median(np.diff(t)))
    n = int(maximum / dt)
    end = len(t) - n
    if end <= n:
        return None
    errors = [np.mean((source[n:end] - response[n+k:end+k])**2) for k in range(n+1)]
    k = int(np.argmin(errors))
    return {'lag_s': k*dt, 'shifted_rmse_percent': float(np.sqrt(errors[k])),
            'unshifted_rmse_percent': float(np.sqrt(errors[0])), 'search_max_s': maximum}


def zoh(t, times, values):
    idx = np.searchsorted(times, t, side='right') - 1
    return np.asarray(values)[np.maximum(0, idx)]


def analyze(folder, output):
    output.mkdir(parents=True, exist_ok=True)
    session = json.loads((folder/'session.json').read_text())
    quality = json.loads((folder/'step_quality.json').read_text())
    arms = pd.read_csv(folder/'step_arms.csv', usecols=lambda c: c in
        ('role', 'control_time', 'receive_monotonic') or c.endswith('_j7'))
    targets = pd.read_csv(folder/'step_targets.csv')
    # Source ROS time -> steady time using source/callback offset from the same
    # messages; observer callback delay remains a small uncertainty. Hardware
    # control/send/receive timestamps below are already on the steady clock.
    clock_offset = float(np.median(targets.receive_monotonic-targets.source_ros_time))
    targets['t'] = targets.source_ros_time + clock_offset
    start = float(quality['following_start_monotonic'])
    stats = dict(session=str(folder.resolve()), quality=quality, motor_torque_note=
        'effort_j7 * position_scale / sign; not calibrated contact force',
        source_clock_conversion_uncertainty_p95_s=float(np.quantile(np.abs(
            targets.receive_monotonic-targets.source_ros_time-clock_offset), .95)),
        pairs={}, evidence_sha256={})
    for name in ('session.json','step_arms.csv','step_targets.csv','step_quality.json'):
        stats['evidence_sha256'][name] = hashlib.sha256((folder/name).read_bytes()).hexdigest()
    fig, axes = plt.subplots(4, 2, figsize=(16, 12), sharex='col')
    streams = {}
    for col, (leader, follower) in enumerate(session['pairs']):
        l = arms[arms.role == leader].sort_values('control_time')
        f = arms[arms.role == follower].sort_values('control_time')
        raw = targets[(targets.follower == follower) & (targets.kind == 'raw')].sort_values('t')
        safe = targets[(targets.follower == follower) & (targets.kind == 'safe')].sort_values('t')
        profile = session['gripper_profiles'][follower]
        lp = session['gripper_profiles'][leader]
        span = profile['open_position_m'] - profile['closed_position_m']
        def pct(q):
            return (np.asarray(q)-profile['closed_position_m']) / span * 100
        # Recover calibrated linear scale from independent raw/model endpoints.
        motor_scale = (profile['measured_open_position_m']-profile['measured_closed_position_m']) / (
            profile['measured_open_motor_rad']-profile['measured_closed_motor_rad'])
        leader_scale = (lp['measured_open_position_m']-lp['measured_closed_position_m']) / (
            lp['measured_open_motor_rad']-lp['measured_closed_motor_rad'])
        t = np.arange(max(l.control_time.min(), f.control_time.min(), safe.t.min(), raw.t.min()),
                      min(l.control_time.max(), f.control_time.max(), safe.t.max(), raw.t.max()), .01)
        r = np.clip(zoh(t, raw.t, pct(raw.q_j7)), 0, 100)
        s = zoh(t, safe.t, pct(safe.q_j7))
        # Sent position in each native snapshot belongs to the recorded send time.
        sent = f.drop_duplicates('send_begin_time_j7').sort_values('send_begin_time_j7')
        cmd = zoh(t, sent.send_begin_time_j7, pct(sent.sent_position_j7))
        actual = np.interp(t, f.control_time, pct(f.position_j7))
        tau = f.effort_j7.to_numpy()*motor_scale
        moving = np.abs(f.motor_velocity_j7.to_numpy()) > .1
        changes = l.feedback_count_j7.diff().fillna(0) > 0
        feedback_times = l.loc[changes, 'control_time'].to_numpy()
        pdiff = np.diff(safe.q_j7.to_numpy())/np.diff(safe.t.to_numpy())
        info = dict(duration_s=float(t[-1]-t[0]), teacher_enabled_values=l.enabled_j7.unique().tolist(),
            teacher_command_counts=l.command_count_j7.unique().tolist(),
            teacher_feedback_interval_median_s=float(np.median(np.diff(feedback_times))),
            teacher_feedback_age_p95_s=float(l.feedback_age_j7.quantile(.95)),
            full_stroke_at_configured_limit_s=abs(span)/profile['max_velocity_m_s'],
            commanded_motor_speed_rad_s=f.sent_velocity_limit_j7.unique().tolist(),
            raw_to_safe=lag_fit(t, r, s, 2.), raw_to_feedback=lag_fit(t, r, actual, 2.),
            sent_to_feedback=lag_fit(t, cmd, actual, .6),
            raw_to_feedback_abs_error_p95_percent=float(np.quantile(np.abs(r-actual), .95)),
            sent_to_feedback_abs_error_p95_percent=float(np.quantile(np.abs(cmd-actual), .95)),
            raw_safe_differ_over_2_percent_time_fraction=float(np.mean(np.abs(r-s)>2)),
            safe_speed_abs_p95_model_m_s=float(np.quantile(np.abs(pdiff), .95)),
            max_abs_motor_torque_nm=float(np.max(np.abs(tau))),
            moving_abs_torque_p95_nm=float(np.quantile(np.abs(tau[moving]), .95)),
            stopped_abs_torque_p95_nm=float(np.quantile(np.abs(tau[~moving]), .95)))
        stats['pairs'][follower] = info
        streams[follower] = (t-start, r, s, cmd, actual)
        ax = axes[0,col]
        for y,label,color in [(r,'Mapped teacher (%)','#444444'),(s,'Rate-limited target','#e69f00'),
                              (actual,'Follower feedback','#0072b2')]:
            ax.plot(t-start,y,label=label,color=color,lw=1.3)
        ax.set_title(f'{leader} -> {follower}')
        ax.set_ylabel('Opening (%)')
        ax.legend(loc='upper right',fontsize=8)
        axes[1,col].plot(t-start,r-s,label='Teacher - limited target',color='#e69f00')
        axes[1,col].plot(t-start,cmd-actual,label='Sent - actual',color='#0072b2')
        axes[1,col].set_ylabel('Error (percentage points)')
        axes[1,col].legend(fontsize=8)
        axes[2,col].plot(l.control_time-start,l.motor_velocity_j7,label='Teacher',lw=.9,color='#444444')
        axes[2,col].plot(f.control_time-start,f.motor_velocity_j7,label='Follower',lw=.9,color='#0072b2')
        limit=float(f.sent_velocity_limit_j7.iloc[0])
        for v in [-limit,limit]:
            axes[2,col].axhline(v,ls='--',color='#e69f00',lw=.8)
        axes[2,col].set_ylabel('Motor speed (rad/s)')
        axes[2,col].legend(fontsize=8)
        axes[3,col].plot(f.control_time-start,tau,label='Follower motor torque',color='#009e73',lw=.8)
        axes[3,col].plot(l.control_time-start,l.effort_j7*leader_scale,label='Teacher J7 telemetry',
                         color='#888888',alpha=.7,lw=.6)
        axes[3,col].set_ylabel('Reported motor torque (Nm)')
        axes[3,col].legend(fontsize=8)
        axes[3,col].set_xlabel('Time after following start (s)')
        for a in axes[:,col]:
            a.grid(alpha=.25)
            a.axhline(0,color='#888888',lw=.5)
    fig.suptitle('Real J7 follow log | 100 Hz targets, 250 Hz motor control\n'
                 'Motor-angle/model displacement mapping; no calibrated contact-force measurement',fontsize=13)
    fig.tight_layout(rect=(0,0,1,.95))
    fig.savefig(output/'gripper_full.png',dpi=160)
    plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(13,7),sharex=True)
    # Show the first large repeated closing/opening section from the recording.
    for ax,(role,(t,r,s,cmd,actual)) in zip(axes,streams.items()):
        for y,label,color in [(r,'Mapped teacher','#444444'),(s,'Rate-limited target','#e69f00'),
                              (cmd,'Sent target','#cc79a7'),(actual,'Actual follower','#0072b2')]:
            ax.plot(t,y,label=label,color=color,lw=1.6)
        ax.set(title=role,ylabel='Opening (%)',xlim=(20,32),ylim=(-5,105))
        ax.grid(alpha=.25);ax.legend(ncol=4,fontsize=9)
    axes[-1].set_xlabel('Time after following start (s)')
    fig.tight_layout();fig.savefig(output/'gripper_detail.png',dpi=160);plt.close(fig)
    (output/'analysis.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(stats['pairs'],ensure_ascii=False,indent=2))
    return stats


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    analyze(args.session,args.output)
