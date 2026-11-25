#!/usr/bin/env python3
"""Summarize J2 trials and export six-axis traces; no hardware access."""
import argparse
import csv
import json
import math
from pathlib import Path
import re


def plot_comparison(session, summaries):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for duration in (12.,6.):
        group=[r for r in summaries if r['passed'] and r['start_degrees']==0. and
               r['end_degrees']==30. and r['move_seconds']==duration]
        if not group:continue
        group.sort(key=lambda r:(r['kp_apr'],r['run']))
        fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
        for index,r in enumerate(group):
            rows=[json.loads(line) for line in (session/r['run']/'trace.jsonl').read_text().splitlines()]
            rows=[x for x in rows if x.get('phase') in ('outbound','outbound_settle','outbound_hold')]
            t=np.array([(x['t_ns']-rows[0]['t_ns'])*1e-9 for x in rows])
            plan,actual=[np.rad2deg([x[k] for x in rows]) for k in ('plan','actual')]
            if index==0:axes[0].plot(t,plan,'k--',label='Plan')
            label=f"KP_APR={r['kp_apr']}"+(' repeat' if 'repeat' in r['run'] else '')
            axes[0].plot(t,actual,label=label);axes[1].plot(t,plan-actual,label=label)
        axes[0].set_title(f'J2: 0 to 30 degrees in {duration:g} seconds; five other arm joints hold')
        axes[0].set_ylabel('Angle (deg)');axes[1].set_ylabel('Plan - encoder (deg)')
        axes[1].set_xlabel('Time from outbound start (s)')
        for ax in axes:ax.grid(alpha=.3)
        axes[0].legend(ncol=3,loc='lower right')
        axes[1].legend(ncol=1,loc='upper left')
        fig.tight_layout();fig.savefig(session/f'comparison_{duration:g}s.png',dpi=170);plt.close(fig)


def summarize(path):
    report=json.loads(path.read_text())
    m=report.get('metrics', {})
    result={k:report.get(k) for k in ('mode','passed','disarm_confirmed','error','start_degrees','end_degrees','move_seconds','planned_peak_speed_deg_s')}
    result.update(m)
    result['run']=path.parent.name
    gain=re.search(r'_kp([0-9]+)_',path.parent.name)
    result['kp_apr']=int(gain[1]) if gain else None
    holds=m.get('holds', {})
    result['smooth_trial']=bool(report.get('passed') and report.get('disarm_confirmed') and
        len(holds)==2 and m.get('overshoot_deg',float('inf'))<=.3 and
        all(h['span_deg']<=.2 and abs(h['mean_error_deg'])<=.2 for h in holds.values()))
    result['tracking_target_met']=result['smooth_trial'] and m.get('plan_error_max_deg',float('inf'))<=2.
    with (path.parent/'trace.csv').open('w') as out:
        writer=csv.writer(out)
        writer.writerow(['time_s','phase']+[f'{key}_j{i}_{unit}' for key,unit in [('plan','deg'),('sent','deg'),('actual','deg'),('effort','Nm')] for i in range(1,7)])
        origin=None
        with (path.parent/'trace.jsonl').open() as source:
            for line in source:
                row=json.loads(line)
                if 'phase' not in row:continue
                if origin is None:origin=row['t_ns']
                values=[(row['t_ns']-origin)*1e-9,row['phase']]
                for key in ('plan_all','reference_all','actual_all'):
                    values.extend(math.degrees(v) for v in row[key][:6])
                values.extend(row['hardware'].get(f'joint{i}',{}).get('effort') for i in range(1,7))
                writer.writerow(values)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('session',type=Path)
    p.add_argument('--plot',action='store_true')
    args=p.parse_args()
    summaries=[summarize(path) for path in sorted(args.session.glob('real_*/report.json'))]
    (args.session/'comparison.json').write_text(json.dumps(summaries,indent=2)+'\n')
    if args.plot:plot_comparison(args.session,summaries)
    for r in summaries:
        print(r['run'],json.dumps({k:r.get(k) for k in ('passed','error','plan_error_max_deg','tracking_limited_fraction','max_other_joint_motion_deg','effort_peak_nm','smooth_trial','tracking_target_met')}))


if __name__=='__main__':main()
