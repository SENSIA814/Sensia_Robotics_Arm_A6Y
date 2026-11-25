"""Read-only analysis of the two real teaching logs; does not import motor/ROS code."""
from pathlib import Path
import os,json,hashlib
import numpy as np
import pandas as pd
import yaml
from scipy.ndimage import gaussian_filter1d
os.environ.setdefault('MPLCONFIGDIR','/tmp/a6y-tracking-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
FONT_PATH='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(FONT_PATH)
FONT_FAMILY=font_manager.FontProperties(fname=FONT_PATH).get_name()

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
SESSIONS=['teaching_2b6bab1c45','teaching_ac84a69ae6']
ALL={};METRICS=[];PEAKS=[];TIMING=[];VELOCITIES=[];PROVENANCE={}

def describe(x):
    x=np.asarray(x);x=x[np.isfinite(x)]
    return dict(rms=float(np.sqrt(np.mean(x*x))),mae=float(np.mean(abs(x))),p95=float(np.percentile(abs(x),95)),maximum=float(np.max(abs(x))))

def estimate_lag(t,x,y):
    """Effective position alignment delay, NOT a transport-delay estimate.

    Interpolate on 10ms grid, mildly smooth both signals (30ms sigma), fit
    y(t) against x(t-lag) on one common overlap for every candidate lag.
    Mask motion of either signal. Reject unexcited / poorly fitted signals.
    """
    grid=np.arange(t[0],t[-1],.01)
    x=gaussian_filter1d(np.interp(grid,t,x),3)
    y=gaussian_filter1d(np.interp(grid,t,y),3)
    if np.ptp(x)<.08:return None
    moving=(abs(np.gradient(x,.01))>.05)|(abs(np.gradient(y,.01))>.05)
    valid=moving & (grid>=grid[0]+2) & (grid<=grid[-1]-2)
    if np.sum(valid)<100:return None
    lags=np.arange(-.3,2.001,.01)
    losses=[np.mean((np.interp(grid[valid]-lag,grid,x)-y[valid])**2) for lag in lags]
    k=int(np.argmin(losses));best=float(lags[k]);shift=np.interp(grid[valid]-best,grid,x)
    return dict(seconds=round(best,3),rmse_before=float(np.sqrt(np.mean((x[valid]-y[valid])**2))),
                rmse_after=float(np.sqrt(losses[k])),correlation=float(np.corrcoef(shift,y[valid])[0,1]),
                moving_seconds=float(valid.sum()*.01),at_boundary=k in (0,len(lags)-1))

for session in SESSIONS:
    folder=ROOT/'diagnostics/arm_teaching_reports'/session
    meta=json.loads((folder/'session.json').read_text());result=json.loads((folder/'result.json').read_text())
    events=[json.loads(s) for s in (folder/'events.jsonl').read_text().splitlines()]
    start=next(e['time'] for e in events if e.get('phase')=='following')
    end=next(e['time'] for e in events if e['kind']=='stop_requested')
    hz=int(meta['frequency']);allrefs=pd.read_csv(folder/'references.csv')
    statuses=[e.get('status') for e in events if e['kind']=='upstream_status' and start<e['time']<end]
    ALL[hz]=dict(start=start,end=end,session=session,folder=folder,pairs={})
    PROVENANCE[session]={'frequency':hz,'following_seconds':end-start,'reported_hz':result['actual_hz'],
        'source_hashes':{},'source_manifest':str(folder/'software.json'),'status_changes':statuses,'result_error':result['error']}
    for filename in ('session.json','result.json','events.jsonl','references.csv','software.json'):
        PROVENANCE[session]['source_hashes'][str(folder/filename)]=hashlib.sha256((folder/filename).read_bytes()).hexdigest()
    for role in ('motion_left','motion_right'):
        cfg=yaml.safe_load((folder/(role+'_teleop.yaml')).read_text())
        data=allrefs[(allrefs.follower==role)&(allrefs.wall_time>=start+1)&(allrefs.wall_time<end)].copy()
        data['t']=data.wall_time-start
        t=data.t.to_numpy();dt=np.diff(t)
        TIMING.append(dict(hz=hz,role=role,count=len(data),observer_hz=(len(data)-1)/(t[-1]-t[0]),
            interval_median_ms=np.median(dt)*1000,interval_p95_ms=np.percentile(dt,95)*1000,
            interval_p99_ms=np.percentile(dt,99)*1000,interval_max_ms=max(dt)*1000,
            observer_counter_gaps=int(np.sum(np.maximum(0,np.diff(data.sequence)-1)))))
        armfolder=Path(result['arms'][role]['directory']);telemetry=pd.read_csv(armfolder/'telemetry.csv')
        telemetry=telemetry[(telemetry.time>=start+1)&(telemetry.time<end)].copy();telemetry['t']=telemetry.time-start
        leader=role.replace('motion','teaching');leaderfolder=Path(result['arms'][leader]['directory'])
        leadertelemetry=pd.read_csv(leaderfolder/'telemetry.csv')
        leadertelemetry=leadertelemetry[(leadertelemetry.time>=start+1)&(leadertelemetry.time<end)].copy()
        for path in (armfolder/'telemetry.csv',armfolder/'control_controller_manager_1.yaml',folder/(role+'_teleop.yaml'),leaderfolder/'telemetry.csv'):
            PROVENANCE[session]['source_hashes'][str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        ALL[hz]['pairs'][role]=dict(refs=data,telemetry=telemetry,config=cfg)
        for j in range(1,7):
            raw=data[f'raw_j{j}'].to_numpy();safe=data[f'safe_j{j}'].to_numpy();actual=data[f'follower_q_j{j}'].to_numpy();teacher=data[f'leader_q_j{j}'].to_numpy()
            low=cfg['limits']['min_position'][j-1]+cfg['limits']['limit_margin'][j-1]
            high=cfg['limits']['max_position'][j-1]-cfg['limits']['limit_margin'][j-1]
            clipped=np.clip(raw,low,high)
            step=min(cfg['limits']['max_joint_step'],cfg['limits']['max_velocity'][j-1]/hz*1.5)
            expected=np.clip(actual+np.clip(clipped-actual,-step,step),low,high)
            # Observer stores latest raw, latest feedback, and received safe;
            # these are asynchronous streams, so do NOT assert exact tuple sync.
            dynamic=abs(clipped-safe)>.005
            grid=np.arange(t[0],t[-1],.01)
            gr=gaussian_filter1d(np.interp(grid,t,raw),3)
            moving=np.interp(t,grid,(abs(np.gradient(gr,.01))>.05).astype(float))>.5
            item=dict(hz=hz,role=role,joint=j,raw_range=np.ptp(raw),step_limit=step,
                motion_seconds=float(np.mean(moving)*(t[-1]-t[0])),samples=len(t),
                bound_clipped_percent=float(np.mean(abs(raw-clipped)>.001)*100),
                dynamic_limited_percent=float(np.mean(dynamic)*100),
                dynamic_limited_moving_percent=float(np.mean(dynamic[moving])*100) if moving.any() else None,
                raw_safe=describe(raw-safe),safe_actual=describe(safe-actual),raw_actual=describe(raw-actual),
                teacher_raw=describe(teacher-raw),reconstruction_residual=describe(expected-safe),
                moving_raw_safe=describe((raw-safe)[moving]) if moving.any() else None,
                moving_safe_actual=describe((safe-actual)[moving]) if moving.any() else None,
                moving_raw_actual=describe((raw-actual)[moving]) if moving.any() else None,
                lag_raw_actual=estimate_lag(t,raw,actual),lag_safe_actual=estimate_lag(t,safe,actual))
            METRICS.append(item)
            k=int(np.argmax(abs(raw-actual)))
            PEAKS.append(dict(hz=hz,role=role,joint=j,time_s=float(t[k]),teacher=float(teacher[k]),raw=float(raw[k]),safe=float(safe[k]),actual=float(actual[k]),
                raw_safe=float(raw[k]-safe[k]),safe_actual=float(safe[k]-actual[k]),raw_actual=float(raw[k]-actual[k])))
            v=telemetry[f'velocity_joint{j}'].to_numpy();vl=leadertelemetry[f'velocity_joint{j}'].to_numpy()
            tq=telemetry[f'q_joint{j}'].to_numpy();tt=telemetry.t.to_numpy();ts=np.interp(tt,t,safe)
            error=ts-tq;aligned=(abs(error)>.015)&(v*error>0)&(abs(v)>.03)
            VELOCITIES.append(dict(hz=hz,role=role,joint=j,follower_velocity_p95=float(np.percentile(abs(v),95)),
                follower_velocity_max=float(np.max(abs(v))),teacher_velocity_p95=float(np.percentile(abs(vl),95)),
                teacher_velocity_max=float(np.max(abs(vl))),median_v_over_error=float(np.median(abs(v[aligned]/error[aligned]))) if aligned.any() else None))

for name,records in [('metrics',METRICS),('peaks',PEAKS),('timing',TIMING),('velocities',VELOCITIES)]:
    (OUT/(name+'.json')).write_text(json.dumps(records,ensure_ascii=False,indent=2))
    pd.json_normalize(records).to_csv(OUT/(name+'.csv'),index=False)
(OUT/'provenance.json').write_text(json.dumps(PROVENANCE,ensure_ascii=False,indent=2))

plt.rcParams.update({'font.family':FONT_FAMILY,'axes.unicode_minus':False,'font.size':10,'figure.facecolor':'#fff','axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
colors=['#147d8c','#e78017','#202c48']
fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained')
for ax,(hz,role,j) in zip(axes.flat,[(50,'motion_right',3),(100,'motion_right',3),(50,'motion_right',2),(100,'motion_right',2)]):
    data=ALL[hz]['pairs'][role]['refs'];peak=next(p for p in PEAKS if (p['hz'],p['role'],p['joint'])==(hz,role,j))
    center=peak['time_s'];view=data[(data.t>center-3)&(data.t<center+4)]
    for key,label,color in zip(('raw','safe','follower_q'),('示教目标 raw','发布目标 safe（ROS 层）','运动臂反馈 actual'),colors):
        ax.plot(view.t,view[f'{key}_j{j}'],label=label,color=color,lw=1.8)
    ax.axvline(center,c='#86909a',lw=.8,ls='--');ax.set_title(f'{hz} Hz · 右臂 J{j} · 最大偏差附近')
    ax.set_xlabel('进入跟随后的时间 / s');ax.set_ylabel('关节角 / rad');ax.grid(alpha=.18)
    ax.text(.02,.98,f"最大总偏差 {abs(peak['raw_actual']):.3f} rad\n其中 raw→safe {abs(peak['raw_safe']):.3f}，safe→actual {abs(peak['safe_actual']):.3f}",
            transform=ax.transAxes,va='top',fontsize=9,bbox=dict(facecolor='white',alpha=.85,edgecolor='none'))
axes[0,0].legend(loc='lower left',fontsize=9)
fig.suptitle('跟随滞后分段：示教目标 → 限制后的目标 → 实际反馈',fontsize=16)
fig.savefig(OUT/'tracking_comparison.png',dpi=160);plt.close(fig)

fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True,layout='constrained')
hz=100;role='motion_right';j=3;data=ALL[hz]['pairs'][role]['refs'];tel=ALL[hz]['pairs'][role]['telemetry']
ax=axes[0]
ax.plot(data.t,data.raw_j3-data.follower_q_j3,label='示教目标 − 实际',c=colors[0]);ax.plot(data.t,data.safe_j3-data.follower_q_j3,label='发布目标 − 实际',c=colors[1])
for val in (-.1125,.1125):ax.axhline(val,c='#9d354c',ls='--',lw=1)
ax.text(32,.13,'±0.1125 rad 目标距离限制',ha='right',c='#9d354c');ax.legend(loc='upper left');ax.set_ylabel('位置差 / rad');ax.grid(alpha=.18)
ax=axes[1];ax.plot(tel.t,tel.velocity_joint3,c=colors[2],label='运动臂 J3 反馈速度');ax.set_xlabel('进入跟随后的时间 / s');ax.set_ylabel('速度 / rad/s');ax.legend(loc='upper left');ax.grid(alpha=.18)
fig.suptitle('100 Hz · 右臂 J3：目标距离被限制，运动臂仍需追赶该目标',fontsize=14)
fig.savefig(OUT/'right_j3_100hz_limits.png',dpi=160);plt.close(fig)
print(pd.DataFrame(TIMING).to_string(index=False))
for m in METRICS:
 if m['role']=='motion_right' and m['joint'] in (2,3):
  print(m['hz'],m['role'],m['joint'],'limited moving%',m['dynamic_limited_moving_percent'],'lag raw',m['lag_raw_actual'],'lag safe',m['lag_safe_actual'])
print(pd.DataFrame([p for p in PEAKS if p['role']=='motion_right' and p['joint'] in (2,3)]).to_string(index=False))
print(pd.DataFrame([v for v in VELOCITIES if v['role']=='motion_right' and v['joint'] in (2,3)]).to_string(index=False))
