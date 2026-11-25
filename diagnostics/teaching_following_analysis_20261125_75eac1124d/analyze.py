"""Compare the J4 retry capture to the previous oscillation; no control writes."""
from pathlib import Path
import hashlib,json,os
os.environ.setdefault('MPLCONFIGDIR','/tmp/a6y-follow-retry-mpl')
os.environ.setdefault('XDG_CACHE_HOME','/tmp/a6y-follow-retry-cache')
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
REPORTS=ROOT/'diagnostics/arm_teaching_reports'
SOURCE=REPORTS/'teaching_75eac1124d'
OLD=REPORTS/'teaching_cc3c0a6bfe'
RAD=180/np.pi

def load(folder):
    quality=json.loads((folder/'step_quality.json').read_text())
    session=json.loads((folder/'session.json').read_text())
    assert session['mock'] is False
    a=pd.read_csv(folder/'step_arms.csv').copy()
    x=pd.read_csv(folder/'step_tracking.csv')
    a['t']=a.control_time-quality['following_start_monotonic']
    x['t']=x.time_monotonic-quality['following_start_monotonic']
    return quality,a,x

quality,arm,tracking=load(SOURCE)
old_quality,old_arm,old_tracking=load(OLD)
metrics=[]
for role,df in tracking.groupby('follower'):
    for joint,d in df.groupby('joint'):
        if joint>6:continue
        for phase,begin,end in [('startup_hold',2.,3.),('moving',4.,39.),('coupled_motion',23.5,24.5),('final_hold',40.,44.3)]:
            w=d[d.t.between(begin,end)].dropna(subset=['teacher_q','raw_q','safe_q','sent_q','actual_q'])
            native=arm[(arm.role==role)&arm.t.between(begin,end)]
            metrics.append(dict(role=role,joint=int(joint),phase=phase,window_s=[begin,end],samples=len(w),
                teacher_ptp_deg=float(w.teacher_q.max()-w.teacher_q.min())*RAD,
                actual_ptp_deg=float(w.actual_q.max()-w.actual_q.min())*RAD,
                sent_error_rms_deg=float(np.sqrt(np.mean(w.sent_error**2)))*RAD,
                sent_error_abs_p95_deg=float(w.sent_error.abs().quantile(.95))*RAD,
                teacher_error_rms_deg=float(np.sqrt(np.mean(w.teacher_error**2)))*RAD,
                teacher_error_abs_peak_deg=float(w.teacher_error.abs().max())*RAD,
                target_limited_fraction=float((abs(w.safe_q-w.raw_q)>.001).mean()),
                velocity_abs_peak_deg_s=float(native[f'velocity_j{joint}'].abs().max())*RAD,
                torque_abs_peak_nm=float(native[f'effort_j{joint}'].abs().max())))

scan_path=OUT/'motion_parameters_1795619762463837085.json'
scan=json.loads(scan_path.read_text())
applied_path=ROOT/'diagnostics/teaching_oscillation_trial_20261125/1795619266101254063_apply/result.json'
applied=json.loads(applied_path.read_text())
assert scan['complete'] and applied['status']=='applied_verified'
parameters={}
for role,a in scan['arms'].items():
    assert a['complete']
    for m in a['motors']:
        assert m['state_before']==m['state_after']==0
        assert m['raw']==applied['arms'][role]['after'][str(m['motor_id'])]['raw']
    parameters[role]={f'J{m["motor_id"]}':{key:m['parameters'][key] for key in ('KP_APR','KI_APR','KP_ASR','KI_ASR')}
                      for m in a['motors']}
rates=[]
for role,d in arm.groupby('role'):
    dt=d.control_time.iloc[-1]-d.control_time.iloc[0]
    rates.append(dict(role=role,samples=len(d),loop_hz=float((d.control_cycle_count.iloc[-1]-d.control_cycle_count.iloc[0])/dt),
        command_hz=float((d.command_count_j1.iloc[-1]-d.command_count_j1.iloc[0])/dt),
        overruns=int(d.control_overrun_count.iloc[-1]-d.control_overrun_count.iloc[0])))
summary=json.loads((SOURCE/'step_summary.json').read_text())
report=dict(session=SOURCE.name,mock=False,following_duration_s=quality['following_stop_monotonic']-quality['following_start_monotonic'],
    previous_session=OLD.name,quality=quality,rates=rates,metrics=metrics,parameters_after_capture=parameters,
    parameter_verification=dict(scan=str(scan_path),all_14_disabled=True,all_24_registers_per_motor_match_last_write=True,
        note='Post-run readback matches prior write; this is not a register capture during motion.',motor_writes_sent=0),
    interpretation=['No growing J4 oscillation in observed stationary sections; final holds are stable over this capture.',
        'J4 still has greater dynamic error than J1-J3. The original follower-dependent step limiter also clips fast targets.',
        'Left J4 actual motion spans 1.333 deg during 23.5-24.5 s while its raw target spans 0.087 deg; other joints are moving, so this is not a whole-arm stationary stability test.',
        'Right J3 stationary residual shrank without changing its gains; consistent with J4 coupling, not proof of causation.',
        'This capture contains multiple movements/reversals and is unsuitable for fitting one step response or exact settling time.'],
    flagged_summary_rows=[r for r in summary['results'] if r.get('flags')],
    alignment_note='Six initial right-arm rows have no aligned teacher sample. Windows used for metrics begin at 2 s; native cycles have no gaps.',
    proposed_next_test=dict(joint='J4',roles=['motion_left','motion_right'],KP_APR=80.,KI_APR=0.,KP_ASR=.00372,KI_ASR=.002,
        applied=False,rationale='Modest position gain increase from 54 with velocity gains fixed; no stability guarantee.'),
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        [SOURCE/'step_arms.csv',SOURCE/'step_tracking.csv',SOURCE/'step_quality.json',SOURCE/'session.log',
         OLD/'step_tracking.csv',OLD/'step_quality.json',scan_path,applied_path]})
(OUT/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')

font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),
    'axes.unicode_minus':False,'axes.spines.top':False,'axes.spines.right':False,'font.size':10})
fig,axes=plt.subplots(3,2,figsize=(13,10),layout='constrained')
for col,side in enumerate(('left','right')):
    role='motion_'+side;name=('左' if side=='left' else '右')+' J4'
    new=tracking[(tracking.follower==role)&(tracking.joint==4)]
    old=old_tracking[(old_tracking.follower==role)&(old_tracking.joint==4)]
    for d,label,color in [(old,'上一轮参数','#a43e56'),(new,'本轮参数','#147fa3')]:
        w=d[d.t.between(0,3)]
        axes[0,col].plot(w.t,(w.actual_q-w.raw_q)*RAD,label=label,color=color,lw=1.2)
    axes[0,col].axhline(0,color='#777',lw=.8,ls=':')
    axes[0,col].set_title(name+' · 启动追赶及静止（前 3 秒）',loc='left')
    axes[0,col].set_ylabel('实际 − 原始目标（°）')
    w=new[new.t.between(4,39)]
    for key,label,color in [('raw_q','示教原始目标','#278652'),('sent_q','底层下发目标','#e29322'),('actual_q','运动实际','#147fa3')]:
        axes[1,col].plot(w.t,w[key]*RAD,label=label,color=color,lw=1.1)
    axes[1,col].set_title(name+' · 本轮运动段',loc='left');axes[1,col].set_ylabel('位置（°）')
    w=new[new.t.between(40,44.3)]
    axes[2,col].plot(w.t,(w.actual_q-w.raw_q)*RAD,label='本轮静止误差',color='#147fa3',lw=1.1)
    axes[2,col].set_ylim(-.25,.25);axes[2,col].axhline(0,color='#777',lw=.8,ls=':')
    axes[2,col].set_title(name+' · 本轮末段静止',loc='left');axes[2,col].set_ylabel('实际 − 原始目标（°）')
    for ax in axes[:,col]:ax.grid(alpha=.22);ax.legend(fontsize=9);ax.set_xlabel('开始跟随后时间（s）')
fig.suptitle('J4 新参数：静止振荡已明显改善，快速跟随仍偏慢\n真机 23:13 · KP_APR=54 / KP_ASR=0.00372 / KI_ASR=0.002 · 曲线未经平滑或时间平移',fontsize=14)
fig.savefig(OUT/'j4_retry_comparison.png',dpi=150)
fig.savefig(OUT/'j4_retry_comparison.svg')
print('Saved analysis.json and j4_retry_comparison.png; current parameters verified read-only')
