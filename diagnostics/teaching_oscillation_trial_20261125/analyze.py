"""Read-only analysis of the stationary-teacher oscillation capture."""
from pathlib import Path
import hashlib,json,os
os.environ.setdefault('MPLCONFIGDIR','/tmp/a6y-oscillation-mpl')
import numpy as np
import pandas as pd
from scipy.signal import find_peaks
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

OUT=Path(__file__).resolve().parent
WORKSPACE=OUT.parents[1]
SOURCE=WORKSPACE/'diagnostics/arm_teaching_reports/teaching_cc3c0a6bfe'
quality=json.loads((SOURCE/'step_quality.json').read_text())
t0=quality['following_start_monotonic']
arm=pd.read_csv(SOURCE/'step_arms.csv').copy()
tracking=pd.read_csv(SOURCE/'step_tracking.csv')
arm['t']=arm.control_time-t0
tracking['t']=tracking.time_monotonic-t0
rad=180/np.pi
metrics=[]
for follower,df in tracking.groupby('follower'):
    for joint,d in df.groupby('joint'):
        if joint>6:continue
        for begin,end in [(0.,1.),(1.,6.39),(3.,6.39)]:
            window=d[d.t.between(begin,end)]
            native=arm[(arm.role==follower)&arm.t.between(begin,end)]
            values={key:float(window[key].max()-window[key].min())*rad
                    for key in ('teacher_q','raw_q','safe_q','sent_q','actual_q')}
            metrics.append(dict(role=follower,joint=int(joint),window_s=[begin,end],
                peak_to_peak_deg=values,error_rms_deg=float(np.sqrt(np.mean(window.sent_error**2)))*rad,
                velocity_peak_deg_s=float(native[f'velocity_j{joint}'].abs().max())*rad,
                torque_peak_nm=float(native[f'effort_j{joint}'].abs().max())))
rates=[]
for role,d in arm.groupby('role'):
    dt=d.control_time.iloc[-1]-d.control_time.iloc[0]
    rates.append(dict(role=role,seconds=float(dt),
        loop_hz=float((d.control_cycle_count.iloc[-1]-d.control_cycle_count.iloc[0])/dt),
        command_hz=float((d.command_count_j1.iloc[-1]-d.command_count_j1.iloc[0])/dt),
        overrun_count=int(d.control_overrun_count.iloc[-1]-d.control_overrun_count.iloc[0])))
right=arm[(arm.role=='motion_right')&arm.t.between(3.,6.39)]
peaks,_=find_peaks(right.position_j4.to_numpy(),prominence=np.deg2rad(3),distance=10)
periods=np.diff(right.t.to_numpy()[peaks])
report=dict(session=SOURCE.name,following_duration_s=quality['following_stop_monotonic']-t0,
    quality=quality,rates=rates,metrics=metrics,
    right_j4_oscillation_period_s=float(np.median(periods)) if len(periods)>1 else None,
    interpretation=['Right J4 raw teacher target is stationary after 1 s while actual oscillation grows.',
        'Right J4 sent target starts oscillating after actual error exceeds the original mapper 0.075 rad step bound.',
        'This is motor response plus follower-dependent target limiting, not transport delay.',
        'Left J4 has a decaying transient with residual oscillation; right J3 also has smaller motion.',
        'Change J4 first; J3 coupling versus independent servo behavior is not resolved by this capture.'],
    source_sha256={n:hashlib.sha256((SOURCE/n).read_bytes()).hexdigest()
                  for n in ('step_arms.csv','step_tracking.csv','step_targets.csv','step_quality.json','session.log')})
(OUT/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),
    'axes.unicode_minus':False,'axes.spines.top':False,'axes.spines.right':False,'font.size':10})
fig,axes=plt.subplots(2,2,figsize=(13,7),sharex=True,layout='constrained')
for row,side in enumerate(('left','right')):
    d=tracking[(tracking.follower=='motion_'+side)&(tracking.joint==4)]
    title=('左' if side=='left' else '右')+'运动臂 J4'
    for key,label,color in [('teacher_q','示教实际','#2686b6'),('raw_q','原始目标','#39954c'),
                             ('sent_q','底层下发目标','#dc8e1b')]:
        axes[row,0].plot(d.t,d[key]*rad,label=label,color=color,lw=1.4,
                         linestyle='--' if key=='raw_q' else '-')
    axes[row,1].plot(d.t,d.sent_q*rad,label='底层下发目标',color='#dc8e1b',lw=1.3)
    axes[row,1].plot(d.t,d.actual_q*rad,label='运动实际',color='#a23a5b',lw=1.3)
    for col in range(2):
        axes[row,col].set_title(title+' · '+('目标链路' if col==0 else '闭环响应'),loc='left')
        axes[row,col].set_ylabel('位置（°）');axes[row,col].grid(alpha=.22)
        axes[row,col].legend(fontsize=9,loc='best');axes[row,col].set_xlim(0,6.4)
for ax in axes[-1,:]:ax.set_xlabel('开始跟随后时间（s）')
fig.suptitle('静止示教端下的 J4 振荡｜2026-11-25 23:02\n右侧原始目标不动；实际振荡增大后，下发目标也受原程序限步牵动',fontsize=14)
fig.savefig(OUT/'j4_oscillation.png',dpi=150)
fig.savefig(OUT/'j4_oscillation.svg')
print('Saved analysis.json and j4_oscillation.png; period:',report['right_j4_oscillation_period_s'])
