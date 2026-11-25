"""Offline log review only. No ROS, serial, register access, or application edits."""
from pathlib import Path
import hashlib, html, json, os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/a6y-f169-mpl')
os.environ.setdefault('XDG_CACHE_HOME', '/tmp/a6y-f169-cache')
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
SOURCE = ROOT/'diagnostics/arm_teaching_reports/teaching_f1699253fb'
PREVIOUS = ROOT/'diagnostics/arm_teaching_reports/teaching_75eac1124d'
WRITE = ROOT/'diagnostics/teaching_j4_kp80_trial_20261125/1795621242294344529_apply/result.json'
DEG = 180/np.pi
HASHES = {}

def record(p):
    HASHES[str(p.relative_to(ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return p

def load(p):
    quality = json.loads(record(p/'step_quality.json').read_text())
    assert json.loads(record(p/'session.json').read_text())['mock'] is False
    a = pd.read_csv(record(p/'step_arms.csv')).copy()
    x = pd.read_csv(record(p/'step_tracking.csv'))
    a['t'] = a.control_time-quality['following_start_monotonic']
    x['t'] = x.time_monotonic-quality['following_start_monotonic']
    return quality, a, x

quality, arm, tracking = load(SOURCE)
old_quality, old_arm, old_tracking = load(PREVIOUS)
target = pd.read_csv(record(SOURCE/'step_targets.csv'))
target['mono'] = target.source_ros_time+target.receive_monotonic-target.receive_wall
t0 = quality['following_start_monotonic']
target['t'] = target.mono-t0
result = json.loads(record(SOURCE/'result.json').read_text())
record(SOURCE/'session.log')
record(SOURCE/'step_summary.json')
assert json.loads(record(WRITE).read_text())['status'] == 'applied_verified'
mapper = ROOT/'A6-Y-G/src/a6_y_teleop/a6_y_teleop/leader_follower_teleop.py'
assert hashlib.sha256(record(mapper).read_bytes()).hexdigest() == '515df7865584ab3f05f7b0074be0e1ac149d41e00f8b79c2771460d4f41b253f'

def rms(x): return float(np.sqrt(np.mean(np.square(x))))
def quantile(x):
    x = np.asarray(x); x = x[np.isfinite(x)]
    return dict(zip(('min', 'median', 'p95', 'max'), map(float, np.quantile(x, [0,.5,.95,1])))) if len(x) else None
def axis(x, role, joint): return x[(x.follower==role)&(x.joint==joint)]

rates = []
for role, d in arm.groupby('role'):
    dt = float(d.control_time.iloc[-1]-d.control_time.iloc[0])
    rates.append(dict(role=role, samples=len(d), loop_hz=(len(d)-1)/dt,
        command_hz=[float(d[f'command_count_j{j}'].iloc[-1]-d[f'command_count_j{j}'].iloc[0])/dt for j in range(1,7)],
        feedback_hz=[float(d[f'feedback_count_j{j}'].iloc[-1]-d[f'feedback_count_j{j}'].iloc[0])/dt for j in range(1,7)],
        overrun_increment=int(d.control_overrun_count.iloc[-1]-d.control_overrun_count.iloc[0]),
        max_control_period_ms=float(d.control_period.max()*1000),
        faults=int(sum((d[f'fault_j{j}']!=0).sum() for j in range(1,7))),
        invalid_position_samples=int(sum((d[f'position_valid_j{j}']!=1).sum() for j in range(1,7)))))

metrics, startup, peaks, holds = [], [], [], []
for label, x, a, moving, final in [
    ('KP54_previous', old_tracking, old_arm, (4,39), (40,44.3)),
    ('KP80_latest', tracking, arm, (3,90), (95,100))]:
    for role, d in x.groupby('follower'):
        for j, b in d.groupby('joint'):
            for phase, window in [('movement_interval', moving), ('final_hold', final)]:
                w = b[b.t.between(*window)].dropna()
                n = a[(a.role==role)&a.t.between(*window)]
                metrics.append(dict(capture=label, role=role, joint=int(j), phase=phase,
                    window_s=list(window), samples=len(w), teacher_ptp_deg=float(np.ptp(w.teacher_q)*DEG),
                    actual_ptp_deg=float(np.ptp(w.actual_q)*DEG), sent_error_rms_deg=rms(w.sent_error)*DEG,
                    teacher_error_rms_deg=rms(w.teacher_error)*DEG,
                    teacher_error_peak_deg=float(w.teacher_error.abs().max()*DEG),
                    sent_error_p95_deg=float(w.sent_error.abs().quantile(.95)*DEG),
                    limited_fraction=float((abs(w.safe_q-w.raw_q)>.001).mean()),
                    speed_abs_peak_deg_s=float(n[f'velocity_j{j}'].abs().max()*DEG),
                    torque_abs_peak_nm=float(n[f'effort_j{j}'].abs().max())))
        w = d[(d.joint==4)&d.t.between(.05,2)].dropna()
        e = (w.actual_q-w.raw_q)*DEG
        # Sustained to 2 s, rather than a possibly transient first crossing.
        outside = np.flatnonzero(abs(e.to_numpy())>.5)
        idx = outside[-1]+1 if len(outside) else 0
        settle = float(w.t.iloc[idx]) if idx<len(w) else None
        startup.append(dict(capture=label, role=role, window_s=[.05,2],
            initial_error_deg=float(e.iloc[0]), teacher_ptp_deg=float(np.ptp(w.teacher_q)*DEG),
            enter_and_remain_within_half_degree_until_2s=settle,
            overshoot_deg=max(0, float(-(e*np.sign(e.iloc[0])).min()))))

for role, d in tracking.groupby('follower'):
    w = d[(d.joint==4)&d.t.between(3,90)].dropna()
    row = w.loc[w.teacher_error.abs().idxmax()]
    peaks.append(dict(role=role, t_s=float(row.t), angles_deg={k:float(row[k]*DEG) for k in
        ('teacher_q','raw_q','safe_q','sent_q','actual_q','sent_error','teacher_error')}))
    # Whole-arm quiet input over 2 s; measure the final 1 s to exclude immediate catch-up.
    for end in np.arange(3,100.25,.25):
        w = d[d.t.between(end-2,end)]
        ptp = w.groupby('joint').teacher_q.agg(lambda v:np.ptp(v)*DEG)
        if len(ptp)!=6 or ptp.max()>.1: continue
        z = w[(w.joint==4)&(w.t>=end-1)]
        holds.append(dict(role=role, input_window_s=[float(end-2),float(end)],
            measured_window_s=[float(end-1),float(end)], max_teacher_axis_ptp_deg=float(ptp.max()),
            J4_actual_ptp_deg=float(np.ptp(z.actual_q)*DEG)))

# Changing 6-axis vectors only: host-side pipeline intervals, not CAN ACK latency.
links = []
for side in ('left','right'):
    teacher = arm[arm.role=='teaching_'+side]; follower = arm[arm.role=='motion_'+side]
    for kind in ('teacher_snapshot_to_raw','safe_to_native_send'):
        df = target[(target.follower=='motion_'+side)&(target.kind==('raw' if kind=='teacher_snapshot_to_raw' else 'safe'))]
        qt = df[[f'q_j{j}' for j in range(1,7)]].to_numpy(); tt = df.mono.to_numpy()
        if kind=='teacher_snapshot_to_raw':
            st = teacher.control_time.to_numpy(); sq = teacher[[f'position_j{j}' for j in range(1,7)]].to_numpy(); tol=2e-8
        else:
            st = follower[[f'send_complete_time_j{j}' for j in range(1,7)]].max(axis=1).to_numpy()
            sq = follower[[f'sent_position_j{j}' for j in range(1,7)]].to_numpy(); tol=1.5e-7
        assert np.all(np.diff(st)>=0)
        intervals=[]; eligible=0
        for k in range(1,len(qt)):
            if np.max(abs(qt[k]-qt[k-1]))<.00005 or not 3<=tt[k]-t0<=90: continue
            eligible+=1
            lo,hi=(tt[k]-.08,tt[k]) if kind=='teacher_snapshot_to_raw' else (tt[k],tt[k]+.08)
            ids=np.arange(np.searchsorted(st,lo),np.searchsorted(st,hi,side='right'))
            matched=ids[np.max(abs(sq[ids]-qt[k]),axis=1)<tol]
            if not len(matched): continue
            index=matched[-1] if kind=='teacher_snapshot_to_raw' else matched[0]
            intervals.append(abs(tt[k]-st[index])*1000)
        links.append(dict(side=side, kind=kind, eligible=eligible, matched=len(intervals),
            unmatched=eligible-len(intervals), interval_ms=quantile(intervals), tolerance_rad=tol,
            caveat='Latest matching teacher snapshot / first matching serial-accepted vector; quantized repeats can understate age. Not motor sampling or CAN ACK timestamps.'))

exit_info={}
for role,a in result['arms'].items():
    home_log=Path(a['directory'])/'native_home.log'
    log=record(home_log).read_text()
    exit_info[role]=dict(home_stage=a['home_stage'], home_elapsed_s=a['home_elapsed'],
        release_confirmed=a['release_confirmed'], close_failed=a['close_failed'],
        native_home_reached_logged='Reached arm/home' in log,
        native_home_trajectory='2 s hold + 20 s move')

notes = [
    'Only saved files were read. No application changes, hardware connection, parameter read/write, or motor motion occurred in this analysis.',
    'KP80 is based on the verified RAM write at 23:41 and the subsequent 23:52 test; the capture contains no during-run gain readback, and power cycling would invalidate that association.',
    'Startup comparison uses similar initial J4 gaps (about 10 and 21 degrees), nearly fixed teacher positions, and a common ±0.5 degree band sustained until t=2 s. It is system catch-up time, not motor internal loop time.',
    'Movement intervals differ (previous 4–39 s; latest 3–90 s) and contain intervening holds and different amplitudes/speeds. Aggregate RMS cannot establish a percentage gain improvement.',
    'Latest final hold is 95–100 s, after all six teacher joints have stopped. Each arm has small encoder-level residual; left J4 has up to about 0.50 degree PTP in an earlier 1 s quiet-input window.',
    'The original relative-to-follower step limiter permits about 4.30 degrees target lead on J4–J6. Rapid motion can therefore create a large teacher/raw-to-sent gap while sent-to-actual remains near that bound.',
    'Initial alignment has 12 missing teacher fields and 24 missing raw/safe fields; all analysis windows after 1 s are complete. Native cycles have no gaps. The generic summary flag is not evidence of CAN packet loss.',
    'Multiple manual movements and reversals do not support one global step-response fit or a unique PID recommendation.'
]
report=dict(session=SOURCE.name, previous_session=PREVIOUS.name, following_local='2026-11-25 23:52:02–23:53:42 +08:00',
    following_duration_s=quality['following_stop_monotonic']-t0, last_verified_write=str(WRITE),
    gain_readback_during_capture=False, motor_writes_this_analysis=0, quality=quality, native_rates=rates,
    observed_target_hz=result['actual_hz'], startup=startup, metrics=metrics, peaks=peaks,
    quiet_input_windows=holds, pipeline_links=links, exit=exit_info,
    missing_fields={k:int(v) for k,v in tracking.isna().sum().items() if v},
    missing_fields_after_1s=int(tracking[tracking.t>1].isna().sum().sum()), notes=notes, source_sha256=HASHES)
assert report['missing_fields_after_1s']==0
assert all(r['overrun_increment']==0 and r['faults']==0 and r['invalid_position_samples']==0 for r in rates)
(OUT/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),
    'axes.unicode_minus':False,'axes.spines.top':False,'axes.spines.right':False,'font.size':10})
colors={'teacher_q':'#2166ac','raw_q':'#26915c','sent_q':'#df8b18','actual_q':'#853e96'}
labels={'teacher_q':'示教实际','raw_q':'原始目标','sent_q':'底层下发目标','actual_q':'运动实际'}

fig,axs=plt.subplots(3,2,figsize=(13.5,10),layout='constrained')
for col,side in enumerate(('left','right')):
    role='motion_'+side; name=('左' if side=='left' else '右')+' J4'
    for x,label,color in [(old_tracking,'上一轮 KP54','#9c6e55'),(tracking,'本轮 KP80','#2166ac')]:
        w=axis(x,role,4); w=w[w.t.between(.05,2)].dropna()
        axs[0,col].plot(w.t,(w.actual_q-w.raw_q)*DEG,label=label,c=color,lw=1.5)
    axs[0,col].axhspan(-.5,.5,color='#26915c',alpha=.13,label='±0.5° 到位带')
    axs[0,col].set_title(name+' · 启动追齐：相近初始位置差',loc='left'); axs[0,col].set_ylabel('实际 − 原始目标（°）')
    window=(27,32) if side=='left' else (55,62)
    w=axis(tracking,role,4); w=w[w.t.between(*window)].dropna()
    for k in ('teacher_q','raw_q','sent_q','actual_q'):
        axs[1,col].plot(w.t,w[k]*DEG,c=colors[k],label=labels[k],lw=1.15,ls='--' if k=='raw_q' else '-')
    axs[1,col].set_title(name+' · 快速变向：目标限步与实际滞后',loc='left');axs[1,col].set_ylabel('位置（°）')
    w=axis(tracking,role,4); w=w[w.t.between(95,100)].dropna()
    axs[2,col].plot(w.t,(w.actual_q-w.raw_q)*DEG,c=colors['actual_q'],label='本轮静止误差',lw=1.1)
    axs[2,col].set_ylim(-.15,.15);axs[2,col].axhline(0,c='#777',ls=':',lw=.8)
    axs[2,col].set_title(name+' · 全臂停止后 95–100 s',loc='left');axs[2,col].set_ylabel('实际 − 原始目标（°）')
    for ax in axs[:,col]:ax.grid(alpha=.2);ax.legend(fontsize=8,loc='best');ax.set_xlabel('开始跟随后时间（s）')
fig.suptitle('J4：启动追随更快，末段保持稳定；快速大幅变向仍跟不上\n23:52 真机日志 · 本次仅离线分析 · 曲线未经平滑或时间平移',fontsize=14)
for ext in ('png','svg'):fig.savefig(OUT/f'j4_comparison.{ext}',dpi=150)
plt.close(fig)

for kind, filename, title in [(1,'01_teacher_vs_target','示教实际 → 原始目标 → 底层下发目标'),(2,'02_target_vs_actual','底层下发目标 → 运动实际')]:
    fig,axs=plt.subplots(6,2,figsize=(14,17),layout='constrained')
    for col,side in enumerate(('left','right')):
        role='motion_'+side; leader=arm[arm.role=='teaching_'+side]; follower=arm[arm.role==role]
        raw=target[(target.follower==role)&(target.kind=='raw')]
        for j in range(1,7):
            ax=axs[j-1,col]
            if kind==1:
                ax.plot(leader[f'feedback_receive_time_j{j}']-t0,leader[f'position_j{j}']*DEG,c=colors['teacher_q'],label=labels['teacher_q'],lw=1.3)
                ax.plot(raw.t,raw[f'q_j{j}']*DEG,c=colors['raw_q'],label=labels['raw_q'],lw=1,ls='--')
            ax.plot(follower[f'send_complete_time_j{j}']-t0,follower[f'sent_position_j{j}']*DEG,c=colors['sent_q'],label=labels['sent_q'],lw=1.15)
            if kind==2:ax.plot(follower[f'feedback_receive_time_j{j}']-t0,follower[f'position_j{j}']*DEG,c=colors['actual_q'],label=labels['actual_q'],lw=1)
            ax.set_title(('左' if side=='left' else '右')+f' J{j}',loc='left');ax.grid(alpha=.2)
            ax.set_xlabel('开始跟随后时间（s）');ax.set_ylabel('关节位置（°）');ax.set_xlim(0,100.4)
    handles,labs=axs[0,0].get_legend_handles_labels()
    fig.legend(handles,labs,loc='upper center',bbox_to_anchor=(.5,.963),ncol=3,frameon=False)
    fig.suptitle(title+'\n2026-11-25 23:52 · 250 Hz 底层 / 100 Hz 目标 · 全六轴',fontsize=15)
    fig.get_layout_engine().set(rect=(0,0,1,.93))
    fig.savefig(OUT/f'{filename}.png',dpi=150);plt.close(fig)

def metric(label,role,j,phase):
    return next(m for m in metrics if m['capture']==label and m['role']==role and m['joint']==j and m['phase']==phase)
def stat(label,role):return next(m for m in startup if m['capture']==label and m['role']==role)
rows=[]
for role in ('motion_left','motion_right'):
    s1,s2=stat('KP54_previous',role),stat('KP80_latest',role)
    a,b=metric('KP54_previous',role,4,'final_hold'),metric('KP80_latest',role,4,'final_hold')
    rows.append(f"<tr><td>{'左' if role.endswith('left') else '右'} J4</td><td>{s1['enter_and_remain_within_half_degree_until_2s']:.2f} → {s2['enter_and_remain_within_half_degree_until_2s']:.2f} s</td><td>{a['actual_ptp_deg']:.3f} → {b['actual_ptp_deg']:.3f}°</td></tr>")
axisrows=[]
for j in range(1,7):
    a,b=[metric('KP80_latest','motion_'+side,j,'movement_interval') for side in ('left','right')]
    axisrows.append(f"<tr><td>J{j}</td><td>{a['sent_error_rms_deg']:.2f} / {b['sent_error_rms_deg']:.2f}</td><td>{a['teacher_error_rms_deg']:.2f} / {b['teacher_error_rms_deg']:.2f}</td><td>{a['limited_fraction']*100:.1f}% / {b['limited_fraction']*100:.1f}%</td></tr>")
linkrows=[]
for l in links:
    q=l['interval_ms'];label='示教快照→原始目标' if l['kind']=='teacher_snapshot_to_raw' else '限步目标→底层串口写入'
    linkrows.append(f"<tr><td>{'左' if l['side']=='left' else '右'} · {label}</td><td>{q['median']:.2f} ms</td><td>{q['p95']:.2f} ms</td><td>{l['matched']} / {l['eligible']}</td></tr>")
document='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>J4 KP80 真机日志复核</title><style>
body{background:#101722;color:#e7edf5;font:16px/1.75 system-ui,sans-serif;max-width:1180px;margin:auto;padding:38px 24px}h1{font-size:30px;line-height:1.4}h2{font-size:22px;margin-top:38px}p{max-width:1060px}.muted{color:#aab9ca}.card{background:#1b2738;border-left:4px solid #6eb9de;padding:18px 24px;border-radius:7px}table{width:100%;border-collapse:collapse;margin:20px 0}th,td{text-align:left;border-bottom:1px solid #334358;padding:10px}th{color:#a5d8ee}img{width:100%;border-radius:8px;background:white;margin:12px 0}a{color:#91d2f0}code{font-size:.9em}details{margin:20px 0}summary{cursor:pointer;font-size:20px}li{margin:8px 0}
</style><h1>J4：启动追随更快，快速变向仍有滞后</h1><p class="muted">真机 2026-11-25 23:52:02–23:53:42 · 会话 f1699253fb · 与 23:13 的 KP54 记录对比<br>本次仅分析保存的日志；未修改程序，未连接或读写电机。</p>
<div class="card">这轮未再看到此前右 J4 持续放大的大幅静止振荡。两侧启动追齐更快；但右 J4 快速变向时，示教与实际的瞬时差值仍达 44.81°。目前更值得关注运动端响应与原程序限步的共同作用，没有证据支持继续提高通信频率。</div>
<h2>相近初始条件下的启动追齐</h2><table><tr><th>关节</th><th>进入并保持 ±0.5°：上轮 → 本轮</th><th>末段静止波动峰峰值：上轮 → 本轮</th></tr>STARTUP_ROWS</table>
<p>两轮启动时，左 J4 误差约 10°，右 J4 约 21°，示教位置基本静止。到位时间从开始跟随计，要求进入 ±0.5° 后保持到第 2 秒。这是整条跟随链路的追齐时间，不是电机内部闭环周期。末段静止窗口：上轮 40–44.3 s，本轮 95–100 s。</p>
<p>末段表现良好不代表所有姿态完全没有波动：按“六轴示教在连续 2 秒内各自变化不超过 0.1°”筛选，取最后 1 秒计算，本轮左 J4 最大峰峰值约 0.50°（38.25–39.25 s），右约 0.17°。未观察到此前持续放大的十几度振荡，但仍有小幅残余波动。</p>
<a href="j4_comparison.png"><img src="j4_comparison.png" alt="J4 启动追齐、快速变向和末段静止对比"></a>
<h2>快速变向时，差值在哪里产生</h2><p>右 J4 在 t=59.246 s：示教 −47.68°、原始目标 −47.46°、底层下发目标 −6.47°、实际 −2.87°。示教与原始目标接近，但原程序相对运动臂反馈的限步让下发目标只领先实际约 3.60°，最终示教与实际相差 44.81°。左 J4 最大示教误差约 29.09°。</p>
<p>J4–J6 每次目标允许领先跟随臂当前位置约 4.30°；运动臂来不及跟上时，限步持续作用，目标变成受约束的追赶过程。不能把总差值全部当成通信延迟，也不能把限步与电机动态简单分成互不相关的两个原因。</p>
<h2>本轮全六轴表现</h2><p class="muted">3–90 s，含其中的停顿。表内按“左 / 右”排列，误差单位为度。目标受限指 |原始目标−限步目标| &gt; 0.001 rad。</p><table><tr><th>轴</th><th>下发目标−实际 RMS</th><th>示教−实际 RMS</th><th>目标受限占比</th></tr>AXIS_ROWS</table>
<p>J1–J3 的下发目标跟随误差较小；剩余差距主要在 J4，J5 快速动作也有明显误差。两轮手动轨迹的幅度、速度及停顿比例不同，不能用整段 RMS 的下降百分比作为参数改善量。这里保留逐轴数据，避免只用末段静止表现判断效果。</p>
<h2>时序与数据完整性</h2><p>跟随 100.352 s；四条臂底层循环及每轴指令约 250 Hz，上位机目标约 99.99 Hz。跟随阶段没有新增控制超时计数、缺失周期、记录队列丢弃、位置无效或故障标志。</p>
<table><tr><th>主机链路匹配</th><th>中位数</th><th>P95</th><th>匹配 / 有效变化向量</th></tr>LINK_ROWS</table>
<p class="muted">仅匹配变化中的六轴向量，窗口为 80 ms。示教使用最近相同快照，底层使用最早相同串口写入；量化后重复值可能低估间隔。这些是主机观测时间，不是 CAN ACK、纯通信延迟或电机内部采样周期。</p>
<p>自动摘要的 sample_gaps_or_invalid_feedback 标记来自启动时各数据流尚未对齐：12 个示教字段空值，24 个原始/限步目标字段空值。第 1 秒之后没有空值；原生控制周期连续。整段包含多次往返，不能拟合成一个理想阶跃。</p>
<h2>启动、退出与参数证据</h2><p>启动未回零，运动臂直接追随示教位置。退出时四条臂均记录 Reached arm/home，原生轨迹为保持 2 秒、运动 20 秒；回零阶段约 22.1–22.5 秒结束，随后全部确认失能，无关闭错误。</p>
<p>本轮参数标签依据 23:41 的已核验写入：双运动臂 J4 KP_APR=80、KI_APR=0、KP_ASR=0.00372、KI_ASR=0.002，仅写 RAM。日志本身没有运行期间的增益回读；若写入后曾断电重启，不能据此确认运行时仍为 80。本次没有重新连接电机扫描。</p>
<details><summary>全六轴：示教实际与运动目标</summary><a href="01_teacher_vs_target.png"><img src="01_teacher_vs_target.png" alt="全六轴示教和目标曲线"></a></details>
<details><summary>全六轴：运动目标与实际位置</summary><a href="02_target_vs_actual.png"><img src="02_target_vs_actual.png" alt="全六轴目标和反馈曲线"></a></details>
<p class="muted">曲线未经平滑或人为时间平移；时间零点均为进入跟随状态。<a href="analysis.json">完整统计与源文件哈希</a> · <a href="analyze.py">离线复现脚本</a></p></html>'''
document=document.replace('STARTUP_ROWS',''.join(rows)).replace('AXIS_ROWS',''.join(axisrows)).replace('LINK_ROWS',''.join(linkrows))
(OUT/'report.html').write_text(document)
print(json.dumps(dict(startup=startup, peaks=peaks, pipeline=links, exit=exit_info, output=str(OUT)),ensure_ascii=False,indent=2))
