"""Read-only analysis of teaching_394b0c668f; no ROS, serial, or control writes."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib,json,os
os.environ.setdefault('MPLCONFIGDIR','/tmp/a6y-follow-analysis-mpl')
os.environ.setdefault('XDG_CACHE_HOME','/tmp/a6y-follow-analysis-cache')
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
SOURCE=ROOT/'diagnostics/arm_teaching_reports/teaching_394b0c668f'
SCAN=ROOT/'diagnostics/teaching_step_logging_20261125/motor_parameters.json'
HASHES={}
def record(path):
    HASHES[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    return path
quality=json.loads(record(SOURCE/'step_quality.json').read_text())
session=json.loads(record(SOURCE/'session.json').read_text())
result=json.loads(record(SOURCE/'result.json').read_text())
original_summary=json.loads(record(SOURCE/'step_summary.json').read_text())
record(SOURCE/'software.json')
mapper=ROOT/'A6-Y-G/src/a6_y_teleop/a6_y_teleop/leader_follower_teleop.py'
record(mapper)
assert HASHES[str(mapper)]=='515df7865584ab3f05f7b0074be0e1ac149d41e00f8b79c2771460d4f41b253f'
for side in ('left','right'):record(SOURCE/f'motion_{side}_teleop.yaml')
scan=json.loads(record(SCAN).read_text())
arm=pd.read_csv(record(SOURCE/'step_arms.csv')).copy()
target=pd.read_csv(record(SOURCE/'step_targets.csv'))
tracking=pd.read_csv(record(SOURCE/'step_tracking.csv'))
t0=quality['following_start_monotonic']
tracking['t']=tracking.time_monotonic-t0
arm['t']=arm.control_time-t0
target['mono']=target.source_ros_time+target.receive_monotonic-target.receive_wall
target['t']=target.mono-t0
names={'left':'左臂','right':'右臂'}
colors=dict(teacher='#0077b6',raw='#31975a',sent='#e68b13',actual='#8c3c9a')

def quantile(x):
    x=np.asarray(x);x=x[np.isfinite(x)]
    return {k:float(v) for k,v in zip(('min','median','p95','max'),np.quantile(x,[0,.5,.95,1]))} if len(x) else None

rates=[]
for role,d in arm.groupby('role'):
    dt=d.control_time.iloc[-1]-d.control_time.iloc[0]
    rates.append(dict(role=role,samples=len(d),seconds=dt,loop_hz=(d.control_cycle_count.iloc[-1]-d.control_cycle_count.iloc[0])/dt,
        command_hz=[(d[f'command_count_j{j}'].iloc[-1]-d[f'command_count_j{j}'].iloc[0])/dt for j in range(1,7)],
        feedback_hz=[(d[f'feedback_count_j{j}'].iloc[-1]-d[f'feedback_count_j{j}'].iloc[0])/dt for j in range(1,7)],
        new_cycles_over_6ms=float(d.control_overrun_count.iloc[-1]-d.control_overrun_count.iloc[0]),
        faults=int(sum((d[f'fault_j{j}']!=0).sum() for j in range(1,7))),
        invalid_positions=int(sum((d[f'position_valid_j{j}']!=1).sum() for j in range(1,7)))))

# Match CHANGING six-axis vectors. Repeated stationary targets carry no unique
# timing information. Bounds and coordinate precision are explicit; these are
# host-side pipeline measurements, never CAN ACK / motor sampling timestamps.
links=[]
for side in ('left','right'):
    teacher=arm[arm.role=='teaching_'+side]
    follower=arm[arm.role=='motion_'+side]
    raw=target[(target.follower=='motion_'+side)&(target.kind=='raw')]
    safe=target[(target.follower=='motion_'+side)&(target.kind=='safe')]
    lq=teacher[[f'position_j{i}' for i in range(1,7)]].to_numpy();lt=teacher.control_time.to_numpy()
    sq=follower[[f'sent_position_j{i}' for i in range(1,7)]].to_numpy()
    st=follower[[f'send_complete_time_j{i}' for i in range(1,7)]].max(axis=1).to_numpy()
    for kind,df,src_t,src_q,tolerance in [('teacher_snapshot_to_raw',raw,lt,lq,2e-8),('safe_to_native_send',safe,st,sq,1.5e-7)]:
        qt=df[[f'q_j{i}' for i in range(1,7)]].to_numpy();tt=df.mono.to_numpy()
        ages=[];details=[];eligible=0;unmatched=0
        for k in range(1,len(qt)):
            if np.max(abs(qt[k]-qt[k-1]))<.00005 or tt[k]-t0<.3:continue
            eligible+=1
            lo,hi=(tt[k]-.08,tt[k]) if kind=='teacher_snapshot_to_raw' else (tt[k],tt[k]+.08)
            indices=np.arange(np.searchsorted(src_t,lo),np.searchsorted(src_t,hi,side='right'))
            matches=indices[np.max(abs(src_q[indices]-qt[k]),axis=1)<tolerance]
            if not len(matches):unmatched+=1;continue
            index=matches[-1] if kind=='teacher_snapshot_to_raw' else matches[0]
            age=tt[k]-src_t[index] if kind=='teacher_snapshot_to_raw' else src_t[index]-tt[k]
            ages.append(age*1000);details.append(dict(target_time_s=tt[k]-t0,matched_time_s=src_t[index]-t0,interval_ms=age*1000))
        pd.DataFrame(details).to_csv(OUT/(side+'_'+kind+'_matches.csv'),index=False)
        links.append(dict(side=side,kind=kind,eligible=eligible,matched=len(ages),unmatched=unmatched,interval_ms=quantile(ages),
            matching_tolerance_rad=tolerance,search_window_s=.08,
            note='Changing six-axis vectors; latest matching teacher snapshot / first matching native send; repeated quantized values can understate source age.'))

metrics=[]
for side in ('left','right'):
    follower=arm[arm.role=='motion_'+side]
    for joint in range(1,7):
        z=tracking[(tracking.follower=='motion_'+side)&(tracking.joint==joint)].dropna().copy()
        z=z[z.t>.2]
        i=int(np.argmax(abs(z.teacher_q-z.sent_q)));peak=z.iloc[i]
        row=dict(side=side,joint=joint,teacher_range_rad=float(np.ptp(z.teacher_q)),
            teacher_minus_sent_peak_rad=float(abs(peak.teacher_q-peak.sent_q)),peak_time_s=float(peak.t),
            at_peak={k:float(peak[k]) for k in ('teacher_q','raw_q','safe_q','sent_q','actual_q')},
            native_error_peak_rad=float(abs(z.sent_q-z.actual_q).max()),
            native_error_rms_rad=float(np.sqrt(np.mean((z.sent_q-z.actual_q)**2))),
            last_5s_native_error_rad=float((z.sent_q-z.actual_q)[z.t>27].median()))
        if joint in (1,2,3):
            # First outward movement has a common long plateau before 16 s.
            # Use the measured plateau, not a presumed ideal step starting at t=0.
            plateau=float(z[(z.t>=13)&(z.t<15.5)].teacher_q.median())
            band=max(.01,.02*abs(plateau))
            first=z[z.t<15.5]
            reached={}
            for key in ('teacher_q','raw_q','safe_q','sent_q','actual_q'):
                outside=np.flatnonzero(abs(first[key].to_numpy()-plateau)>band)
                idx=outside[-1]+1 if len(outside) else 0
                reached[key]=float(first.t.iloc[idx]) if idx<len(first) else None
            row.update(first_plateau_rad=plateau,settling_band_rad=band,first_plateau_entry_s=reached,
                after_teacher_hold_s=reached['actual_q']-reached['teacher_q'],
                after_native_target_enters_band_s=reached['actual_q']-reached['sent_q'],
                plateau_13_15s_native_error_rad=float((first.sent_q-first.actual_q)[(first.t>13)&(first.t<15)].median()))
        if joint in (2,3):
            ids=np.searchsorted(z.time_monotonic,follower.control_time,side='right')-1
            ids=np.clip(ids,0,len(z)-1)
            use=((follower.t>.5)&(follower.t<15)&
                (z.teacher_q.to_numpy()[ids]-follower[f'sent_position_j{joint}'].to_numpy()>.2)&
                (follower[f'error_j{joint}']>.09))
            v=follower.loc[use,f'velocity_j{joint}'].to_numpy();e=follower.loc[use,f'error_j{joint}'].to_numpy()
            row.update(saturated_ramp_samples=int(use.sum()),saturated_ramp_velocity_rad_s=quantile(v),
                saturated_ramp_native_error_rad=quantile(e),velocity_over_error_per_s=quantile(v/e))
        metrics.append(row)

summary=dict(session=SOURCE.name,started_local=datetime.fromtimestamp(float(target.receive_wall.iloc[0]),ZoneInfo('Asia/Shanghai')).isoformat(),
    following_seconds=quality['following_stop_monotonic']-t0,quality=quality,native_rates=rates,
    observed_target_hz=result['actual_hz'],links=links,joints=metrics,
    motor_parameter_snapshot=dict(observed_at=scan['observed_at'],source=str(SCAN),
        motion_left_KP_APR=[m['parameters']['KP_APR'] for m in scan['arms']['motion_left']['motors'][:6]],
        motion_right_KP_APR=[m['parameters']['KP_APR'] for m in scan['arms']['motion_right']['motors'][:6]]),
    conclusions=['主要延迟来自原程序相对运动臂当前位置的限步与运动端位置跟随响应的共同作用；不支持继续提通信/控制频率作为首要措施。',
        '前三轴目标只能领先跟随臂当前反馈0.1125 rad，后三轴0.075 rad；下发目标成为斜坡。',
        '相近约0.11 rad跟踪误差下，左J2/J3速度约0.15 rad/s，右约0.32 rad/s；与此前只读KP_APR 54/120的左右差异方向一致，不能据此认定唯一原因。'],
    caveats=['本次含初始运动、长保持与返回，并非单一理想阶跃；自动单阶跃报告拒绝拟合是合理的，本报告单独分析首段保持。',
        '原报告每轴1个初始对齐空值，共12行，源于各流启动相位；本次原生周期无缺失、位置均有效，无须将该标记解释为CAN丢帧。',
        '上述匹配间隔反映主机采样/发布/串口写入链路，不是纯CAN往返延迟或电机内部周期。',
        '首段到位带取±max(0.01 rad,2%首段平台幅值)，统计从示教进入该带到实际持续进入该带的时间。',
        '多轴同时运动且受跟随限步约束，不能直接从这些曲线确定新的PID参数。',
        'J2/J3返回时下发目标有0.02 rad下限（关节下限+原程序limit_margin），这部分示教/运动角差不是电机静差。'])
(OUT/'analysis.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
pd.json_normalize(metrics).to_csv(OUT/'per_joint_metrics.csv',index=False)
pd.json_normalize(links).to_csv(OUT/'pipeline_metrics.csv',index=False)

font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),'axes.unicode_minus':False,
    'axes.spines.top':False,'axes.spines.right':False,'axes.titlesize':11,'font.size':10,'savefig.facecolor':'white'})

def plot_axis(ax,side,joint,kind):
    teacher=arm[arm.role=='teaching_'+side];follower=arm[arm.role=='motion_'+side]
    raw=target[(target.follower=='motion_'+side)&(target.kind=='raw')]
    if kind==1:
        ax.plot(teacher[f'feedback_receive_time_j{joint}']-t0,teacher[f'position_j{joint}'],c=colors['teacher'],lw=1.9,label='示教臂实际位置',zorder=2)
        ax.plot(raw.t,raw[f'q_j{joint}'],c=colors['raw'],lw=1.1,ls=(0,(3,3)),label='原程序原始目标（核查通信）',zorder=3)
    ax.plot(follower[f'send_complete_time_j{joint}']-t0,follower[f'sent_position_j{joint}'],c=colors['sent'],lw=1.7,label='运动臂下发目标')
    if kind==2:
        ax.plot(follower[f'feedback_receive_time_j{joint}']-t0,follower[f'position_j{joint}'],c=colors['actual'],lw=1.65,label='运动臂实际位置')
    ax.set_xlim(0,32);ax.set_xticks(np.arange(0,33,4));ax.grid(alpha=.19)
    ax.set_xlabel('进入跟随后时间 / s');ax.set_ylabel('关节位置 / rad')
    ax.set_title(names[side]+' J'+str(joint),loc='left',fontweight='bold')

for kind,filename,title in [(1,'01_teacher_vs_target','① 示教臂实际位置 → 运动臂目标：分开观察原始目标与下发目标'),
                           (2,'02_target_vs_actual','② 运动臂下发目标 → 实际位置：观察运动端跟随响应')]:
    fig,axes=plt.subplots(6,2,figsize=(14,17.5),layout='constrained')
    for col,side in enumerate(('left','right')):
        for joint in range(1,7):plot_axis(axes[joint-1,col],side,joint,kind)
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.5,.963),ncol=len(labels),frameon=False)
    fig.suptitle(title+'\n真机记录 2026-11-25 21:14 · 底层 250 Hz / 目标 100 Hz · 全部横轴使用同一时间起点',fontsize=15,y=.997)
    fig.get_layout_engine().set(rect=(0,.022,1,.90))
    fig.text(.5,.006,'下发目标 = 主机成功交给串口的 POS_VEL 位置值；J7 夹爪只读。曲线未经平滑或人为时间平移。',ha='center',fontsize=9,color='#586475')
    fig.savefig(OUT/(filename+'.png'),dpi=160,bbox_inches='tight');fig.savefig(OUT/(filename+'.svg'),bbox_inches='tight');plt.close(fig)

fig,axes=plt.subplots(2,2,figsize=(14.5,8),layout='constrained')
for row,side in enumerate(('left','right')):
    for col,kind in enumerate((1,2)):
        plot_axis(axes[row,col],side,2,kind)
        axes[row,col].set_ylim(-.07,1.9)
        axes[row,col].set_title(names[side]+' J2 · '+('示教实际 / 原始目标 / 下发目标' if kind==1 else '下发目标 / 运动实际'),loc='left',fontsize=12)
        axes[row,col].legend(loc='upper right',fontsize=8,framealpha=.96)
    m=next(x for x in metrics if x['side']==side and x['joint']==2)
    axes[row,0].text(.04,.84,'示教与原始目标基本重合\n下发目标被限步变为斜坡',transform=axes[row,0].transAxes,fontsize=10,
        bbox=dict(boxstyle='round,pad=.4',facecolor='white',edgecolor='#d6dee5',alpha=.95))
    axes[row,1].text(.04,.84,f"受限追赶速度约 {m['saturated_ramp_velocity_rad_s']['median']:.2f} rad/s\n示教进入保持带后，约 {m['after_teacher_hold_s']:.2f} s 到位",transform=axes[row,1].transAxes,fontsize=10,
        bbox=dict(boxstyle='round,pad=.4',facecolor='white',edgecolor='#d6dee5',alpha=.95))
fig.suptitle('跟随慢的主要证据：目标限步与运动端响应共同决定追赶速度',fontsize=17)
fig.get_layout_engine().set(rect=(0,.04,1,.95))
fig.text(.5,.006,'代表性关节 J2；J3 呈相同趋势。到位带为首段平台 ±2%（不小于 0.01 rad）。原始曲线，无平滑、无平移。',ha='center',fontsize=10,color='#586475')
fig.savefig(OUT/'overview_J2.png',dpi=170);fig.savefig(OUT/'overview_J2.svg');plt.close(fig)

rows=''.join(f'<tr><td>{names[m["side"]]} J{m["joint"]}</td><td>{m["saturated_ramp_velocity_rad_s"]["median"]:.3f}</td><td>{m["saturated_ramp_native_error_rad"]["median"]:.3f}</td><td>{m["after_teacher_hold_s"]:.2f}</td><td>{m["plateau_13_15s_native_error_rad"]:.4f}</td></tr>' for m in metrics if m['joint'] in (2,3))
link_rows=''.join(f'<tr><td>{names[m["side"]]}</td><td>{"示教快照 → 原始目标发布" if m["kind"]=="teacher_snapshot_to_raw" else "限步目标发布 → 底层串口接受"}</td><td>{m["interval_ms"]["median"]:.2f}</td><td>{m["interval_ms"]["p95"]:.2f}</td><td>{m["matched"]}/{m["eligible"]}</td></tr>' for m in links)
html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>四臂跟随曲线与结论</title><style>body{font:16px/1.75 system-ui,sans-serif;color:#243447;background:#f7f9fc;max-width:1250px;margin:28px auto;padding:0 22px 40px}h1{font-size:29px}h2{margin-top:32px}img{width:100%;background:white;border:1px solid #d7e0e8;border-radius:7px}table{border-collapse:collapse;width:100%;background:white}td,th{padding:9px;border:1px solid #d7e0e8;text-align:left}a{color:#0871b2}.conclusion{border-left:5px solid #0871b2;background:#eaf4fb;padding:15px 20px}.muted{color:#5b6c7d}code{background:#e7edf4;padding:2px 5px}</style>
<h1>先保持 250 / 100 Hz，转查位置跟随响应与限步逻辑</h1>
<p class="muted">真机会话 teaching_394b0c668f · 2026-11-25 21:14:20 起 · 约 31.96 秒 · 本次仅离线分析，没有修改程序、频率或参数。</p>
<div class="conclusion"><b>结论：通信不是当前数秒滞后的主要来源。原始目标能及时反映示教位置；原程序以运动臂当前位置为中心限步，产生缓慢斜坡。电机在这个小误差下的响应又决定斜坡推进速度，左臂 J2/J3 明显比右臂慢。</b></div>
<img src="overview_J2.png" alt="左右臂J2目标形成和实际跟随对比">
<h2>两个阶段必须分开看</h2>
<p>第一组图同时显示示教实际角、原程序原始目标和最终下发目标。示教实际角与原始目标基本重合；原始目标到下发目标的大差距来自目标处理。第二组图使用底层实际交给串口的目标对比运动臂实际角，直接观察执行端响应。</p>
<p>原版 <code>apply_safety()</code> 对前三轴设置 <code>min(0.35, 7.5 / 100 × 1.5) = 0.1125 rad</code> 的超前量，后三轴为 <code>0.075 rad</code>，参考量是运动臂当前反馈。因而运动臂慢，下一次目标也随它缓慢前进；它没有直接收到完整的示教阶跃。</p>
<h2>通信与采样证据</h2>
<p>四臂各记录 7,977 个原生周期，累计计数对应约 250 Hz；队列丢弃、周期缺失和新增超过 6 ms 的周期均为 0。每侧 raw/safe 各 3,191 条，上位机目标约 99.89 Hz。</p>
<table><tr><th>臂</th><th>主机侧链路</th><th>中位 / ms</th><th>P95 / ms</th><th>变化目标匹配数</th></tr>'''+link_rows+'''</table>
<p class="muted">仅对变化的六轴目标向量做时间匹配；相同量化值可能重复，因此示教源年龄是近似估计。时间精度受 4 ms 原生采样与 10 ms 目标周期限制。这些数值不是 CAN ACK、物理往返时间或电机采样时刻，不能声称通信延迟为零；但不支持以通信解释本次持续数秒的追赶。</p>
<h2>运动端响应：左侧明显更慢</h2>
<table><tr><th>关节</th><th>受限追赶速度中位 / rad·s⁻¹</th><th>目标−实际误差中位 / rad</th><th>示教进入保持带后到位 / s</th><th>首段保持末期误差 / rad</th></tr>'''+rows+'''</table>
<p>首段长保持在约 3～16 秒；到位带为 ±max(0.01 rad, 2%首段平台幅值)，须持续保持到该段结束。上述到位耗时包含限步形成的目标斜坡，不能当作固定目标阶跃的电机固有稳定时间。相近约 0.11 rad 误差下，左 J2/J3 约 0.15 rad/s，右约 0.32 rad/s，右侧约快一倍；保持足够时间后误差很小，主要表现为追赶缓慢，未见持续明显振荡。</p>
<p>20:58 的已授权只读扫描显示：左运动臂 J1～J6 的 KP_APR=54，右侧=120；相应位置积分和速度环参数相同。这与左右响应差异方向一致，是优先复核位置环的依据；多轴负载和限步耦合仍存在，不能认定某个增益是唯一原因，更不能直接确定新参数。</p>
<h2>全部关节曲线</h2>
<p><a href="01_teacher_vs_target.png">下载图 1 PNG</a> · <a href="01_teacher_vs_target.svg">图 1 SVG</a> · <a href="02_target_vs_actual.png">下载图 2 PNG</a> · <a href="02_target_vs_actual.svg">图 2 SVG</a></p>
<img src="01_teacher_vs_target.png" alt="全部J1到J6示教实际和目标时间曲线">
<img src="02_target_vs_actual.png" alt="全部J1到J6下发目标和实际时间曲线">
<h2>记录边界与下一步</h2>
<p>本次包含初始移动、长保持和返回。自动单阶跃报告因没有初始静止段或有再次移动而拒绝给出统一阶跃拟合；本次分析选取首段保持进行比较。原报告每轴的一个初始对齐空值来自数据流起始相位，并不是原生周期缺失。J2/J3 返回时下发目标最低约 0.02 rad，来自关节下限加原版 limit_margin；不要把示教 0 与实际约 0.02 的差距当成电机静差。夹爪只读，不作位置闭环评价。</p>
<p><b>建议：暂不继续提频。优先检查运动臂位置闭环，重点是左 J2/J3；同时把原版限步的影响纳入后续调参验证。</b>本轮先给结论，不修改增益或限步。</p>
<p class="muted">完整指标：<a href="analysis.json">analysis.json</a> · <a href="per_joint_metrics.csv">逐轴指标 CSV</a> · <a href="pipeline_metrics.csv">链路指标 CSV</a> · <a href="provenance.json">来源 SHA256</a> · <a href="analyze.py">可复现分析脚本</a>。绘图采用各流原始主机时间戳，无平滑、无为了重合而移动曲线。</p></html>'''
(OUT/'report.html').write_text(html)
for path,value in HASHES.items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==value
(OUT/'provenance.json').write_text(json.dumps(HASHES,ensure_ascii=False,indent=2))
print(json.dumps(dict(links=links,core_metrics=[m for m in metrics if m['joint'] in (2,3)]),ensure_ascii=False,indent=2))
print('Output:',OUT)
