"""Offline log analysis only: no ROS imports, serial access, or control changes."""
from pathlib import Path
import ast, hashlib, json, os
import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d
os.environ.setdefault('MPLCONFIGDIR','/tmp/a6y-tracking-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
# Reuse the previous estimator verbatim, without executing its file-level writes.
METHOD=ROOT/'diagnostics/teaching_tracking_analysis_20261125/analyze.py'
fn=next(n for n in ast.parse(METHOD.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='estimate_lag')
exec(compile(ast.Module(body=[fn],type_ignores=[]),str(METHOD),'exec'))
FILES={str(METHOD):hashlib.sha256(METHOD.read_bytes()).hexdigest()}

def record(path):
    FILES[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    return path

def dist(series):
    return {k:float(v) for k,v in zip(('median','p95_sampled','maximum_sampled'),np.nanquantile(series,[.5,.95,1]))}

sessions={};tracking=[];native=[]
for hz,sid in ((125,'teaching_ac84a69ae6'),(250,'teaching_769d352988')):
    folder=ROOT/'diagnostics/arm_teaching_reports'/sid
    meta=json.loads(record(folder/'session.json').read_text())
    result=json.loads(record(folder/'result.json').read_text())
    events=[json.loads(s) for s in record(folder/'events.jsonl').read_text().splitlines()]
    start=next(e['time'] for e in events if e.get('phase')=='following')
    end=next(e['time'] for e in events if e['kind']=='stop_requested')
    refs=pd.read_csv(record(folder/'references.csv'))
    refs=refs[(refs.wall_time>=start+1)&(refs.wall_time<end)].copy()
    refs['t']=refs.wall_time-start
    sessions[hz]=dict(session=sid,following_seconds=end-start,target_config_hz=meta['frequency'],
                      observed_target_hz=result['actual_hz'],error=result['error'],exit_error=result['close_error'])
    record(folder/'software.json')
    for role,data in refs.groupby('follower'):
        record(folder/(role+'_teleop.yaml'))
        arm=Path(result['arms'][role]['directory'])
        record(arm/'control_controller_manager_1.yaml')
        telemetry=pd.read_csv(record(arm/'telemetry.csv'))
        telemetry=telemetry[(telemetry.time>=start+1)&(telemetry.time<end)]
        for j in range(1,7):
            t=data.t.to_numpy();raw=data[f'raw_j{j}'].to_numpy();safe=data[f'safe_j{j}'].to_numpy();actual=data[f'follower_q_j{j}'].to_numpy()
            k=int(np.argmax(abs(raw-actual)))
            velocity=telemetry[f'velocity_joint{j}'].to_numpy()
            error=np.interp(telemetry.time,data.wall_time,safe)-telemetry[f'q_joint{j}'].to_numpy()
            use=(abs(error)>.03)&(velocity*error>0)&(abs(velocity)>.05)
            tracking.append(dict(control_hz=hz,role=role,joint=j,raw_range_rad=float(np.ptp(raw)),
                safe_to_actual_alignment=estimate_lag(t,safe,actual),
                actual_velocity_p95=float(np.quantile(abs(velocity),.95)),actual_velocity_max=float(max(abs(velocity))),
                median_velocity_over_error=float(np.median(abs(velocity[use]/error[use]))) if use.any() else None,
                peak=dict(t=float(t[k]),raw=float(raw[k]),safe=float(safe[k]),actual=float(actual[k]),
                          raw_minus_safe=float(raw[k]-safe[k]),safe_minus_actual=float(safe[k]-actual[k]))))
    if hz==250:
        current=refs
        freqs=pd.read_csv(record(folder/'frequency.csv'))
        for role,data in freqs.groupby('role'):
            a,b=data.iloc[0],data.iloc[-1];dt=b.control_time-a.control_time
            native.append(dict(role=role,seconds=dt,loop_hz=(b.control_cycle_count-a.control_cycle_count)/dt,
                command_hz=[(b[f'command_count_j{i}']-a[f'command_count_j{i}'])/dt for i in range(1,8)],
                feedback_hz=[(b[f'feedback_count_j{i}']-a[f'feedback_count_j{i}'])/dt for i in range(1,8)],
                new_cycles_over_6ms=b.control_overrun_count-a.control_overrun_count,
                sampled_control_period_ms=dist(data.control_period_ms),sampled_read_write_ms=dist(data.bus_cycle_ms),
                sampled_observer_diagnostic_age_s=dist(data.diagnostic_age_s)))

summary=dict(sessions=sessions,native=native,tracking=tracking,
    conclusion='保持底层250 Hz / 目标100 Hz，优先检查运动端POS_VEL响应及与原程序目标限步的共同影响。',
    caveats=['两次手动示教动作不同，不能将差值视为严格A/B因果效果。',
             '滞后为运动段轨迹最佳平移对齐量，包含动力学/采样/调度，不等于纯通信延迟或精确时间常数。',
             '发送计数是主机串口接受命令；反馈计数是解析帧，缺少逐条CAN ACK/电机接收时间戳。',
             '耗时与反馈年龄为低频诊断采样，不能据此声称端到端通信为零延迟。',
             'raw、safe、两端反馈来自异步观测，单个时刻数值是近似分解。'])
(OUT/'analysis.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
(OUT/'provenance.json').write_text(json.dumps(FILES,ensure_ascii=False,indent=2))
font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),'axes.unicode_minus':False,'axes.spines.top':False,'axes.spines.right':False})
fig,axs=plt.subplots(2,2,figsize=(13,7.5),layout='constrained')
for col,(role,name) in enumerate((('motion_left','左运动臂'),('motion_right','右运动臂'))):
    d=current[current.follower==role]
    for kind,label,color in [('raw','示教目标','#078c9f'),('safe','实际发布目标','#e1812c'),('follower_q','运动臂反馈','#303a68')]:
        axs[0,col].plot(d.t,d[kind+'_j3'],label=label,c=color,lw=1.5)
    axs[0,col].set_title(name+' J3：250 Hz底层 / 100 Hz目标');axs[0,col].set_ylabel('位置 / rad');axs[0,col].legend(fontsize=9)
    axs[1,col].plot(d.t,d.safe_j3-d.follower_q_j3,c='#e1812c',label='发布目标 − 实际反馈')
    for x in (-.1125,.1125):axs[1,col].axhline(x,c='#888',ls='--',lw=1)
    lag=next(x for x in tracking if x['control_hz']==250 and x['role']==role and x['joint']==3)['safe_to_actual_alignment']['seconds']
    axs[1,col].set_title(f'发布目标到实际反馈：对齐滞后约 {lag:.2f} s；虚线为 ±0.1125 rad 限步')
    axs[1,col].set_ylabel('位置差 / rad');axs[1,col].set_xlabel('进入跟随后 / s')
for ax in axs.flat:ax.grid(alpha=.18)
fig.suptitle('最新真机日志：提频已生效，运动端追赶与原程序限步仍然存在',fontsize=15)
fig.savefig(OUT/'tracking.png',dpi=160);plt.close(fig)
rows=''.join(f'<tr><td>{"左" if role=="motion_left" else "右"}臂 J{j}</td>'+''.join('<td>'+str(next(x for x in tracking if x['control_hz']==hz and x['role']==role and x['joint']==j)['safe_to_actual_alignment']['seconds'])+' s</td>' for hz in (125,250))+'</tr>' for role,j in [('motion_right',2),('motion_right',3),('motion_left',4)])
html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>250 Hz 真机日志分析</title><style>body{font:17px/1.7 sans-serif;max-width:1100px;margin:36px auto;padding:0 24px;color:#223;background:#fafbfd}h1{font-size:28px}table{border-collapse:collapse;width:100%}td,th{padding:9px;border:1px solid #ccd}img{width:100%}small{color:#567}code{background:#eef;padding:2px 5px}</style><h1>建议保持 250/100 Hz，转查运动端 POS_VEL 响应与目标限步</h1><p>最新会话：teaching_769d352988，2026-11-25 20:28，真机跟随约39.22秒，正常回零退出。</p><p>四臂每轴命令和反馈的累计计数都约250 Hz；采样覆盖约38.9秒内没有新增超过6 ms的底层周期。上位机目标观测平均99.38 Hz。现有证据不支持继续提频作为主要解决手段。</p><table><tr><th>关节</th><th>125 Hz底层 / 100 Hz目标</th><th>250 Hz底层 / 100 Hz目标</th></tr>'''+rows+'''</table><p>这里是发布目标到实际反馈的运动段最佳平移对齐量，两次动作不同，不能作为严格A/B试验。最新左臂J2/J3分别约0.68/0.69秒，右臂约0.31秒，左右差异也值得在电机参数中核对。</p><img src="tracking.png"><p>原版程序仍以运动臂当前位置为中心限步：前三轴 min(0.35, 7.5 / 100 × 1.5) = 0.1125 rad。最新右臂J3在跟随后约12.87秒：示教目标1.232、发布目标0.656、实际反馈0.540 rad。约0.692 rad总差中，约0.576 rad位于示教目标→发布目标，约0.116 rad位于发布目标→实际反馈。</p><p>最新左臂J2/J3反馈速度P95约0.164/0.168 rad/s，右臂约0.339/0.339 rad/s。相同目标超前量下，运动端响应偏慢会使限步后的目标持续跟着实际位置慢慢推进；这不能只归因于通信，也不能直接认定为某个PID参数。</p><p>下一步维持本次频率，先核对运动臂当前位置环/速度环增益、速度与加减速约束，再围绕J2/J3做单轴受控响应记录。本轮未修改程序、频率、电机参数，未连接实体串口。</p><small>原始记录、方法与限制见 analysis.json / provenance.json / analyze.py。主机发送计数不是CAN ACK，读写调用耗时不是线上的端到端延迟；raw/safe/反馈为异步观测。</small></html>'''
(OUT/'report.html').write_text(html)
for p,digest in FILES.items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest
print('Saved analysis, plot and report; all source hashes unchanged.')
