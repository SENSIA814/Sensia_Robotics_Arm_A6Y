from pathlib import Path
import json,html,datetime
P=Path(__file__).resolve().parent
metrics=json.loads((P/'metrics.json').read_text());peaks=json.loads((P/'peaks.json').read_text());timing=json.loads((P/'timing.json').read_text())
esc=html.escape
rows=[]
for m in metrics:
 lag=m['lag_safe_actual'];v=f"{lag['seconds']:.2f}" if lag else '未充分运动，不估算'
 role='左臂' if m['role']=='motion_left' else '右臂'
 rows.append(f"<tr><td>{m['hz']} Hz</td><td>{role} J{m['joint']}</td><td>{v}</td><td>{m['raw_safe']['maximum']:.3f}</td><td>{m['safe_actual']['maximum']:.3f}</td><td>{m['raw_actual']['maximum']:.3f}</td></tr>")
page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>A6Y 跟随日志定位 · 2026-11-25</title>
<style>body{font:16px/1.75 system-ui,"Noto Sans CJK SC",sans-serif;background:#f4f6f8;color:#172330;margin:0}main{max-width:1100px;margin:auto;padding:32px}h1{font-size:30px}h2{font-size:21px;margin-top:32px}p{margin:12px 0}section{background:white;border:1px solid #dce3e9;border-radius:10px;padding:20px 26px;margin:20px 0}table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}td,th{text-align:left;padding:9px;border-bottom:1px solid #dce3e9}th{background:#eff5f6}code{background:#eff3f7;padding:2px 6px;border-radius:4px}img{width:100%;height:auto}.lead{border-left:5px solid #147d8c;padding:8px 18px;background:#e8f4f4}.note{color:#546676;font-size:14px}a{color:#086c86}ul{padding-left:22px}details{margin:18px 0}summary{cursor:pointer;font-weight:600}</style><main>
<h1>运动跟随滞后：两次真机日志的分段定位</h1>
<p class="note">2026-11-25 · 只读分析既有日志 · 没有连接电机、修改运行程序或调整参数</p>
<p class="lead"><b>结论：运动端对发布目标的响应偏慢有证据；上游“目标距离限制”也在显著放大滞后。</b><br>当前不能把全部问题归到电机 PID，也不能仅用 50/100 Hz 效果近似就排除通信。优先区分 raw → safe → actual 三段，避免把被限制过的目标误当作示教臂的原始目标。</p>
<section><h2>样本与比较范围</h2><table><tr><th>会话</th><th>开始时间</th><th>目标频率</th><th>跟随时长</th><th>日志观测平均频率</th></tr>
<tr><td>teaching_2b6bab1c45</td><td>19:50:46</td><td>50 Hz</td><td>219.63 s</td><td>约 50 Hz</td></tr>
<tr><td>teaching_ac84a69ae6</td><td>19:55:27</td><td>100 Hz</td><td>33.09 s</td><td>左 100.11 / 右 99.59 Hz</td></tr></table>
<p>都是左对左、右对右的四臂真机会话，均正常停止，没有会话故障。两次 software.json 中源码哈希全部一致。统计仅取进入 following 后第 1 秒至首次 stop_requested 之前，排除启动接管、软启动和退出回零。</p>
<p><b>两次操作者动作、行程和持续时间不同，不是相同轨迹的 A/B 试验。</b>不能直接把第二次更大的峰值误差解释为提高频率导致的因果结果。100 Hz 左臂 J1、J2 基本未运动，不能据此评价这两轴响应。</p></section>
<section><h2>1. 原程序在目标生成阶段限制了追赶距离</h2>
<p>归档原程序 <code>leader_follower_teleop.py:508–537</code> 中，alpha=1、预测关闭。将示教角度限制到关节范围后，围绕<b>运动臂实际反馈位置</b>限制目标距离，并非仅限制相邻两条指令的差：</p>
<p><code>step = min(0.35, max_velocity × 1.5 / frequency)</code><br><code>safe = actual + clip(raw − actual, −step, +step)</code>（另有位置边界限制）</p>
<table><tr><th>关节</th><th>50 Hz 最大目标距离</th><th>100 Hz 最大目标距离</th></tr><tr><td>J1～J3</td><td>0.225 rad（12.89°）</td><td>0.1125 rad（6.45°）</td></tr><tr><td>J4～J6</td><td>0.150 rad（8.59°）</td><td>0.075 rad（4.30°）</td></tr></table>
<p>因此加倍发布频率，同时把允许目标领先实际位置的距离减半。运动臂已经落后时，发布目标会跟着实际位置缓慢前移；如果位置响应速度与误差相关，就可能进一步限制追赶速度。这是代码和日志共同支持的机制，不能理解为单纯“发得越快就跟得越快”。</p>
<p><b>100 Hz 右臂 J3，进入跟随后 9.243 s 的同一条日志：</b>示教目标 1.350996 rad，发布目标 0.416345 rad，实际位置 0.303845 rad。总偏差 1.047150 rad（60.00°）分为 raw→safe 0.934650 rad 和 safe→actual 0.112500 rad。该时刻绝大部分角度差已发生在目标限制段；这只是误差分解，不等于每段对闭环因果贡献的百分比。</p>
<img src="tracking_comparison.png" alt="两次试验右臂 J2、J3 的 raw、safe 和 actual 曲线">
<p class="note">绿色：示教目标 raw；橙色：跟随节点发布目标 safe（ROS 层）；深蓝：运动臂反馈 actual。每幅图选择该关节总偏差最大值附近，横轴均为各自进入跟随后经过的时间。</p>
<p>100 Hz 右 J3 的这次上升中，raw 达到 1 rad 的时间为 8.955 s，safe 为 11.082 s，actual 为 11.446 s：目标生成段已经有约 2.13 s 的追赶过程，之后实际位置还落后发布目标约 0.36 s。这个单次过阈值时间是局部现象，不是固定传输延迟。</p></section>
<section><h2>2. 运动端对发布目标的响应滞后仍然明显</h2>
<p>将 safe 曲线前移与 actual 对齐，得到以下<b>等效跟踪滞后</b>。这是综合运动响应、链路和日志采样的曲线估计，不是电机内部纯时延，也不是 PID 参数。</p>
<table><tr><th>对象</th><th>50 Hz</th><th>100 Hz</th></tr><tr><td>左臂 J1 / J2 / J3</td><td>0.69 / 0.68 / 0.71 s</td><td>J1、J2 未充分运动；J3 约 0.54 s，动作较短</td></tr><tr><td>右臂 J1 / J2 / J3</td><td>0.32 / 0.33 / 0.33 s</td><td>J1 未充分运动；J2 0.32 s、J3 0.33 s</td></tr><tr><td>左腕 J4～J6</td><td>约 0.17～0.20 s</td><td>约 0.17～0.18 s</td></tr><tr><td>右腕 J4～J6</td><td>约 0.10～0.11 s</td><td>约 0.07～0.08 s</td></tr></table>
<p>使用每臂独立的约 20 Hz telemetry.csv 位置记录交叉检查：50 Hz 左 J2/J3 为 0.68/0.71 s，右 J2/J3 为 0.33/0.33 s；100 Hz 右 J2/J3 为 0.32/0.32 s，与 references.csv 结果一致。左/右臂、前三轴/腕部的响应差别也不符合“所有轴只是同一个固定通信延迟”的简单解释。</p>
<p>右 J3 的反馈速度峰值在两段记录中分别为 0.667 rad/s（50 Hz）和 0.364 rad/s（100 Hz），远低于驱动配置的 7.5 rad/s 速度字段上限。100 Hz 的目标距离限幅持续阶段约 0.3 rad/s 的运动速度，与 0.1125 rad 的小位置误差并存；<b>这支持“目标限制与偏慢的位置响应相互作用”</b>，尚不能据此区分内部位置增益、速度环、加减速处理、负载与链路各占多少。
</p><img src="right_j3_100hz_limits.png" alt="100Hz 右 J3 位置误差限幅与反馈速度"></section>
<section><h2>3. 关于通信：没有超时证据，但现有日志不足以完全排除</h2>
<p>两次实际观测目标频率接近设定值，following 期间没有反馈超时或跟随停止状态。接收日志存在调度抖动：50 Hz 单臂相邻记录间隔 P99 约 44～46 ms、最大约 101 ms；100 Hz P99 约 32～35 ms、最大约 67 ms。这些是<b>日志观察进程的回调间隔</b>，不能直接当成总线发送周期或网络延迟。</p>
<p>两次 controller_manager.log 都记录 <code>update rate is 125 Hz</code>。用户切换的 50/100 Hz 是跟随节点的目标发布频率，并没有同时改变底层 ros2_control 的配置频率；因此这两次不能作为底层 CAN 通信频率的排除试验。日志中另有无法启用 FIFO 实时调度的警告，它只说明实时调度未启用，不能单独证明本次滞后由调度造成。</p>
<p>现有 references.csv 的 wall_time 是观察者接收 safe 的时刻，raw、teacher_q、follower_q 是各订阅的最近值，不保证是同一控制周期；sequence 是观察者自己累加，连续不能证明没有丢包。记录没有保留 safe/JointState 原始时间戳，也没有驱动实际接受的位置目标或实际 CAN 发送时刻。</p>
<p>所以现阶段可以确认“目标生成限制”和“发布目标至反馈的慢响应”两层，<b>不能确认慢响应完全由电机内部位置/速度环参数造成</b>。</p></section>
<section><h2>4. 参数记录只作为下一步核对线索</h2>
<p>2026-11-24 的只读文件按 USB 身份对应：运动左臂 304C33733036，J1～J6 的 KP_APR=54；运动右臂 209B3971534B，J1～J6 的 KP_APR=120。这个左右差异与本次“左臂更慢”的现象相符，但<b>它是历史快照，不是这两次运行同步读回的当前参数</b>，不能据此直接给出新参数。仓库内 10 月的旧调参记录也不能直接套用到当前四臂。</p>
<p>下一步若要把问题收敛到具体电机环路，先补齐：同一周期的 raw/safe 与源时间戳、驱动实际执行位置目标及 POS_VEL 速度字段、CAN 发送/反馈时间或序号，以及当前 KP_APR、KP_ASR、KI_ASR、ACC/DEC、MAX_SPD 的只读快照。用同一段小范围动作重复采样，并把“未触发目标限幅”的片段单独分析，再决定调整哪一环。本次不修改这些参数，也不要求立即扩大动作范围。</p></section>
<section><h2>方法与可复核结果</h2>
<p>等效滞后算法：将位置数据插值到 10 ms 网格，对两信号做 30 ms 标准差的高斯平滑；仅分析任一信号变化率超过 0.05 rad/s 的片段；所有候选滞后使用相同的中间时间窗，在 −0.3～2.0 s 内以 10 ms 步长最小化 safe(t−lag) 与 actual(t) 的均方误差。目标范围不足 0.08 rad 或有效运动不足 1 s 时不估计。插值不增加原始测量精度。该方法只用于观察响应量级，不能辨识电机传递函数。</p>
<p>角度表中的最大值分别统计，不应跨列相加；上文 9.243 s 的数值才是同一时刻的可加分解。J2/J3 近零位还存在原程序 0.02 rad 的位置边界 margin，它引起的零位附近目标偏差不能误判为电机误差。</p>
<details><summary>全部关节统计：位置误差单位 rad，等效滞后单位 s</summary><table><tr><th>频率</th><th>运动臂关节</th><th>safe→actual 等效滞后</th><th>max |raw−safe|</th><th>max |safe−actual|</th><th>max |raw−actual|</th></tr>'''+''.join(rows)+'''</table></details>
<p><a href="metrics.csv">完整逐轴指标 CSV</a> · <a href="peaks.csv">同一时刻峰值分解 CSV</a> · <a href="timing.csv">日志回调间隔 CSV</a> · <a href="velocities.csv">反馈速度统计 CSV</a> · <a href="analyze.py">分析脚本</a> · <a href="provenance.json">数据来源与 SHA256</a></p>
<p class="note">程序与参数均未更改；输出仅新增于 diagnostics/teaching_tracking_analysis_20261125。分析依赖的原始日志保持原样。</p></section></main></html>'''
(P/'report.html').write_text(page,encoding='utf-8')
print(P/'report.html')
