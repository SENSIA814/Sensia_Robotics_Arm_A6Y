# A5 VR 实机入口

当前按用户要求优先使用**原生位置控制**：Quest 右手柄 → 相对 TCP 目标 → 六轴 IK → C++ 许可/限速 → 电机 `pos_vel` 位置闭环。首次目标从真实反馈捕获，不使用虚拟 home；不运行重力补偿或阻抗力矩计算。`motion_range:=full` 同时控制末端平移、旋转和夹爪；原小范围配置仍保持夹爪只读。原关节阻抗入口仍保留在本文后半部分，不能用它的保持记录证明新位控已经通过实机验收。

**2026-10-09 J1 后续调参已完成：**在不改变 VR/控制器限制的情况下，将电机内部 `KP_APR` 从 54 调到 250。12 次单轴往返完成并确认失能；30°/12 秒同轨迹最大落后从 7.68°降至 0.94°，30°/6 秒重复测试约 1.96°，均不再触发跟踪减速。250 当前仅保存在电机 RAM，未写 Flash。参数与曲线见 [J1 调参报告](../../电机调试/J1_POSITION_TUNING_20261009.md)；这仅验证了当前姿态下 J1 的响应，其他轴与完整 VR 跟随仍需分别验证。以下早期测试保留为调参前的历史基线。

**2026-10-09 J1 独立位置响应实测：**已在用户授权的安装位置完成相对 +30°、保持、返回起点。每段采用 12 秒五次平滑曲线，计划峰值 4.6875°/s；只有 J1 可使能，其余电机配置为只读。结果：最大行程 30.0096°，返回误差 0.0437°，其他关节读数最大变化 0.0219°（约一个编码器量化格）；正常结束并确认失能，控制进程已退出。

此测试不使用 Quest 或 IK，仍出现跟踪滞后：计划到下发最大差 5.361°，下发到实测最大差 2.395°；约 47.3% 的测试采样处于 `TRACKING_LIMITED`。在减速段按编码器位置的 250 ms 差分估计，正反向持续速度均约 3.07°/s。因此该现象可在独立 J1 位控中复现，不能归因于 VR 逆解计算耗时；尚不能据此确定电机硬件最大速度或具体固件参数问题。详见[实测报告](../../../validation/j1_position/real_30deg_12s/report.json)、[曲线](../../../validation/j1_position/real_30deg_12s/tracking.png)和[原始 CSV](../../../validation/j1_position/real_30deg_12s/trace.csv)。

复测入口（默认 `mock`；`real` 会自动执行 PREPARE、ARM、J1 往返及 DISARM，不需要 VR 网关）：

```bash
# 先退出旧控制 launch。实机测试只适用于其他关节失能后能保持稳定的安装姿态。
bash scripts/run_j1_position_test.bash --mode mock --degrees 30 --move-seconds 12
bash scripts/run_j1_position_test.bash --mode real --degrees 30 --move-seconds 12
```

每次在 `validation/j1_position/` 创建独立目录，保存实际控制参数、只读电机配置、日志、三条角度曲线和报告。测试从实时反馈捕获起点，角度不超过 ±30°，受原有关节范围及速度/加速度/跟踪/反馈超时约束；故障不自动恢复，也不强行返回。正常达到端点与起点后才记成功；中断时请求失能，控制器/驱动保留独立超时。专用入口允许明确标记 `live=False` 的诊断参考，生产 VR 配置仍拒绝测试输入。只读关节查询周期为 16 ms，以满足未修改的 50 ms 反馈阈值。

J1 原生参数的[只读快照](../../../validation/j1_position/j1_motor_params_before_tuning.json)已取得：`CTRL_MODE=2`、`KP_APR=54`、`KI_APR=0`、`KP_ASR≈0.00384`、`KI_ASR≈0.002`、`ACC=2`、`DEC=-2`、`MAX_SPD=600`、`Gr=40`、`PMAX/VMAX/TMAX=12.5/10/28`；24 个寄存器均有匹配应答，读取前后状态均为失能。反馈映射范围与项目 DM4340P 解码表一致。该快照未修改任何寄存器；不能仅凭这些数值认定某个增益不正确，`MAX_SPD` 和加减速参数的单位需结合实际固件核对。

退出控制 launch 后，可再次只读导出参数（不使能、不写入、不保存 Flash）：

```bash
python3 tools/read_motor_position_params.py --joint joint1
```

已从旧 A6-Y 项目移植通用的读取、临时写入和 Flash 保存工具，支持按关节选择目标。安装与调参命令见 [电机参数编辑](../../电机调试/MOTOR_PARAMETERS.md)；本次移植只完成代码和离线验证，没有修改电机寄存器。

该工具对白名单读取帧进行检查；每次新建 JSON 快照，保留寄存器编号、原始值和发送帧。后续位置环调整应先保留本快照，逐项写入并读回，再重复相同 30°/12 秒测试，比较下发到实测误差、速度、过冲和静止抖动。正常 `pos_vel` 模式使用固件内部位置/速度环，ROS 阻抗刚度参数不参与此模式。`PMAX/VMAX/TMAX` 是协议映射范围，不应作为提速旋钮。[厂商 DM-J4340P-2EC 说明书](https://doc.switch-science.com/media/files/5e033168-99f1-4d2d-b758-e03192a1f071.pdf)说明了模式 2、位置/速度环寄存器及读写/保存协议；具体固件行为仍以实物版本为准。

**2026-10-09 10:49 实机复测后的修正：**上一次减速逻辑已加载，但跟随 4.696 秒后仍因 J1 跟踪差停止。停止前 J1 输出参考为 0.181656 rad、实测为 0.131800 rad；下一拍候选输出与实测的差约 0.050116 rad，超过原 0.05 rad 边界。此时握把按下、输入有效，手柄姿态年龄约 20.2 ms。依据见[本次诊断记录](../../../validation/vr_position/tracking_drift_diagnosis.json)。

原因是旧减速函数在差值超过 0.04 rad 后，只让参考速度匹配电机报告的速度，没有继续收小差值。位置与速度反馈的量化差异、有限减速度使偏差逐渐累积。现在保留误差修正的正负号：偏差超出软规划范围后，参考速度会降到实测速度以下，等待实际位置追上；不反向追赶旧目标，也不跳变输出位置。硬停止边界、速度及加速度配置不变。

本次完整 587 拍实测反馈回放在修改前于第 587 拍复现 `TRACKING_ERROR`，修改后保持 `TELEOP`，最大跟踪差约 0.043547 rad。43 项 C++ 状态机/轨迹测试及 2 项原生位控测试通过，新增覆盖持续速度偏差、六轴正反向移动及短暂停顿、夹爪速度偏差。更新已编译安装。回放固定使用记录中的反馈，不等同于新代码的实机运动验收；仍需重启位控 launch，重新 `prepare`、`arm` 后复测。

**2026-10-09 首次反馈减速实现（后续修正见上文）：**full 位控新增 `vr_tracking_governor: true`。控制器根据输出参考与实测位置的差值、实测速度及允许加速度，提前降低正在追赶电机的那一轴的速度；电机恢复跟踪后连续加速，无需重新握把或 ARM。这种等待仍属于 `TELEOP`，诊断 `controller=TRACKING_LIMITED` 表示正在依据反馈减速，不是停止。

六轴以原 0.05 rad 跟踪故障边界的 80%（0.04 rad）作为减速规划范围，夹爪对应 0.0032 m；会依据相对速度提前留出制动距离，不是等到范围边缘才硬截断。速度跟得上时仍允许六轴 1.0 rad/s、夹爪 0.003 m/s，输出加速度限制保留。测量突然大幅变化、超出剩余误差范围、关节越界、追踪失效、反馈丢失或电机故障仍停止；严重跟踪误差不会被无条件自动重新使能。该策略处理短暂滞后和慢响应，不能替代电机位置环参数或机械问题的排查。

这次修复使用 `1791512218917917883_commission_follow.jsonl` 中的 26 拍真实反馈/目标回放，以及启动迟滞、慢响应、编码器量化、恢复和反向运动模型验证。回放固定使用原来测得的反馈，不能当作修复后的实机轨迹。重启方式仍为下方三个终端指令。

该回放原先触发 `TRACKING_ERROR`，修复后 26 拍保持 `TELEOP`，最大输出跟踪差约 0.036749 rad，未跨过原 0.05 rad 停止边界。39 项状态机/轨迹回归、2 项原生位控测试、49 项相关 Python 测试，以及[完整 ROS 模拟](../../../validation/vr_position/tracking_governor_mock_report.json)通过。理想反馈的模拟轨迹峰值仍约 0.624 rad/s，证明没有重新引入之前 0.196 rad/s 的固定瓶颈；慢响应模型的结果与理想模拟分开验证。更新已编译安装，模拟已退出，尚未实机复测。详细配置、证据和限制见[本轮实现记录](../../../validation/vr_position/tracking_governor_implementation_report.json)。

**2026-10-09 连续跟踪修复（当前 full 行为）：**上层现在发送附近的关节目标，由 C++ 统一生成实际输出轨迹。`position_profile.py` 中 `joint_goal_horizon=0.25` 为逆解提供足够的目标距离；它不是等待 250 ms，也不预测未来手势，目标仍随每次新反馈更新。六轴速度上限维持 1.0 rad/s、加速度 3.0 rad/s²，夹爪维持 0.003 m/s；TCP 目标的平移/旋转限速仍使用实际控制周期。

完整位控控制器启用 `vr_trajectory_goals: true`，区分“尚未执行的目标”和“当前电机输出参考”。0.03 rad 的上层超前范围不再夹紧未来目标；控制器生成的每拍输出仍受速度、加速度、关节范围及六轴 0.05 rad / 夹爪 0.004 m 实际跟踪误差检查约束。输入失效、松手、手动暂停和失能行为保留；trial/阻抗入口继续使用原先的参考方式。若电机本身明显滞后，仍会停止并报告跟踪误差，不能把软件输出提速等同于实机性能已经验收。

此前逻辑把每拍最多 0.008 rad 的小步目标再次作为减速终点，配置 1.0 rad/s 时输出最高约 0.1964 rad/s；[10 月 9 日实机诊断](../../../validation/vr_position/follow_lag_diagnosis_20261009.json)确认了这个瓶颈。下面 10 月 8 日的“0.03 rad 超前限幅”及 0.1975 rad/s 模拟记录是历史结果，不代表本版 full 的控制方式。J5 越界约 0.005° 的末尾故障属于独立的边界问题，本轮没有扩大其行程。

**本轮验证与使用：**已构建安装，无需另外编译；退出旧位控 launch，再按下面三个终端的原指令启动，并重新 `prepare`、`arm`。34 项 C++ 状态机/轨迹测试通过，覆盖六轴达到 1.0 rad/s、夹爪达到 0.003 m/s、到点减速、反向、执行器不动时的跟踪误差停止，以及高速松手、断流和失能。128 项 Python 用例的最新结果通过；其中一次全量运行仍按旧夹爪小步语义断言，更新该断言后 48 项相关用例全部通过。完整 [ROS 模拟](../../../validation/vr_position/continuous_mock_report.json)在相同 J5 目标移动 0.3 rad / 手柄目标斜坡 0.5 s 的测试中达到 0.624 rad/s，末端最终位置和姿态误差通过 2 mm / 0.035 rad 验证。模拟运行于域 86、无效 CAN 路径，已退出，没有使能实机。

扩大控制器回归时，另外两项捕获位置的阻抗动力学测试失败；在 `/tmp` 编译移除本轮控制器改动的副本后，失败项及数值完全相同，未将其算作通过。完整证据、修改文件和剩余实机验证项见[实现记录](../../../validation/vr_position/continuous_tracking_implementation_report.json)。

## 完整位控（用户指定，2026-10-08）

用户明确要求取消小范围调试约束，并选择“完整控制：末端平移、旋转和夹爪”，随后要求简化保护并自行实机验证。快捷启动顺序如下，每个终端先进入本项目目录。

**当前完整位控已按 Piper 参考项目精简：关闭 CAD 网格碰撞检查，不再受折叠姿态的 ±2° 碰撞例外框限制；IK 数值求解失败时保持控制器当前参考，下一帧继续求解。** 该模式不提供自动自碰或环境避障。关节/夹爪行程、速度/加速度、跟踪偏差、输入和反馈超时、电机故障停止保留。原 trial/阻抗入口仍执行 CAD 检查。对比依据见 [Piper 控制链路对比](../参考方案/VR_PIPER_CONTROL_COMPARISON.md)。

终端 1，启动实机网关：

```bash
cd /home/alce/Code/A6-Y-G
bash scripts/run_vr_gateway.bash --profile real --port 8014
```

头显打开 `https://10.200.1.65:8014`，进入 VR 并先松开右手侧握把。

终端 2，启动完整位控（六轴平移/旋转与夹爪）：

```bash
cd /home/alce/Code/A6-Y-G
bash scripts/run_vr_position_full.bash
```

终端 3，使能原位保持，再接管：

```bash
cd /home/alce/Code/A6-Y-G
bash scripts/vr_position_control.bash prepare
sleep 1
bash scripts/vr_position_control.bash arm
```

`prepare` 应返回 `accepted=True, state=HOLD`，`arm` 应返回 `accepted=True, state=ARMED_HOLD`。ARM 若拒绝，先用 `bash scripts/vr_position_control.bash status` 查看状态；保持右手柄追踪有效、侧握把松开，并等待机械臂稳定后重试。启动脚本不自动 ARM。

完整位控已启用 `vr_auto_rearm_after_release: true`。首次启动仍需上述 `prepare`、`arm`；同一手柄连接短暂追踪无效或输入超时后，停止并保持当前位置，追踪恢复、机械臂稳定且侧握把松开时自动回到 `ARMED_HOLD`。再按下握把即从当时实机位置重新接管，无需再回电脑执行 `arm`。一直按住握把恢复追踪时保持暂停，需松开再按；手动 `pause`、故障失能、`disarm`、更换输入会话/重启网关不自动恢复。原受限配置和阻抗入口保持手动 ARM 行为。

该变更已编译安装，并通过 27 项 C++ 许可状态机测试、35 项 Python 回归，以及[完整链路模拟](../../../validation/vr_position/auto_rearm_mock_report.json)。模拟覆盖 ARM 后位姿无效、跟随中输入中断、松开后无 ARM 命令自动恢复、重新抓取、手动暂停保持和 DISARM；没有重新使能实机验证。更新后需重启控制 launch 及使用新环境的工具终端。

2026-10-08 用户再次报告“不动”后，从[本次实机记录](../../../validation/vr_commission/1791453390170711540_commission_follow.jsonl)定位到另一处问题：PREPARE、ARM 和握把接管均已成功，但接管后约 **80 ms** 就进入 `STOPPING / TRACKING_ERROR`，随后保持 `PAUSED`。当时 J2 为 −0.006676 rad（约 −0.3825°），属于已接受的零位容差；Bridge 使用稍早的反馈生成参考，C++ 目标已向名义行程内推进，旧参考被错误当成继续向容差外运动。此次不是未按到握把，也不是首先由头显失联触发；该类停止不属于追踪恢复自动接管的条件。

本次已修复并编译安装：

- 对零位容差内、滞后量能由反馈年龄和既有速度上限解释的参考，投影到当前边界保持，不扩大向外行程。
- 关节和夹爪制动纳入当前控制拍的位移，避免接近名义限位时多走一拍后误报越界。
- 慢计算之后不增大下一次 IK/夹爪运动步长，避免计算时间与路径长度相互放大；时间戳和通信超时检查仍使用真实时间。
- 控制终端会打印实际停止原因，例如 `VR STOPPING: TRACKING_ERROR`；诊断保留该原因，避免仅显示 `PAUSED: following`。

验证：29 项 C++ 状态机测试、36 项 Python 回归通过；以本次日志中**实际折叠角度及夹爪零位**作为模拟起点，平移、旋转、夹爪、松手停止、输入丢失后松手自动恢复、手动暂停与 DISARM 全部通过。此前默认展开姿态的模拟没有覆盖这个零位边界问题。报告：[修复记录](../../../validation/vr_position/zero_lag_implementation_report.json)、[折叠起点模拟](../../../validation/vr_position/zero_lag_folded_mock_report_v3.json)。本轮未启动真实机械臂；仍需用户按上面的三个终端命令重启后验证实机运动。

网关 `right events` 增长只证明事件到达；`pose_age` 持续增长表示没有新的有效位姿。追踪丢失时停止是预期行为，恢复后先松开侧握把，再按住接管。若仍不动，优先查看位控终端的 `VR STOPPING` / `VR FAULT` 原因及 `bash scripts/vr_position_control.bash status`，不能仅凭网关的 `grip=True` 判断电机正在跟随。

**18:22 实机复测后的反馈延迟修复：**[新记录](../../../validation/vr_commission/1791454925421861367_commission_follow.jsonl)中未再出现 `TRACKING_ERROR`，并观察到 7 次自动恢复。但跟随中出现 `INVALID_REFERENCE`：实际电机落后参考几毫弧度时，重复检查实际位置到参考位置的路径使计算达到约 25 ms；同一线程的定时任务又可能先于反馈回调执行，继续使用上一轮的旧反馈。参考 1566、1568 在发布时的反馈年龄估计为 50.84、54.01 ms，已经超过既有 50 ms 有效期。

现已改为收到新控制器反馈后立即计算参考；定时器只负责反馈缺失诊断。碰撞检查保留每对连杆最近 128 个已查询距离，每次复用仍扣除保守运动距离并验证原采样间隔及裕量，不以“之前安全”直接跳过新路径。发布记录新增实际发布时间，便于直接计算反馈年龄。输入、反馈及驱动超时未放宽。

本轮证据：

- VR 包完整 Python 回归 **79 项通过**，包括实机滞后路径缓存、过期反馈、几何边界及调度回归。
- 对本次记录的 269 个 TELEOP 计算拍离线重算，超过 0.2 ms 的距离查询由 3395 次降至 307 次，最高计算时间由 24.84 ms 降至 13.90 ms；这是离线计时，不保证每拍都满足 8 ms。
- 在隔离 ROS 域中人为加入 **26 ms** 计算延迟，旧调度 102 条有效参考中有 **45 条**在发布时超过 50 ms，最高 68.13 ms；新调度 101 条中 **0 条**超时，最高 38.03 ms。报告：[旧调度](../../../validation/vr_position/feedback_dispatch_before.json)、[新调度](../../../validation/vr_position/feedback_dispatch_after.json)。
- 从本次实际折叠角度出发的[完整模拟](../../../validation/vr_position/feedback_latency_mock_report.json)通过平移、旋转、夹爪、松手保持、失联恢复、手动暂停和 DISARM。先前模拟硬件直接跟上指令，未覆盖实机滞后带来的重复计算，本次额外使用了实际记录和延迟注入验证。

已安装更新并退出模拟进程。本轮没有使能真实机械臂，真实跟随仍由用户重启上面的三个终端命令验证。[本轮修复记录](../../../validation/vr_position/feedback_latency_implementation_report.json)。

**18:39 实机复测及 Piper 简化：**[最新记录](../../../validation/vr_commission/1791455954068435363_commission_follow.jsonl)中 998 条有效跟随参考的最大发布反馈年龄为 30.03 ms，之前的延迟问题未再出现。三次 CAD 停止都发生在 J5 离首次 PREPARE 超过约 2° 时；改为按相关关节判断仍会被 `link1/link5` 拒绝，不能靠扩大模型容差声称碰撞已解决。因此按用户要求，full 原生位控明确设置 `geometry_checking=False`，不再构建/运行 CAD 检查器；启动终端和 `/arm6/vr/status` 会明确显示已关闭。

最新记录也含一次 10.9 ms 内 14.2 cm 的追踪跳变。此类不连续输入仍停止并等待松手重新接管，不作为真实手部运动执行。IK 求解异常或非有限候选在 full 中保持上一次控制器参考（不是滞后的电机反馈），冻结目标积累，下一帧自动重试；trial/阻抗保持原行为。重放本次真实失败帧，旧配置仍可复现模型停止，新配置可越过原 2° 门槛，且单步仍满足原速度和关节范围。详见[本次审计](../../../validation/vr_position/latest_trace_analysis.md)。

用户已在明确获知 CAD 检查关闭后回复“授权放宽全部”。此版本已安装：95 项完整 Python 回归通过，随后新增格式异常 IK 回退并通过 3 项定向测试。[扩展模拟](../../../validation/vr_position/piper_full_mock_report.json)从本次折叠起点持续 TELEOP 6.00 秒、751 个样本，J5 实际/参考离首次 PREPARE 都达到 3.955°；平移、旋转、夹爪、松手、失联恢复、手动暂停和 DISARM 均通过。模拟进程已退出；本轮未启动真实机械臂。启动命令仍为本文顶部三个终端，无需另外编译。[实现与验证记录](../../../validation/vr_position/piper_simplification_implementation_report.json)。

当前完整配置取消：CAD 网格碰撞硬检查、5 mm 局部工作球、1°/2°会话位移限制、单拍计算超过 8 ms 锁停、IK 暂时不可达持续 0.5 秒锁停。8 ms 仅作性能记录；IK 不可达时输出经过限幅的可行参考或保持位置，返回可达目标后可继续。保留关节/夹爪行程、速度/加速度、反馈和参考超时、手柄失联/追踪跳变停止、跟踪偏差保护、显式接管和故障失能；C++ 控制器与驱动的独立超时不变。目标更新仍请求 125 Hz，实际频率取决于反馈与计算耗时。

**19:27 提速后的反复短暂跟随修复：**[这次实机记录](../../../validation/vr_commission/1791458838072132602_commission_follow.jsonl)已加载 TCP 1.8 m/s、7.5 rad/s 和六轴 1.0 rad/s。15 段 TELEOP 大多仅维持 0.16～0.6 秒，其中 13 次因 `reference_lead_exceeded` 停止；有效跟随参考发布时的最大反馈年龄为 1.016 ms。首个故障片段中，现有参考最大跟踪差约 0.02572 rad，下一步 IK 候选与实际位置相差 0.030047 rad，刚超过 0.03 rad 就锁住跟随，导致电机尚未跟上便停止。

完整位控启用 `limit_reference_lead=True` 和 `bounded_ik=True`：逆解在求解时同时考虑每个关节的行程、单步速度和实测位置附近的超前范围，将受限关节固定或减速，并允许其他关节继续配合逼近末端目标。某一轴已经没有超前余量时，不再将整个六轴步长一起压到零。触及跟踪边界时清除积累的中间 TCP 目标，避免追赶旧目标。诊断显示 `limited_reference_lead` 表示正在限幅等待，并非 PAUSED。若旧参考已因实测位置变化而超出软限幅区间，后续步长只能保持或缩小已有偏差，不瞬间跳变参考；C++ 的 0.05 rad 跟踪故障检查仍独立生效。trial/阻抗入口保留原来的逆解和超前锁停行为。

[离线审计](../../../validation/vr_position/lead_trace_analysis.json)重建的 13 个故障拍均可在旧策略下复现，新策略均保持有效参考且不超过 0.03 rad；旧日志未记录精确锚点，重建使用首个 TELEOP 反馈推定锚点，不能据此宣称后续实机轨迹已验证。新日志已补充锚点、接管时间和已接受序号。[Python 回归](../../../validation/vr_position/lead_limit_python_tests.xml)116 项、[C++ 许可状态机回归](../../../validation/vr_position/lead_limit_cpp_tests.xml)29 项通过，新增静止后缓慢跟踪的 300 拍反馈模型测试；[完整模拟](../../../validation/vr_position/lead_limit_mock_report.json)通过跟随、旋转、夹爪、松手、失联恢复、手动暂停和失能。修复已安装，模拟已退出；本轮未启动实机，需按本文三个终端步骤重启后验证实际连续跟随。

**夹爪不能明显朝上/朝下的修复：**后续[实机记录](../../../validation/vr_commission/1791463389447976492_commission_follow.jsonl)确认俯仰旋转输入已到达，末端目标确实包含朝上和朝下。先前整组缩小步长的方法会让已经达到超前阈值的基座或肩部拖住手腕；从记录选取上下俯仰目标、将对应慢轴置于规划边界后，两个旧逻辑测试均出现 J4 步长为零。当前 full 的有界阻尼最小二乘逆解分别处理各轴限制，上下两个回归均恢复手腕俯仰，其他轴继续受到原来的行程、速度及跟踪约束。

在“一个慢轴保持不动、其他轴有跟踪滞后”的离线 125 拍测试中，旧策略两个方向的 J4 位移均为零，新策略朝下约 -9.03°、朝上约 +56.84°。这些是构造的模型测试，不是新的实机测量。另有模型见证姿态证明现有关节范围能容纳工具轴竖直向上和向下，并验证逆解能收敛到对应位置与姿态；这不代表任意 TCP 位置都能保持不动而转成竖直，也没有扩大机械行程。肩部、肘部通常需要共同调整。

本轮 [126 项 Python 回归](../../../validation/vr_position/pitch_python_tests.xml)及[完整 ROS 模拟](../../../validation/vr_position/pitch_mock_report.json)通过；上下俯仰专门用例见 [test_pitch_tracking.py](../../../src/a6_y_vr_teleop/test/test_pitch_tracking.py)，配置只在 full 原生位控开启。已安装并退出模拟，没有启动实机，实际效果需重启控制 launch 后复测。

暂停保持或撤去输出：

```bash
bash scripts/vr_position_control.bash pause   # 停止跟随，保持位置
bash scripts/vr_position_control.bash disarm  # 撤去电机输出，需要机械支撑
```

快捷启动与下面的完整参数等价：

```bash
cd /home/alce/Code/A6-Y-G
bash scripts/run_vr_position.bash mode:=commission stage:=follow \
  motion_range:=full operator_trial:=true
```

网关仍使用 8014；进入 VR 后先松开右手**侧握把键**。在另一个已加载环境的终端：

```bash
source scripts/environment.bash
a6y_load_install
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
python3 tools/vr_control.py --controller position prepare
python3 tools/vr_control.py --controller position status
# 确认保持稳定、prepare_ramp_complete=true、gripper_control=true。
python3 tools/vr_control.py --controller position arm
```

按住右手柄侧面、中指操作的握把键后，手柄平移控制 TCP 位置，转动手柄控制 TCP 姿态。食指扳机控制夹爪：压到底为闭合，释放为张开。每次接管先保持真实开度，扳机相对接管值变化超过 0.02 后才开始响应；开合目标经过限速，不会直接跳到端点。松开侧握把后，六轴和夹爪一起停止到捕获的实际位置；失联进入 PAUSED，同一连接追踪恢复并松开握把后自动恢复接管许可。

完整配置保留 URDF 关节范围、±2°零点读数容差、目标跟踪偏差、通信新鲜度和故障失能，不执行 CAD 碰撞检查。2026-10-08 按用户要求，将当时生效的 TCP 平移 0.18 m/s、旋转 0.75 rad/s 上限各提高 10 倍：当前为 **1.8 m/s、7.5 rad/s**。平移比例沿用用户设定的 **1.0**。六轴 IK、控制器和电机速度上限同步由 0.1 调到 **1.0 rad/s**，参考加速度由 0.3 调到 **3.0 rad/s²**；实际超速停机阈值为 **1.25 rad/s**。这是软件上限，不保证实际 TCP 速度达到该数值或实机运动恰好快 10 倍；实际运动还受姿态、IK、跟踪偏差及电机响应影响，提速后的硬件跟随与停距尚待实测。

完整位控使用独立的 `a5_vr_position_full_limits.yaml`，避免共享运动限值中的 0.5 rad/s 把 IK 截断，也不改变 trial/阻抗的共享配置。修改速度时需同步 `position_profile.py` 的 full 分支和检查值、`a5_vr_position_full_controllers.yaml`、`a5_vr_position_full_motors.yaml`、`a5_vr_position_full_limits.yaml`；仅改 `a5_vr_real.yaml` 的速度仍会被 full 分支覆盖。安装配置后重启控制 launch 并重新 PREPARE/ARM 生效，不向正在运行的控制进程热更新。

夹爪不参与这次提速：目标范围 [-0.03,0] m，速度 0.003 m/s、加速度 0.01 m/s²、实际速度保护 0.01 m/s、最大跟踪偏差 0.004 m。电机单位转换仍使用原 0.0239 m/rad，初始零位正小偏差只允许保持或向名义行程返回。

原生位置环使用固件内部增益/电流限制，软件不发送重力或阻抗 effort。本次提高软件速度上限不代表对之前 J6 速度异常的原因修复或电机额定能力认证。旧小范围跟随在追踪无效后进入 PAUSED，约 65 秒后 J6 达到 0.10989 rad/s 并触发 0.1 rad/s 试验速度保护；随后七电机失能、进程已退出，记录见 [受限跟随结果](../../../validation/vr_position/restricted_follow_result.json)。

`zero_collision_tolerance` 仅用于仍执行 CAD 检查的 trial/阻抗入口；当前 full 快捷脚本不再传该参数。夹爪使用现有 30 mm 工作行程，不扩大到之前 -40.77 mm 的越界读数。完整运行不继承旧力控或六轴只读夹爪的验收结果；下列早期记录中的碰撞检查描述属于当时配置。

配置为 `a5_vr_position_full_controllers.yaml`、`a5_vr_position_full_motors.yaml`、`a5_vr_position_full_limits.yaml`。完整模拟可用 `mode:=mock stage:=follow motion_range:=full`，配合域 86 的 `verify_vr_real.py --mock --controller position`。

本次 10 倍速度配置的离线验证：103 项 Python 测试通过，两个 ROS 包构建安装完成。[提速模拟报告](../../../validation/vr_position/speed_10x_mock_report.json)使用 `--mock --controller position --exercise-full-range --exercise-speed`，模拟 J5 接管后移动 17.19°，测试段关节峰值速度 0.1975 rad/s，超过旧 0.1 rad/s 上限；平移、旋转、夹爪、松手、失联恢复、手动暂停及失能全部通过。该轨迹没有达到 1.0 rad/s，不把上限设置或模拟结果当作实机最大速度证明。模拟使用域 86 和无效 CAN 路径，未访问实机或头显，测试进程已退出。[Python 测试记录](../../../validation/vr_position/speed_10x_python_tests.xml)。

以下为提速前的完整配置证据：

- Python 回归 80 项通过；[完整模拟报告](../../../validation/vr_position/full_mock_report_v3.json)验证平移/旋转/夹爪以及释放、失联、重新接管和失能。模拟不作为实机运动证明。
- [七电机实机保持报告](../../../validation/vr_position/full_physical_hold_report.json)：15.06 秒，六轴最大位置误差 0.0437°，夹爪最大偏差 0.0092 mm；六轴有一拍速度 0.03663 rad/s 超过 0.03 rad/s 静止验收线，因此整体 `passed=false`。DISARM 后七电机失能已观察确认。报告因原工具的 NumPy 布尔序列化错误从原始记录重建，未为补报告重复动作。
- 真实 Quest 输入恢复后已成功 PREPARE、ARM 并短暂进入 TELEOP，但用户确认实机没有可见运动，不能作为有效跟随验收。记录包含一次约 160 ms 输入间断触发暂停，以及一拍 9.24 ms 回调超过旧 8 ms 预算触发暂停。原始数据：[完整实机跟随记录](../../../validation/vr_position/full_physical_follow_bridge.jsonl)。
- 已对接管时 link4–link6 重复网格查询增加逐采样点的保守距离缓存复用，距离、采样精度及路径边界不变。同一记录片段离线重放的最高计算时间由 10.51 ms 降为 4.75 ms，见 [离线计时](../../../validation/vr_position/folded_timing_after.txt)；这是离线结果，实机动态效果由用户后续验证。单拍 8 ms 锁停也已按用户要求取消，超时原始时间戳仍交由 C++ 检查。
- 交还用户操作前已 DISARM 并退出助手启动的实机 launch 和网关；[停机反馈](../../../validation/vr_position/full_follow_shutdown.json)记录 30 帧七电机全部失能。
- [简化后完整模拟](../../../validation/vr_position/simplified_full_mock_report.json)通过平移、旋转、夹爪、松手保持、输入丢失暂停、恢复后重新 ARM 和 DISARM。记录中实际出现 9.52–14.09 ms 的回调，均仅记录 `stop_requested=false`，未再触发旧 8 ms 锁停；150 ms 失联暂停仍验证通过。此测试没有访问实机。

## 受限位置控制入口（早期配置，2026-10-08）

构建方法见下文。实机网关继续使用 8014；原 8012 网关对应虚拟模式，不会向实机发送输入。

```bash
cd /home/alce/Code/A6-Y-G
bash scripts/run_vr_gateway.bash --profile real --port 8014
```

Quest 打开 `https://10.200.1.65:8014/?ws=wss%3A%2F%2F10.200.1.65%3A8014`（电脑 IP 改变时替换两处），进入 VR 并先松开右手握把。每个端口只启动一份网关。

另一个终端启动位控。当前折叠姿态使用已由用户确认的局部模型碰撞容差：

```bash
bash scripts/run_vr_position.bash mode:=commission stage:=follow \
  operator_trial:=true zero_collision_tolerance:=true
```

启动后仍为 DISABLED。现场保持支撑后，在第三个终端执行：

```bash
source scripts/environment.bash
a6y_load_install
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
python3 tools/vr_control.py --controller position prepare
python3 tools/vr_control.py --controller position status
# 确认 HOLD 稳定、prepare_ramp_complete=true 且右手握把释放，再接管。
python3 tools/vr_control.py --controller position arm
```

`prepare` 捕获实际六轴位置并使能电机内部位置闭环，等待 0.5 秒后才允许 ARM；这里没有力矩渐增。握住右手握把才开始相对跟随，松手后固定停止参考并等待速度降低。失联后进入 PAUSED，不自动恢复；恢复输入、松开握把后重新 ARM。暂停命令仍保持位置，DISARM 则撤去电机输出：

```bash
python3 tools/vr_control.py --controller position pause
# 机械臂已被托住后：
python3 tools/vr_control.py --controller position disarm
```

本阶段固定 125 Hz 位置参考更新、平移比例 0.2、TCP 5 mm/s、局部 TCP 半径 5 mm；关节参考和电机速度上限均为 0.03 rad/s，参考加速度 0.1 rad/s²，实际速度保护 0.1 rad/s。相对首次 PREPARE 的累计参考/实际位移限制分别为 1°/2°。电机内部位置环的频率由固件决定，125 Hz 不是其内部伺服频率。内部电流/力矩限幅由固件决定，原 effort 控制器的 N·m 上限不适用于 `pos_vel`，不能宣称其已经得到验证。

位控专用电机配置保留 ID、sign、offset、比例。J2/J3 的驱动命令边界包含已确认的零位 ±2° 容差，使首次负小角度捕获不被裁成零；不整体平移坐标。C++ 位置插件复用原反馈检查与许可状态机，在进入动力学计算前直接输出位置和速度上限，所有 effort 命令为零。Bridge 和本地命令使用独立的 `/arm6/a6_y_vr_position_controller/` 话题。

其他入口及测试：

```bash
bash scripts/run_vr_position.bash                         # 只读，不能 PREPARE
bash scripts/run_vr_position.bash mode:=mock stage:=follow # 隔离域 86，不打开 CAN
bash scripts/run_vr_position.bash mode:=commission stage:=hold # 实机仅保持，拒绝 ARM
# mock 测试（先 source 并加载 install）：
export ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1
python3 tools/verify_vr_real.py --mock --controller position \
  --output validation/vr_position/mock_new.json
```

位控当前提供只读、mock 和受限 commission；尚不开放完整 `mode:=execute`，也不继承旧力控验收标签。配置为 `a5_vr_position_controllers.yaml` 和 `a5_vr_position_motors.yaml`。

本轮构建成功，Python 78 项、C++ 85 项选定测试通过；原有两项固定目标力控压力测试仍排除，未宣布完整 C++ 套件全过。位控 mock 通过接管、移动、松手、失联、重新许可和失能，观测约 125 Hz。实机确认六轴电机进入原生模式 2，夹爪只读；保持 15.040 秒通过，最大位置偏差 0.000763 rad（0.0437°），观测最大速度 0.02198 rad/s，参考固定；DISARM 后持续确认失能。它是当前位置、有支撑下的保持结果，不是动态跟随或全工作空间验收。

随后启动受限位控跟随入口，但头显停止产生新位姿，尚未执行该入口的 PREPARE/ARM，保持 DISABLED；**真实 VR 动态位控跟随仍待测试**。报告：[总记录](../../../validation/vr_position/implementation_report.json)、[实机保持](../../../validation/vr_position/physical_hold_report.json)、[mock](../../../validation/vr_position/mock_report.json)。

## 原关节阻抗路线与历史记录

当前已实现软件入口，**支撑下使能、15 秒保持及软件 DISARM 已实测通过**。常规 `execute` 仍要求完整验收；用户明确选择直接试运行时，可使用下文 `operator_trial` 受限跟随入口。J3 总力矩上限按用户 2026-10-08 的确认统一为 20 N·m。基座方向、关节对应及正负方向已采用用户现场确认；零点和角度比例测试按用户要求跳过。夹爪行程、TCP、真实 Quest 追踪、独立停机/命令丢失、运动停距和空间检查仍未完成。

## 构建

```bash
cd /home/alce/Code/A6-Y-G
source scripts/environment.bash
colcon build --base-paths src \
  --packages-up-to a6_y_vr_teleop --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
a6y_load_install
```

新增 `a6_y_teleop_msgs` 提供固定长度消息；原控制器默认 `reference_mode=captured`。独立 VR 配置选 `external_vr`、125 Hz、Ki=0，原三个控制 YAML 的增益没有覆盖。

## 模式、域和输入

| 模式 | 启动 | ROS domain | UDP | 电机输出 |
|---|---|---|---|---|
| 原虚拟 | `scripts/run_vr_rviz.bash` | 84 | 15150 | 不加载硬件 |
| mock | `scripts/run_vr_real.bash mode:=mock` | 86 | 15154 | 内存模拟；绝不打开 CAN |
| 只读实机 | `scripts/run_vr_real.bash` | 0，或 `A6Y_REAL_ROS_DOMAIN_ID` | 15153 | `auto_enable=false`；拒绝 prepare |
| 支撑下保持验收 | `scripts/run_vr_real.bash mode:=commission stage:=hold` | 同实机 | 15153 | 初始失能；允许本地 prepare，C++ 拒绝 arm |
| 小范围跟随验收 | `scripts/run_vr_real.bash mode:=commission stage:=follow` | 同实机 | 15153 | 前置检查通过后允许本地 prepare/arm |
| 实机执行 | `scripts/run_vr_real.bash mode:=execute` | 同上 | 15153 | 调试记录通过后，仍须本地 prepare 和 arm |

ROS 限定 localhost；只有网关 HTTPS/WSS 面向局域网。新入口检查已有 `/arm6/controller_manager` 和串口占用。旧控制 launch 应在机械臂已支撑后退出。

实机网关使用独立协议 schema 2，明确 `purpose=real, source_kind=live`：

```bash
# 如原虚拟网关仍占用 8012，可用 8014。
scripts/run_vr_gateway.bash --profile real --port 8014
```

Quest 浏览器打开 `https://<电脑局域网 IP>:8014`，允许证书/VR 访问，进入沉浸模式，先松开右手握把。沿用已生成的 TLS 证书；主机 IP 改变时按虚拟说明重新生成。原虚拟网页对应的网关仍只发往 15150，不会切换成实机输入。

`tools/vr_replay.py` 拒绝实机保留端口 15153；schema 1、test/demo/replay 不会被实机桥接接受。端口和来源标签是防误操作措施，不是本地主机安全认证。

## 只读预览

```bash
scripts/run_vr_real.bash mode:=readonly
```

RViz 的实际模型来自 `/arm6/joint_states`；透明参考模型使用 `/arm6/vr/reference_joint_states`、TF 前缀 `vr_ref/`。绿色箭头是实际 TCP，黄色箭头是限制后的目标。握住手柄只改变参考预览；该模式没有控制器输入话题的发布器。

可保存记录：

```bash
scripts/run_vr_real.bash mode:=readonly \
  record_file:=/tmp/a5-vr-readonly-new.jsonl
```

文件必须不存在。JSONL 包含配置哈希、原始输入、真实反馈、模式与已采纳参考。反馈过期后停止预览；不把旧 q 或夹到限位后的 q 当成有效实测。

另一个终端可执行纯读取验收：

```bash
source scripts/environment.bash
a6y_load_install
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
python3 tools/verify_vr_real.py --duration 15 \
  --output validation/vr_real/readonly.json
```

只读采集也会发送驱动状态查询；驱动退出时发送失能帧。它不使能电机或发送跟随目标。

## 零位容差（2026-10-08）

按现场要求，六个旋转关节允许机械零位附近有 **±2°（±0.034906585 rad）** 的读数误差。
配置位于 [`a5_motion_limits.yaml`](../../../src/a6_y_controllers/config/a5_motion_limits.yaml) 的各关节 `zero_tolerance`；
夹爪为直线坐标，按当前 `position_scale=0.0239 m/rad` 将电机端 ±2° 换算为 **±0.000834267 m（±0.834 mm，单侧坐标）**。

这是反馈接受容差：`a5_motors.yaml` 中的 `offset` 不变，反馈、FK、日志和 RViz 仍使用原始映射值。不会把容差内的数值归零，也不是阻抗控制的误差死区。

- J2、J3 的名义下限是 0，允许实际反馈最低到 −2°；夹爪的名义上限是 0，允许实际反馈最高到 +0.834 mm。
- J1、J4、J5、J6 的零点在正常行程内部，本来就允许 ±2°；它们两端的机械行程限制不扩大。J2/J3 的另一端上限及夹爪张开下限也不扩大。
- 接管首个参考等于实际姿态。对容差内但超出名义行程的锚点，后续参考可以保持或限速向正常范围移动，不能命令它继续向外移动；进入正常范围后按名义限位运行。容差不会主动驱动关节回零。
- Bridge 和 C++ 控制器共同检查；控制器参数 `vr_zero_tolerance` 与上述配置必须一致。`vr_feedback_tolerance=0` 避免额外放宽非零端点。超出容差时拒绝 prepare；已使能时触发本地越限保护。

已记录的 J2 零位 **−0.2732°**、夹爪零位 **+0.3895 mm** 均在容差内。
另一次夹爪张开读数 **−40.7723 mm** 超过当前 **−30 mm** 运行下限，仍会被拒绝；零位容差不能代替张开行程/传动比例核对。
原始记录分别在 [`validation/zero_reference/`](../../../validation/zero_reference/) 和 [`validation/gripper_open_reference/`](../../../validation/gripper_open_reference/)。

本次修改只更新代码与配置，重新构建并重启对应 VR launch 后生效；不在线修改运行中控制器。现场标定、保持与停止验收仍按原流程完成，容差本身不代表这些检查已通过。

本轮已完成构建，Python **52 项通过**，C++ 所选 **77 项通过**（含 17 项接管状态机测试）。原有两项已知失败的固定目标压力测试未重跑，不计入这 77 项。验证使用离线计算与内存硬件，没有启动实机；预检确认 Bridge/控制器限位一致，现场验收状态仍为未完成。
报告：[Python](../../../validation/zero_tolerance/python_tests.xml)、[C++](../../../validation/zero_tolerance/controller_tests.json)、[预检](../../../validation/zero_tolerance/preflight.json)。

## 用户确认的标定范围（2026-10-08）

用户确认基座 Z 轴竖直向上、关节对应正确、正负方向正确，并明确要求不进行零点和角度比例测试。
[`a5_calibration.yaml`](../../../src/a6_y_controllers/config/a5_calibration.yaml) 保存了日期和原始说明：

- 基座标记为 `operator_confirmed`，由安装方向推得重力方向为基座 -Z；没有填写虚构的重力实测值或精度。
- 六个旋转关节和夹爪的对应及方向标记为 `operator_confirmed`；`zero_scale_test: skipped_by_user` 表示按要求跳过零点/比例采样。这部分不再阻塞预检。
- 电机 `sign`、`offset`、`position_scale` 及 ±2° 零位容差沿用当前配置，采样数组保持为空。

预检报告的 `calibration_evidence` 分别列出用户确认和跳过的测试。S0 报告的 `field_calibration_complete` 表示按当前约定处理完所需检查，`field_measurements_complete` 才表示全部采用实测完成；用户确认/跳过不会被报告为实测完成。
本次确认不涉及 VR 手柄坐标映射 `axis_mapping`；夹爪实际开度/行程、TCP、保持与停止、工作空间等检查仍单独保留。
更新后相关 Python 测试 33 项通过，预检缺项从 51 条减少到 43 条（其中 32 条仍为未登记快照）。本轮没有连接或驱动实机。
报告：[测试](../../../validation/calibration_user_confirmation/python_tests.xml)、[预检与用户决定](../../../validation/calibration_user_confirmation/preflight.json)。

## 输出前检查与本地操作

首次现场验收使用 `commission` 入口。`hold` 不要求“保持已经通过”，`follow` 不要求“停止试验已经通过”，从而可以先收集证据；正常 `execute` 仍检查全部验收结果。

机械臂保持支撑、旧 launch 已退出后，先启动仅保持模式：

```bash
scripts/run_vr_real.bash mode:=commission stage:=hold
```

另一个终端运行保持验收工具。它先核对七关节硬件身份、初始失能及 C++ 跟随禁用状态，再发送 PREPARE；不发送运动目标。当前力控配置为刚度 2 秒、重力补偿 5 秒渐增，工具等待控制器报告渐增完成后记录 15 秒。

```bash
source scripts/environment.bash
a6y_load_install
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
python3 tools/verify_vr_hold.py --duration 15 \
  --output validation/vr_commission/hold_new.json
```

文件名必须未被使用。默认测试结束后继续保持承重；使用 `pause` 可停止跟随许可并保持，托住后再 `disarm`。若机械臂在整个试验中始终有机械支撑，可以添加 `--disarm-at-end`，让工具在结束/失败后失能，等待新鲜硬件反馈并继续观察至少 0.25 秒 DISABLED 状态。退出时出现故障会使整项判定失败，即使保持阶段通过。故障可能直接触发本地失能；出现故障后先检查记录，不自动重新使能。

保持测试的暂定判据为：实际姿态相对固定参考误差不超过 1°、渐增后观测速度不超过 0.03 rad/s、反馈年龄不超过 0.05 s、无饱和，且参考不漂移。报告与原始 JSONL 一起保存；支撑下通过不等于无支撑承载已经验证。

完成保持记录、独立停机/命令丢失、真实 Quest 及方向、夹爪/TCP 和空间检查后，可进行小范围跟随验收：

```bash
scripts/run_vr_real.bash mode:=commission stage:=follow
```

需要先填写停止距离、关节转角与停止时间的允许值，但不要求已有停止试验结果。此模式固定为平移、TCP 参考速度 5 mm/s、每次握抓的 TCP 局部半径 5 mm、关节参考速度 0.03 rad/s、加速度 0.1 rad/s²。C++ 以本次控制器首次 PREPARE 的位置为基准，限制每轴参考累计变化 1°、实际变化 2°、实际速度 0.1 rad/s；松手、重抓和 disarm/prepare 都不重置该基准，改变试验初始姿态需重启入口。

两个验收阶段默认自动创建 `validation/vr_commission/` 下的 JSONL，保存运行模式、有效控制参数、文件哈希、真实反馈和许可状态。`stage:=hold` 的 Bridge 只做只读预览；即使另一个程序直接发 ARM，控制器也会拒绝。配置中的 `vr_ramp_gravity_on_prepare=true` 同样用于正常 VR 执行，原固定目标控制模式保留原来的重力行为。

离线演练可用 `mode:=mock stage:=hold`，配合 `verify_vr_hold.py --mock` 和域 86；dry_run 不模拟动力学，也始终报告物理电机未使能，因此不能作为实机保持或停止证据。

### 用户选择直接受限跟随（2026-10-08）

用户明确要求“直接开始实机控制跟随，再从控制中找到并且修复遇到的问题”，并确认头显就绪。此选择记录在 `validation/vr_live_trial/operator_request.json`。可以显式运行：

```bash
scripts/run_vr_real.bash mode:=commission stage:=follow operator_trial:=true
```

该选项只对 `commission/follow` 生效，默认关闭。它允许把尚未完成的头显方向、夹爪/TCP、停机/空间验收留到试运行中处理，既有记录仍保持 pending；不会将本次入口变成已通过完整验收的 `execute`。仍要求已确认的关节对应/方向、限位与力矩配置一致、已有支撑保持结果，以及现场确认的独立电源切断装置。

试运行保持上述 5 mm/s、5 mm 局部范围、关节累计参考 1°/实际 2°、实际速度 0.1 rad/s 的限制；只平移，夹爪不接入遥控。真实反馈检查、CAD 自碰检查、刚度/重力独立渐增、松手停止、输入失联暂停及本地故障失能继续工作。桌面和周围障碍物尚无实测配置，小范围约束不能替代环境避障。

控制器仍从 DISABLED 启动。使用实机网关、进入 VR 并松开右手握把后，在同一 ROS 域执行 `prepare`，等待状态中的 `prepare_ramp_complete=true` 并确认保持稳定，再执行 `arm`；握住右手握把后才开始相对跟随。不得把模拟或回放数据作为实机输入。每次启动自动记录有效参数、未完成项、真实输入和控制反馈，故障后先检查记录，不自动重新许可。

首次姿态检查在折叠状态报告 `mesh_clearance:link1_0:link4_0`；用户随后确认实物连杆之间留有间隙、没有接触，并明确要求增加零位附近碰撞容差。该确认保存在 `validation/vr_live_trial/folded_pose_confirmation.json`。这属于操作员确认的模型差异，不是已测得的毫米级间距或停距。

### 折叠零位附近的碰撞容差

按上述确认，可在受限试运行中显式启用：

```bash
scripts/run_vr_real.bash mode:=commission stage:=follow \
  operator_trial:=true zero_collision_tolerance:=true
```

它是局部碰撞对例外，不会改动关节反馈、offset、IK 坐标或力矩死区。规则如下：

- 仅作用于 `link1–link4`、`link1–link5`、`link2–link4` 三对已确认存在模型差异的连杆；其他碰撞对和已配置的桌面检查不变。
- J2、J3 都须在数值零位 ±2° 内。首次 PREPARE 前只允许静态姿态检查，不能凭此发送移动路径。
- PREPARE 后以 C++ 控制器首次记录的实际关节姿态为固定中心；参与检查的整条路径须同时落在六轴各 ±2° 的范围内。松手、重抓、DISARM/PREPARE 均不扩大或重置该范围。
- 离开这个区域后立即恢复三对连杆的普通碰撞检查，不再容忍其相交。实机参考累计 1°、实际累计 2° 及限速等保护仍独立生效。
- 默认关闭，只能通过受限试运行参数启用（或隔离 mock 演练）。正常 execute 不使用此例外。JSONL 和状态诊断记录正在应用容差的连杆对与固定中心。

因此，在该限定区域内无需仅为清除这些已确认的模型告警而强制展开机械臂。试运行仍应观察实物间隙；这项局部例外不能作为整机碰撞模型已经标定的证据。

本次实现构建通过，Python 69 项通过；隔离 mock 的 prepare、arm、跟随、松手、失联、重新许可、disarm 流程通过。实机入口已按该选项重启，当前折叠姿态通过带容差的检查；当时尚未收到真实头显输入，控制器仍为 DISABLED，未发送 PREPARE/ARM，不能把它写成物理跟随通过。报告：[测试](../../../validation/vr_live_trial/zero_collision_tests.xml)、[mock](../../../validation/vr_live_trial/zero_collision_mock_report.json)、[实机姿态检查](../../../validation/vr_live_trial/zero_collision_readiness.json)。

```bash
source scripts/environment.bash
a6y_load_install
python3 tools/vr_preflight.py --output validation/vr_real/preflight.json
```

退出码 2 表示尚不能启动 `execute`；报告中的 `commissioning_stages.hold/follow` 分别列出两个验收入口是否可启动及缺项。检查依据包括：

- `a5_calibration.yaml` 中的测量记录或明确的用户决定：基座/关节方向采用上述用户确认，零点/比例采样已跳过；夹爪开度和 TCP 检查仍需实际数据。不接受仅改一个 verified 标签。
- `a5_vr_commissioning.yaml` 中的操作员、日期、独立停机、命令丢失、真实 Quest、支撑下使能与停止试验记录。
- URDF、网格、标定、限位、驱动与控制器二进制、VR 参数快照的哈希一致。
- 工作空间与桌面高度已填写；间距覆盖模型/跟踪误差和实测停距。默认 0.5 mm 的 CAD 检查间距只用于软件验证，不能视为实物余量。

这些物理记录不能从仿真报告自动生成。支撑下保持/停机调试与 VR 自由空间运动是不同的现场步骤；不要把 mock 的通过报告填成物理停止证据。

完整预检通过后才启动正常 `execute`；`commission stage:=follow` 通过对应阶段检查后使用相同本地许可命令。在与实机相同 ROS domain 的终端运行：

```bash
python3 tools/vr_control.py status
python3 tools/vr_control.py prepare  # 已支撑、允许承重控制；进入 HOLD
# 等待 prepare_ramp_complete=true，确认保持稳定；保持右手握把释放。
python3 tools/vr_control.py arm      # 绑定当前新鲜 VR 会话
```

握住右手握把后 Bridge 请求 CAPTURE；控制器返回同周期实际 q，首个参考严格等于该锚点。随后才计算手柄的新位移，握手前的移动不补发。正常 `execute` 配置仅平移，比例 0.2、TCP 速度 0.02 m/s、关节参考速度 0.1 rad/s、加速度 0.3 rad/s²；`commission stage:=follow` 使用上文更小的验收限值。

松手 → STOPPING → ARMED_HOLD，可重新握住并重锚定。输入失联、丢边沿、工作空间/计算超时 → STOPPING → PAUSED，须松开后重新执行 arm。真实反馈失效、超速、越限或停止超时进入 FAULT，沿用控制器失能路径，不自动重启。

```bash
python3 tools/vr_control.py pause    # 撤销 VR 许可，保持承重
python3 tools/vr_control.py disarm   # 已托住机械臂后失能；不是普通松手暂停
```

Ctrl+C 会停用控制器并失能。实机运动阶段先松手/暂停、托住机械臂，再退出。主机或总线断开时软件不能保证补偿继续，须使用已核验的独立停止/支撑方式。

## 离线验证

```bash
# 终端 1：mock 不模拟 effort 导致的运动，只测试接口。
scripts/run_vr_real.bash mode:=mock use_rviz:=false

# 终端 2：脚本先核对七关节 dry_run，才发送模拟许可与输入。
source scripts/environment.bash
a6y_load_install
export ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1
python3 tools/verify_vr_real.py --mock \
  --output validation/vr_real/mock_integration.json

# 单元和 C++ 测试
export PYTHONPATH="$PWD/src/a6_y_vr_teleop:$PYTHONPATH"
python3 -m pytest -q src/a6_y_vr_teleop/test tools/test_s0.py
export ROS_DOMAIN_ID=87
ctest --test-dir build/a6_y_controllers --output-on-failure
```

C++ 的 `ExternalVrRosHandshakeDynamicTrackingAndLocalStop` 使用真正的控制器、ROS 消息与六轴耦合惯量，积分步长 1 ms，控制周期 8 ms；假设理想重力抵消和简单粘性阻力。它验证小目标跟随和本地停止，不能代替未知摩擦/负载、延迟压力及实物停距测试。

当前固定目标回归中 `RepeatedLargePushesWithStrongerResistance`、`FrictionReleaseAtRecordedPose` 在现有调参组合下存在失败场景，涉及延迟和 20/40 ms 等工况。保留断言与参数，并记录失败，不宣称完整回归或 R2 全部通过。

结果保存在 `validation/vr_real/`。其中的状态消息频率是可观测发布频率，不代表 read/update/write 全链路执行时间，也不是硬实时保证。

## 初次接入结果（2026-10-08，历史记录）

- 构建通过。Python 41 项通过；C++ 总计 74 项中 72 项通过，其中新增接管状态机 14 项、外部参考接口/耦合动力学与失能反馈等待测试通过。上述两项固定目标压力测试仍失败。
- mock 完成 prepare、arm、捕获真实接口锚点、跟随、正常松手、输入丢失、人工重新许可、disarm；状态发布约 125.000 Hz。mock 的实际 q 保持模拟值，不能拿它证明真实移动。
- 实机只读记录 2064 个状态，观测约 124.995 Hz；状态间隔 P99 约 9.065 ms、最大约 9.479 ms；最大反馈年龄约 48.045 ms。采集始终为 DISABLED，七电机始终未使能。
- 当时实测 J2 为 −0.002098 rad、夹爪为 +0.00038953 m，按当时的严格 0 端点检查被 Bridge 拒绝。后续增加了上文的零位容差，这两个读数可通过位置范围检查；历史报告和原始值保留。
- 首次只读启动曾因将驱动超时收紧到 0.1 s 而失败，记录 J2 初始年龄约 0.106 s。新入口恢复驱动原有 0.5 s 等待；控制器失能阶段等待完整新鲜反馈，使能后的 0.05 s 检查保留。
- 直接重复 launch 会被现有控制器检查拒绝；未完成调试记录的 execute 会在打开硬件前被拒绝。
- 头显现场测试按用户要求推迟，`actual_quest_tested=false`。没有执行真实机械臂跟随、旋转或夹爪开合。测试入口与本次新增网关均已退出。

报告：[只读实机](../../../validation/vr_real/readonly.json)、[mock 集成](../../../validation/vr_real/mock_integration.json)、[C++ 结果](../../../validation/vr_real/controller_tests.json)、[Python 结果](../../../validation/vr_real/python_tests.xml)、[执行前缺项](../../../validation/vr_real/preflight.json)。原始记录压缩为 `validation/vr_real/readonly_trace.jsonl.gz`、`mock_trace.jsonl.gz`；失败断言保存在 `controller_test_output.log.gz`。

当时完成的是软件能力、基本动力学与被动实机反馈验证；后续保持验收见下文。

## 支撑下保持与退出复测（2026-10-08）

已增加 `commission stage:=hold/follow`、累计关节位移保护、保持验收脚本和分阶段预检。此轮始终使用现场已确认的机械支撑，没有发送真实 VR 跟随或夹爪开合目标。

| 实机记录 | 结果 |
|---|---|
| 首次保持 | PREPARE 后约 15 ms，J4 速度 0.1685 rad/s 超过验收上限 0.1，保护失能。用户确认无人碰动。旧代码第一拍直接加载约 1.80 N·m；改为 2 秒重力/刚度渐增后复测，速度阈值未放宽。单次对照不能排除所有负载/反馈因素。 |
| 渐增后首次复测 | 保持 15.04 秒通过，最大偏差约 0.72°；DISARM 的驱动失能批次耗时约 42 ms，触发控制器 40 ms 周期故障。原工具只统计保持，误把总结果写为通过；保留原始记录并另存复核报告，总体验收未通过。 |
| 最终复测 | 保持 **15.040 秒、1881 个稳定段样本**；最大偏差 **0.612°**、稳定段最大速度 **0.02198 rad/s**、参考固定、无饱和。DISARM 后持续观察 **0.345 秒**，保持 DISABLED；七电机反馈均为失能。整体通过。 |

失能周期修复仅在已经 DISABLED 时等待下一正常周期，期间继续零输出、不处理 PREPARE；使能后的周期和速度保护保留。保持/退出分别判定，防止再次把退出故障遗漏。`supported_enable` 已登记为通过；该证据不代表无支撑承载、独立断电、命令丢失或运动停距已验收。

构建通过，Python **62 项通过**，C++ **82 项选定测试通过**，含重力渐增和失能后长周期两项回归。上述两项已知失败的压力测试排除在该轮之外，未宣布完整回归通过。小范围跟随 mock 演练通过 prepare、arm、捕获、跟随、松手、输入失联、重新许可和 disarm，观察约 125 Hz；没有访问硬件。所有本轮实机/模拟控制进程均已退出。

报告：[最终实机保持与失能](../../../validation/vr_commission/physical_hold_final.json)、[原始记录](../../../validation/vr_commission/physical_hold_final.jsonl)、[首次超速](../../../validation/vr_commission/physical_hold.json)、[第二次结果复核](../../../validation/vr_commission/physical_hold_ramp_review.json)、[mock 跟随](../../../validation/vr_commission/mock_follow.json)、[C++](../../../validation/vr_commission/controller_tests.json)、[Python](../../../validation/vr_commission/python_tests.xml)、[阶段预检](../../../validation/vr_commission/preflight.json)。每次 launch 另有 `*_launch.jsonl` 保存有效参数和文件哈希。

下一步仍需现场数据：实体急停/电源开关的独立断电效果、丢失电机命令时的行为、Quest 及手柄方向、夹爪实际开度/TCP、工作空间和桌面高度/余量。用户已确认有可切断电机电源的实体装置，尚未实际操作切断试验。上述条件完成后，填写停止验收允许值，用 `commission stage:=follow` 采集松手、输入丢失、Bridge 退出三类物理停止记录；最终登记快照后才开放 `execute`。

### 下一项：独立断电测试的现场步骤

该项需要现场人员实际操作已确认的实体装置；仅有软件反馈消失不能证明电机断电。机械支撑保持到整个测试结束。

1. 启动 `mode:=commission stage:=hold`，使用新的 `record_file`；在同一 ROS 域运行 `python3 tools/vr_control.py prepare`，等待 prepare_ramp_complete=true 并确认 HOLD 稳定。此项不运行带自动结束的保持工具。
2. 现场按实体急停或切断**电机电源**，记录操作时间、电机电源指示及机械臂实际反应。保留 USB/CAN 连接以观察主机记录；反馈失效后的 FAULT 是预期保护，不能自动视为断电已成功。
3. 先保持断电并退出该 launch，再由现场恢复电源。恢复后只启动 `mode:=readonly`，确认七电机失能；这一步不执行 prepare。
4. 将操作员观察、切断前后 JSONL 和恢复后的只读记录共同作为 `independent_stop` 证据。未实际执行时保持 pending。

丢失电机命令是另一项试验：需要先核实当前电机固件的通信超时配置和行为。拔 USB、杀死控制进程与切断电机电源不同，不能拿这次 DISARM 或独立断电记录替代。

## 实际 Quest 接入后的力控复测（2026-10-08）

头显最初连接在旧虚拟网关 8012，退出该网关并重新进入 8014 后，收到真实输入。仅执行 PREPARE、没有 ARM 的两次复测分别在约 0.183 秒和 0.503 秒触发 J4 速度保护（0.10989 rad/s），位置读数同步变化。第二次把重力加载时间从 2 秒延长为 5 秒，仍未消除故障，不能记为修复或实机跟随通过。两次都确认七电机失能并退出控制进程。记录：[首次](../../../validation/vr_live_trial/first_live_prepare.json)、[五秒复测](../../../validation/vr_live_trial/prepare_ramp5_report.json)。

同时修复了网关记录原始事件中二进制字段时的 JSON 序列化错误；规范输入数据仍按真实到达时间发送，未使用回放冒充输入。用户随后明确要求先停止力控路线、实现位置控制。
