# 查看动力示教与夹爪日志

本指南用于复核启动失败、跟随延后或接触反馈触发情况。分析已有文件不需要连接机械臂；命令从仓库根目录执行。

## 1. 找到对应会话

在动力示教窗口点击“本次日志”或“历史日志”，按臂筛选后使用“打开目录”。会话位于：

```text
diagnostics/arm_teaching_reports/teaching_<会话ID>/
```

上位机启动输出另在 `diagnostics/arm_teaching_logs/`。优先用 `session.json` 核对本轮配对、USB、运行模式、实际参数和源码记录，避免把不同轮测试混在一起。`session.log`、`events.jsonl` 与 `result.json` 用于确定启动、跟随、故障和退出结果。

## 2. 先区分启动失败与跟随问题

若会话在身份检查、反馈或模式核验阶段失败，先处理该项；没有进入跟随就没有这一轮的跟随延迟可分析。USB 枚举失败和电机反馈失败分别检查：前者看系统是否存在对应 by-id 路径，后者看该总线上 7 台电机的状态。

进入跟随后，先看 `step_quality.json` 的记录完整性、队列丢弃及缺失周期，再比较目标和反馈。数据缺口会影响精确的时间估计。

## 3. 判断夹爪反馈是否触发

打开 `gripper_haptics.csv`，按 `leader` / `follower` 选择一对机械臂，查看以下字段：

| 字段 | 用途 |
| --- | --- |
| `state` | `free`、`contact` 或 `inactive`；`contact` 表示接触保持已经触发 |
| `contact_reason` | `stall` 为受阻判据，`load` 为持续负载判据 |
| `raw_m`、`target_m`、`actual_m` | 映射目标、最终目标与运动端实际单指模型位置，单位 m |
| `follower_motor_Nm` | 运动夹爪协议反馈换算的电机力矩 |
| `teacher_motor_Nm` | 示教端请求的总电机力矩，包含接触反馈与阻尼 |
| `reflected_motor_Nm`、`damping_motor_Nm` | 接触反馈与轻阻尼的各自分量 |
| `leader_velocity_m_s`、`follower_velocity_m_s` | 双端单指模型速度，用于复核闭合、停滞和张开 |

先确认是否进入 `contact`，再比较请求力矩与臂级日志中的实际发送值；有接触状态不代表每一时刻都输出非零反馈。当前逻辑检测到示教端反向速度时会立即撤掉接触分量，微小回弹也可能造成短暂撤力。提高总上限不会自动修复这一现象。

夹爪力矩是电机估计量，不能直接当作标定后的指尖力。会话日志中的 `effort_j7` 使用移动关节广义力单位；不要直接把它标为 N·m，换算关系见[原理说明](../explanation/power-teaching-and-gripper-feedback.md#力矩反馈与单位)。

## 4. 复核跟随时间

用 `references.csv`、`step_targets.csv` 和 `step_arms.csv` 对照原始映射目标、限速后目标、实际发送目标及反馈。不要只比较手动输入和电机曲线而忽略目标限速。

六轴阶跃报告可在停止采集后重新生成。把下面路径改成实际会话目录：

```bash
A6Y_SESSION=diagnostics/arm_teaching_reports/teaching_实际会话ID
/usr/bin/python3 A6-Y-G/tools/arm_gravity/analyze_teaching_step.py "$A6Y_SESSION"
```

结果写回该会话的 `step_summary.json`、`step_summary.csv`、`step_summary.txt` 和 `step_tracking.csv`；运行前保留需要比较的旧报告。J7 记录为米制夹爪数据，不套用六轴 rad 阶跃阈值。

`send_complete_time` 是主机串口接受完整帧的时刻，不是 CAN ACK；`feedback_receive_time` 是主机解析时刻，不是电机采样时刻。曲线平移拟合的等效滞后、上升时间、稳定时间各自回答不同问题，不能合并成一个固定“USB 延迟”。

## 5. 导出供复核的诊断包

等待窗口显示报告已生成，再点“导出诊断包”。同时记录这次是空载、软物体还是硬物体、实际使用哪一侧，以及出现问题的大致时间。原始采样与现场日志保存在本地；仓库中的运行配置不是原始实验记录。
