# Piper 与 A5 实机位控：可简化的部分

核对日期：2026-10-08。参考为本地 `/home/alce/Code/Meta_to_Piper/piper-quest3-teleop` 当前源码。用户在获知 full 将关闭 CAD 自碰检查后明确回复“授权放宽全部”。下述改动已实现、构建并安装，95 项完整 Python 回归和 3 项 IK 异常定向测试通过；扩展模拟连续跟随 6 秒，J5 距首次 PREPARE 达到 3.955°，越过旧 2° 门槛，停止/恢复测试也通过。[模拟报告](../../../validation/vr_position/piper_full_mock_report.json)。本轮未启动实机，模拟不作为真实运动证明。

Piper 值得借鉴的是连续跟随：握住时基于相对位姿计算目标，松开保持，下一次握住重新捕获基准；单帧 IK 失败保留上一关节目标，下帧继续。A5 应减少不必要的锁存暂停，而保留实际关节边界、限速、反馈/命令超时和电机故障处理。

| 环节 | Piper 源码事实 | A5 对照与选择 |
|---|---|---|
| 频率 | IK 主循环与 CAN sender 配置均为 100 Hz，反馈录制 20 Hz。[配置](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/config.py:18) | A5 配置 125 Hz；不能仅由配置声称实际循环达标，也不必为了移植改成 100 Hz。 |
| 相对控制 | HOLD 再握住时重设机器人和手柄基准；松开转 HOLD。[状态机](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:238) | 保留 A5 从实际关节捕获和握把接管。Piper HOLD 仍运行 IK，并非严格冻结关节，不能照抄这一点。[IK 分支](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:277) |
| 输入比例 | 平移 1.0、旋转 0.9；姿态使用相对旋转和轴映射。[配置](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/init_fk_mapper.py:21)、[映射](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/mapping/vr_mapper.py:169) | A5 平移比例 0.2，full 启用旋转；比例独立于下游速度限制，不应把 Piper 坐标轴和模型零位直接套入。[A5 配置](/home/alce/Code/A6-Y-G/src/a6_y_vr_teleop/config/a5_vr_real.yaml:7)、[full 配置](/home/alce/Code/A6-Y-G/src/a6_y_vr_teleop/a6_y_vr_teleop/position_profile.py:31) |
| 平滑与死区 | 映射后做 1€ 位置滤波、姿态 SLERP 与滞回死区；配置位置解冻 2 mm、姿态约 0.6°。[滤波](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/control/pose_filter.py:75)、[参数](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/config.py:32) | A5 已有 TCP 与关节步长限制。可借鉴输入平滑，但死区是抑制抖动，不是忽略碰撞的容差。 |
| IK 限制 | Mink 只配置 ConfigurationLimit 和 VelocityLimit，速度默认 π rad/s；未添加 CollisionAvoidanceLimit。[任务与限制](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/init_mink.py:14) | 保留 A5 模型限位和自身速度上限；Piper 速度不能证明 A5 硬件可使用相同值。 |
| 自碰处理 | 上述 IK 限制列表未加入碰撞避让；应用层也没有候选网格路径检查。 | 按用户要求，A5 **full 原生位控**设置 `geometry_checking=False`，跳过 CAD 检查，启动及诊断明确显示已关闭。**该模式没有自动 CAD 自碰避让，也不保证无碰撞。** trial 与阻抗流程仍默认强制检查。 |
| 无效 IK | 结果非有限或求解抛异常时保留 `last_q`，下一帧继续，没有因此进入锁存暂停。[调用与异常处理](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:277) | 建议将可恢复的候选运动拒绝改为“保持可接受参考、下一帧重试”，而不是所有错误都调用 `pause()`。 |
| 追踪与反馈超时 | 输入共享矩阵/按键未携带事件时间和序号；sender 重复最新共享目标，运行反馈只用于录制，读回失败继续。[输入回调](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:144)、[发送循环](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/piper_send_process.py:49) | 在这条应用代码路径未发现输入新鲜度和共享目标超时检查；这不证明 SDK/固件没有保护。A5 不复制此缺口，继续保留输入、反馈与命令看门狗。 |
| 启动 | 先读实际角度，但初始模式为 RETURNING，随后自动关节回零。[实际读回](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:148)、[初始模式](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/context.py:40) | A5 保留显式 prepare/arm 和实际位置保持，不移植自动回零。 |

## 本轮实现范围

**full 原生位控关闭 CAD 检查。** 本次实际记录显示，原有零位容差随运动离开小范围后，又被 `link1/link4` 等模型检查阻断。三对已确认折叠间隙的网格距离仍可能报告相交，不能通过调大 ±2° 容差证明任意运动无碰撞。按用户要求，完整位控设置 `geometry_checking=False`，控制链跳过 CAD 检查，不增加后台扫描或提示计算。关节/夹爪硬边界、速度和加速度、目标超前/跟踪误差、输入与反馈超时、追踪跳变和电机故障停止继续保留。trial 和阻抗模式仍默认 `geometry_checking=True`。

启动信息和诊断显示“CAD 自碰检查已关闭”，不输出“无碰撞”判断，也不把关闭状态描述为已通过检查。这一选择消除了 CAD 计算对控制时序的依赖，同时明确失去自动 CAD 自碰检查；现场环境也没有自动感知。

**按碰撞对使用相关关节容差。** 原 A5 零位例外要求全部六轴都在首次 PREPARE 的 ±2° 内；共同上游轴刚体转动、无关下游轴转动也会使例外失效。[原检查位置](/home/alce/Code/A6-Y-G/src/a6_y_vr_teleop/a6_y_vr_teleop/geometry_guard.py:145) 同一模块已计算每对连杆的 `relative_axes` 并用于距离界。[相关轴计算](/home/alce/Code/A6-Y-G/src/a6_y_vr_teleop/a6_y_vr_teleop/geometry_guard.py:60)

按对限定相关轴是后续强制几何检查可做的精确改进，但**不足以解决本次完整位控诉求**：J5 等相关轴超过 ±2° 仍会退出例外，模型误差并未消失。本轮不靠扩大例外范围来声称模型准确；完整位控直接关闭 CAD 检查，强制模式继续保持原验证范围。

**IK 候选失败时保持并重试。** full 设置 `hold_on_ik_failure=True`。仅当实际反馈有效、原参考仍可接受时，格式无效或数值求解失败的候选保持控制器当前参考并报告原因，下一帧重新计算；不把延迟的实际反馈当作回退目标，避免突然反向。保持时冻结 TCP 目标积分，避免不可达目标累积。目标超前/跟踪误差、输入失效、反馈过期、真实越界和电机故障继续进入原停止流程。在强制几何模式中，候选仍必须通过原路径检查；完整位控跳过 CAD 判定。

**保留追踪跳变停止。** `tracking_jump` 仍检测突然的手柄位姿变化。[输入处理](/home/alce/Code/A6-Y-G/src/a6_y_vr_teleop/a6_y_vr_teleop/real_bridge.py:98) 本轮不删除此检查：不能为了取消暂停而把追踪重定位直接当作真实手移动送入机械臂。松开再握住后按原流程重新捕获基准；正常有效手运动继续由 TCP 与关节速度限制平滑跟随。

## 不可达目标与候选回退

Piper 每步调用 `mink.solve_ik()` 求速度，然后用 `dt` 积分；`dt` 截在 0.1–33.3 ms。应用层未检查末端误差是否已收敛：有限解直接接受，异常/非有限结果保留旧关节目标。[单步实现](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/control/ik_stepper.py:23) 这只能证明它允许连续尝试，不证明任意不可达目标一定得到平滑解；本次未取得对应安装版本的 Mink 内部源码，因此不对求解器在所有不可达情况的行为作进一步承诺。

A5 已用阻尼最小二乘、全局关节速度缩放、位置限幅和四档步长回退（1、1/2、1/4、1/8），仅接受不增加加权末端误差的候选；没有改善就返回原关节位置。[A5 IK](/home/alce/Code/A6-Y-G/src/a6_y_vr_teleop/a6_y_vr_teleop/a5_kinematics.py:79) 因而本次重点不是增加另一套 IK，而是把“合法但无进展/可恢复求解失败”与“输入、硬件或实际状态故障”区分，避免前者反复锁死整条控制链。新增回退需覆盖持续握把、反向离开关节边界、追踪重定位及真实故障仍停止的回归测试；full 关闭 CAD 与 trial/阻抗强制检查也需分别覆盖。
