# 四臂 MoveIt / ros2_control 移植版

当前默认入口使用 `A6_Y-main(1).zip` 中的 MoveIt 配置、标准轨迹控制器和 C++ 达妙驱动。支持四臂选择、启动自动回零、可选只读启动、XYZ/RPY 拖动、绿色目标预览、夹爪操作、独立速度/加速度设置、回零与退出保护。

## 启动回零、曲线零参考与夹爪电子限位

选择运动端／示教端的左臂或右臂，点击 **启动并回零**：先读取反馈，再用 MoveIt 规划 J1～J6 = 0，预览 1 秒后自动执行。六轴反馈进入 ±0.005 rad 后保持零位，等待下一步指令；已在零位时直接进入位置保持。夹爪保持启动时的开度。真机模式下，此按钮会授予本次执行许可并使能七个电机。启动仍默认模拟模式。

启动回零失败会保留会话并显示原因，不自动重试。启动或回零期间可点击停止；停止、撤销许可、关闭请求均可取消尚未执行的启动回零。若需要先检查实物，可在启动前取消 **启动后自动规划回零并保持**，或在启动命令末尾添加 `--read-only-start`。

回零成功后，以当时的**实测末端位姿**建立本次会话的曲线零参考，XYZ、Roll/Pitch/Yaw 此时归零。XYZ 显示基坐标轴方向上的位移 `p-p0`，RPY 显示相对旋转 `RPY(R0ᵀR)`；实际反馈噪声仍会显示。之后再次回零不会重设参考，便于观察回零误差。回零完成前六条相对曲线留空，关节、夹爪曲线及日志继续记录。左侧“绝对 TCP”和 CSV 的 `tcp_absolute_*` 保留原坐标；`plot_zero` 事件记录参考矩阵。这不是重新标定，也不改动 URDF、关节角反馈或运动目标。

夹爪通过 `gripper_limits.yaml` 设置电子端点，默认从原模型闭合端、张开端**各内收 0.7 mm**：

| 界面开度 | 新目标单指位置 | 原模型端点 |
|---|---:|---:|
| 0% 闭合 | −0.7 mm | 0 mm |
| 100% 张开 | −34.3 mm | −35 mm |

目标及反馈百分比使用相同电子行程。反馈不裁剪，初始处于电子范围外时可能显示负值或大于 100%；毫米反馈及 `gripper_model_percent` 仍记录原模型数值。夹爪的规划终点必须在电子范围内；范围外起点只允许向内恢复，路径检查失败时取消动作，不改写 MoveIt 轨迹。

0.7 mm 是可调整的初始余量，尚未实测张开端极限，不保证四臂实物均能到达这些端点，也不做自动顶限位寻找端点。调整 `defaults.closed_margin_mm` 或 `defaults.open_margin_mm` 后关闭并重新启动会话；可用 `role_overrides` 为四臂分别配置同名字段。余量必须为正且保留有效行程。这是本程序的目标／轨迹电子限位，不是驱动硬件限位；原 URDF、驱动位置范围、标定和 PID 保持原值。

验证：37 项 Python／Qt 检查通过，RViz 面板构建及执行条件／退出测试通过。四臂均使用真实 MoveIt、原版 100 Hz JTC 与 C++ dry_run 完成启动回零、位置保持、六条曲线归零、失能和退出；运动左臂另外完成电子闭合端／张开端规划执行。没有驱动真实机械臂。报告：`diagnostics/arm_moveit_startup_home_validation_20261125.json`；验证入口：`verify_startup_home.py`。

## 选择窗口的实时曲线与日志

启动命令不变。选择机械臂并启动后，**第一页选择窗口右侧**自动显示反馈，不需要进入 RViz：

- **末端与关节**：相对本次回零位姿的 XYZ（mm）、Roll/Pitch/Yaw（°），J1～J6 的模型关节角度（°）和反馈速度（°/s）。只读启动且尚未回零时显示绝对 TCP，标题标明参考方式。末端由当前反馈经正运动学计算，不是拖动目标。
- **七电机与夹爪**：M1～M7 的电机角度（°）、速度（°/s），另显示夹爪单指行程（mm）和速度（mm/s）。M 编号对应电机 ID。电机角度是根据当前标定的方向、比例和零偏，从原驱动滤波后的关节位置反算；不是未经处理的 CAN 编码值。速度使用原驱动发布的 `JointState.velocity`，不在界面另做差分或滤波。
- **夹爪开合**：单独的大幅滚动曲线，同图对比青色的**反馈开度**和橙色的**目标开度**，单位 %。0% 为电子闭合端、100% 为电子张开端，纵轴默认覆盖 0～100%，不裁剪超范围反馈。支持与其他曲线相同的 30／60／120 秒窗口及暂停；页顶显示数值和根据反馈速度判断的张开中／闭合中／静止。设置目标仅改变橙色线，反馈线随实际测量变化。
- **运行日志**：实时查看应用结果或各 ROS 进程的原始终端输出，可按来源选择、按文字筛选，例如“碰撞”“ERROR”“planning”；支持打开历史会话日志目录。

曲线按 **20 Hz 目标采样频率**记录，界面约 10 Hz 刷新，显示最近 **30 / 60 / 120 秒**，每条曲线标注当前值。暂停曲线、切换标签、隐藏窗口均不停止后台记录。纵轴自动缩放，不把反馈裁剪到预设范围；这不是完整可达工作空间边界的可视化。RPY 为 `Rz(yaw) Ry(pitch) Rx(roll)` 的规范欧拉角，跨 ±180° 时断线，姿态奇异处仍可能跳变。

反馈超过 0.5 秒、某电机反馈缺失或故障时，相应曲线留空并显示有效关节数；没有速度反馈时显示空值而非 0。真机同时检查 `dynamic_joint_states` 的新鲜度与电机状态。CSV 中原始驱动 SI 数值仍保留，配合 `valid` 列判断，不把旧值伪装为实时值。

每次启动单独保存到：

```text
/home/taoqiu/文档/ChatGPT/A6Y远程遥操作/diagnostics/arm_moveit_native_reports/<机械臂角色>_<会话编号>/
```

| 文件 | 内容 |
| --- | --- |
| `application.log` | 可读的封装层日志，时间为北京时间：操作、目标、许可和参数变化、规划结果、碰撞、执行、回零、退出、异常 |
| `gripper.log` | 夹爪目标、规划和执行结果的独立索引；日志页可选择“夹爪开合记录”。同步目标明确标注来源，设置目标不会被记成执行成功 |
| `events.jsonl` | 结构化事件、MoveIt 返回码、异常堆栈、碰撞连杆、轨迹文件名；UTC ISO 时间和 Unix 秒便于关联 ROS 时间戳 |
| `telemetry.csv` | 连续反馈：时间、反馈年龄、末端 XYZ/RPY、六关节角度/速度、七电机角度/速度、夹爪目标与反馈开度（%）、目标行程、反馈行程/速度、开合方向、执行分组、原始 SI 反馈及各关节有效性和故障状态 |
| `telemetry_metadata.json` | 坐标系、单位、采样频率、模型 SHA256、关节映射和本次碰撞配置 |
| `move_group.log`、`controller_manager.log`、`rviz2.log`、`spawner_*.log` 等 | 相应 ROS 子进程原始 stdout/stderr；保留输出中的 ROS 时间戳，不改写原始内容 |
| `ros_logs/` | ROS 自身生成的节点日志 |
| `plan_<时间戳>_<分组>.json` | 每次成功规划的原始 MoveIt 轨迹；`last_native_plan.json` 同时保留最后一条 |
| `session.json`、`live_status.json`、配置文件 | 最终报告、当前状态快照以及本次实际使用的配置 |

点击左侧 **查看本次日志** 打开日志页，**打开日志目录 / CSV** 打开文件管理器；**查看历史日志…** 选择以前的会话目录。日志页为有界实时预览，最多显示最近 2000 行，完整文件保留在磁盘。多个 ROS 文件按读取顺序汇总，跨进程定位时以原始时间戳为准。

采样、日志写入使用独立线程，约每秒刷新磁盘；界面只取有界历史，不进入 ros2_control 的控制循环。写入失败或事件队列溢出会在监测区明确提示，停止/退出不会等待日志无限阻塞。长时间记录会持续占用磁盘，未设置自动删除历史日志。此次没有改动 URDF、控制器算法、标定或电机 PID。

验证：28 项 Python / Qt 检查通过，包括标定换算、速度缺失、过期反馈、磁盘故障、碰撞事件、日志筛选及原有退出策略；原版 C++ dry_run 的机械臂与夹爪执行、停止、回零退出通过。模拟会话保存 828 条反馈和 6 条独立规划，执行期间采样间隔中位数约 50.10 ms；退出清理出现过约 1.05 秒间隔，时间戳如实记录，曲线不跨间隔连线。未执行真实硬件运动。验证索引：`diagnostics/arm_moveit_telemetry_validation_20261125.json`。

## 裸机碰撞配置（当前默认）

用户已确认：实机使用 A5 结构和新 A5 末端，所有装饰外壳未安装，电机和承力件保留；要求保留现有 URDF，仅放宽碰撞保护。本次不修改 `src/A5/urdf/A5.urdf`、STL、关节安装坐标、轴方向、零偏或末端定义。

启动窗口新增 **碰撞检查** 选择，运行会话期间锁定，关闭后可切换：

- **裸机碰撞配置**（默认）：在生成的 MoveIt SRDF 中额外忽略 `link1 ↔ link4`、`link1 ↔ link5` 两对自碰撞；**运动端右臂**另忽略 `link2 ↔ link4`。运动端左臂、示教端左／右臂不应用这一新增例外。其余原有自碰撞规则、环境障碍物、关节范围和轨迹复核继续生效。
- **完整碰撞配置**：恢复原移植版的碰撞矩阵，包括上述三对检查；适用于需要检查带外壳网格的情况。

规则保存在 `collision_policy.yaml`，原始 URDF 和压缩包中的 SRDF 不改写。每次会话的 RViz 状态、`live_status.json` 和 `session.json` 都记录选用配置。两档使用相同 A5 运动学和末端。

**此放宽作用于整对连杆，不会区分装饰外壳与内部电机／承力件。** 因此不能宣称被忽略连杆对的实体碰撞仍受检查，也不代表已经生成准确裸机碰撞网格。绿色目标表示“按当前碰撞配置检查通过”。启动回零使用同一碰撞配置；拖动和单独规划只更新预览。

运动端右臂现场 `motion_right_9tzfeso0` 的两次失败均来自起点的 `link2 ↔ link4` 网格接触，路径前 6/41、4/25 个点复核失败并返回 −2；目标逆解有效不代表起点或整条路径有效。新增规则写在 `bare.role_disabled_pairs.motion_right`，选择窗口、RViz 和会话记录显示该臂实际生效的连杆对。源 A5 URDF、网格、标定及其他机械臂的规则保持不变。

右臂验证：使用原版 C++ dry_run 重放现场起点和两个目标，旧裸机规则复现同一 −2；新规则的两个目标分别得到 32 点、20 点有效轨迹，加入环境障碍物后仍拒绝规划，均能只读退出。未连接真实串口、未提交轨迹执行。报告：`diagnostics/arm_moveit_right_collision_20261125.json`；验证器：`verify_right_collision.py`。

验证：20 项 Python 检查通过；用 `motion_left_bcaiwqdg` 保存的真实起点和目标，在原版 C++ dry_run 中重放，完整配置复现 −2，裸机配置成功完成规划、机械臂与夹爪模拟执行、回零及失能。加入环境障碍物后仍拒绝规划。另验证 RViz 显示裸机规则、只读退出以及选择窗口恢复完整配置。未连接实机执行。结果见 `diagnostics/arm_moveit_bare_collision_20261125.json` 和 `diagnostics/arm_moveit_bare_validation_20261125.json`。

保留全检查启动命令：

```bash
bash "/home/taoqiu/文档/ChatGPT/A6Y远程遥操作/A6-Y-G/scripts/run_arm_moveit.bash" --collision-profile full
```

## 直接复制启动

在 Ubuntu 图形桌面终端复制整行，不用切换目录：

```bash
bash "/home/taoqiu/文档/ChatGPT/A6Y远程遥操作/A6-Y-G/scripts/run_arm_moveit.bash"
```

首次启动自动编译移植包和 RViz 面板。后续源码未变化时复用构建结果。ROS Humble 的接口生成器不支持当前中文源路径，因此使用 `/tmp/a6y-moveit-build-<UID>` 作为可重建的英文路径构建缓存；项目源码始终在本目录。缓存清理后下次启动会重新编译。入口不加载旧 `install/` 中的同名驱动。

1. 在图形窗口选择 **运动端左臂／右臂、示教端左臂／右臂**，选择模拟或真机，再启动。
2. 默认 **启动并回零** 会读取七电机反馈，自动规划回零、使能并执行，到位后保持等待。需要先核对模型与实物时，取消自动回零复选框，以只读方式启动；此时执行前需手动勾选许可。
3. RViz 选择 **Interact**。中心球和直线箭头控制 XYZ，三个圆环控制 Roll/Pitch/Yaw。默认完整位姿；勾选“仅位置”后隐藏圆环，允许规划器选择姿态。
4. 绿色目标模型显示 MoveIt 的有效逆解；目标不可达或碰撞时保留最后有效模型并显示原因。拖动只预览，不控制实体机械臂。
5. 分别设置速度与加速度比例，点击 **规划 Plan**，检查原生 MoveIt 轨迹动画，再点 **执行 Execute**。
6. 夹爪用开度滑条或张开／闭合按钮设置目标，点击 **夹爪规划**，再点击 **执行夹爪**。

### 显示规划通过，但执行按钮是灰色

先看 RViz 左侧 A6Y 面板顶部的执行条件提示。启动回零会授予本次执行许可；只读启动或撤销许可后，需要手动勾选 **已核对零位与方向，允许真机执行**。选择窗口也有同一许可开关，两处状态同步。单独勾选只授予执行许可，不会自动运动。然后点击 **规划 Plan**，检查预览，再点击 **执行 Execute**。夹爪规划使用 **执行夹爪** 按钮。

有效规划保留 30 秒；修改目标或速度／加速度参数也会清除规划，此时需要重新规划。面板现在明确显示“等待授权”“重新规划”或“当前操作尚未结束”，选择窗口也显示相同原因。**只读反馈、轨迹控制器未激活是只读启动后的正常状态，并不阻止已授权的有效规划进入执行。** 默认自动回零会激活控制器并保持零位。

规划成功消息中的秒数是轨迹预计执行用时，不是规划计算耗时。此次运动端左臂日志记录了 35 个轨迹点、预计 3.38 秒，未提交执行请求。新增提示通过 12 项 Python 检查及 RViz Qt 面板的执行条件和关闭场景检查；没有连接实机执行。

默认速度和加速度均为 **10%**，可分别设置 **1%～100%**。启动回零也使用此默认比例。修改目标或比例会清除旧规划。夹爪 0%／100% 对应上文电子端点；另一手指由 URDF mimic 联动。这个数值是单指行程，不是标定后的物理指间距离，也不是夹持力设置。

**启动自动回零或只读模式下首次执行，会按原程序同时接管并使能七个电机。** 机械臂和夹爪分别由自己的轨迹控制器执行，未改变目标的关节保持当前位置。此行为与旧 Python 版“仅使能本次分组”不同。模拟模式也运行原版 C++ 驱动的 `dry_run` 和两个实际 JTC，不打开串口。

## 回零／退出卡住的修复

最新修复针对 `motion_left_vhx9ie99` 的现场：已勾选许可、规划失败，但两个轨迹控制器仍为 inactive、实际执行次数为 0。旧退出条件把许可当作接管，反复要求回零并因碰撞失败取消退出。现在退出只看实际接管、执行及释放确认状态：未接管或已确认释放的只读会话直接关闭，与许可勾选和规划成功与否无关；不会为了退出而首次使能或回零。现场会话已通过撤销未使用许可再关闭完成恢复，记录在该目录的 `readonly_exit_recovery.json`，没有请求运动。

已修复本机这次出现的关闭流程问题：主控进程消失后，旧 RViz 面板仍拦截关闭并等待不存在的主控批准。相关现场日志保留在 `diagnostics/arm_moveit_native_reports/motion_left_kigx5tg9/`；清理记录为 `orphan_recovery.json`，当时残留控制器七关节全部为只读配置，未执行运动或使能。

- 重复回零／关闭请求合并为一次。停止、紧急失能和撤销许可会清除排队动作；被取消的回零请求不能再次提交规划或执行。
- 已接管会话回零失败时，立即结束本次操作并取消退出。只读会话不进入此流程。MoveIt 返回 −2 时显示“轨迹未通过规划后复核”，并显示日志中的碰撞连杆；不会跳过碰撞检查。现场日志包含 `link1 ↔ link4`、`link1 ↔ link5`，这表示模型路径检查失败，不等于已确认实物发生碰撞。
- 等待规划／执行时显示已经等待的秒数和超时上限。会话目录每秒更新 `live_status.json`，便于区分规划等待、执行等待、失能与清理。
- RViz 获得关闭许可后，丢失后续状态不会再次锁住关闭；检测到主控进程已经消失时，可以正常关闭显示。状态超过 2 秒未更新时显示“仅关闭显示”按钮，明确不代表已经回零或失能。
- 各原生 ROS 子进程增加独立生命周期守护。主控异常退出时，守护进程先请求原生进程正常退出，超时再终止残留进程；不会自行规划回零。正常清理按整组约 6 秒的等待预算执行，ROS 清理函数报错也不能跳过剩余清理步骤。

上一轮修复通过 15 项 Python 检查、5 种 Qt 关闭场景，以及原生 RViz/MoveIt、碰撞取消退出、虚拟串口回零失能和主控异常退出清理测试。索引：`diagnostics/arm_moveit_close_fix_20261125/validation.json`。这些测试没有正确覆盖“许可已勾选但从未接管”的退出策略，本次补充独立回归；已接管后的碰撞退出测试先执行一次 C++ dry_run 轨迹以建立真实控制器接管状态。

本次通过 18 项 Python 检查、5 种 Qt 关闭场景、已接管后的碰撞与停止保护，以及原版 C++ 驱动的虚拟串口只读退出复现。带 RViz 的测试约 7.05 秒退出，1575 帧全部为反馈查询，没有使能、位置控制、回零或失能帧；所有测试子进程退出。索引：`diagnostics/arm_moveit_readonly_exit_fix_20261125/validation.json`。只读退出复现命令参数为 `verify_native_serial.py --readonly-close-test --rviz`；该验证器只打开它创建的 `/dev/pts/` 虚拟串口。

回零路径必须符合选用的碰撞配置。退出修复本身未修改碰撞规则；其后的裸机配置仅额外放宽前文列出的连杆对，并按所选臂应用右臂例外。原程序的 MoveIt 规划参数、ros2_control 控制算法和电机 PID 保持不变。

## 实际执行链路

```text
四臂选择窗口 / RViz 操作柄与面板
  → 原版 CachedKDL：受关节范围、碰撞检查约束的目标逆解
  → MoveGroup action：原版 OMPL 配置、路径规划和时间参数化
  → 原生 RViz Trajectory 插件：显示原始 RobotTrajectory
  → ExecuteTrajectory action：原始轨迹不改写
  → 原版 MoveItSimpleControllerManager
  → FollowJointTrajectory
  → arm_controller / gripper_controller（标准 JointTrajectoryController）
  → 原版 A6YDamiaoSystem（100 Hz、C++ SLCAN、POS_VEL）
```

Pinocchio 只用于当前模型的正运动学、TCP 数值和绿色模型绘制，不再负责 MoveIt 的逆解或执行控制。Python 层只处理选择、UI、原生 ROS action/service 和会话生命周期，不插值轨迹、不重定时、不直接收发电机控制帧、不追加旧版速度包络或 0.15 rad 跟踪门限。

参考程序的 `a6_y_interfaces`、`a6_y_moveit_control` 也已原样编译保存；默认 UI 直接调用 MoveGroup / ExecuteTrajectory 原生动作接口，未运行里面硬编码旧末端名字的 `end_effector_server`。RViz 的规划按钮仍为保留需求的 A6Y 面板，不冒充 MoveIt 官方 MotionPlanning 面板。

## 沿用的参数与必要适配

- 原样使用归档中的 `kinematics.yaml`、`joint_limits.yaml`、`moveit_controllers.yaml`、`ros2_controllers.yaml`，包括 100 Hz、position 命令接口、机械臂与夹爪两个 JTC。
- 运行参数沿用原 `a6_y_real_moveit_bringup.launch.py`：POS_VEL、`command_timeout=0`、`feedback_timeout=0`、`max_start_delta=0.1`、不在 configure 阶段写模式。内部滤波、模式确认、命令发送及失能流程沿用原驱动。
- 保留已经标定的当前 A5 URDF、四臂 USB 身份、关节方向、零偏、夹爪比例与实测 PMAX。没有套用归档中方向和夹爪行程不同的旧模型。
- 原 SRDF 文件保留；运行时适配当前 link 名称、当前夹爪 open/close，去掉不适用于当前关节方向的 ready 姿态。完整配置沿用原碰撞矩阵；裸机配置额外忽略 `link1 ↔ link4/link5`，仅运动端右臂再忽略 `link2 ↔ link4`。home 仍是 J1～J6 全零。
- 原 C++ 源码唯一修改是允许从每关节参数读取 `protocol_pmax` / `protocol_vmax`，用于当前已核对的电机编码量程；没有修改读写控制算法或 PID。所有归档文件 SHA256 位于 `ros2_stack/upstream_manifest.json`，源码差异可直接查看 `ros2_stack/upstream_changes.patch`。另外补齐原 CMake 要求、但 ZIP 未包含的空 `textures/` 目录。
- 只读阶段设置原驱动已有的 `auto_enable=false`、每关节 `read_only=true`，两个 JTC 为 inactive；首次执行切换为原版可执行配置。由于这些是原驱动启动参数，只重启 controller_manager，MoveIt/RViz 保留。每次 manager 使用不同服务名，隔离旧服务客户端；提交轨迹前等待控制器动作服务就绪。

| 组/关节 | 原配置最大速度 | 原配置最大加速度 |
| --- | --- | --- |
| J1～J3 | 1 rad/s | 2 rad/s² |
| J4～J5 | 1.2 rad/s | 2.5 rad/s² |
| J6 | 1.5 rad/s | 3 rad/s² |
| 夹爪单指 | 0.05 m/s | 0.2 m/s² |

界面比例作用于这些 MoveIt 规划限值，不写电机固件 PID 或加减速寄存器。底层 POS_VEL 速度上限继续使用电机配置；高比例轨迹是否能在实机准确跟踪，仍需后续现场测量。本次没有做用户明确推迟的 PID 和延迟调节。

## 回零、停止与退出

- **规划并执行回零**：读取当前反馈，用原 MoveIt 规划 J1～J6=0，预览 1 秒后执行。夹爪保持当前开度；完成后保持零位。
- **正常关闭、窗口退出、SIGINT/SIGTERM**：已经接管的会话先按同一流程回零，核对六轴反馈在 ±0.005 rad 内，再调用原驱动退出失能流程，并以只读回读确认失能后结束。
- **停止**：取消原生 action 和待处理执行，保持控制器；回零途中停止会取消退出，窗口继续保留。
- **规划或执行失败**：本次规划作废，错误保留，界面与 ROS 会话继续运行，可以重新规划或请求退出。未接管时直接退出；已接管后的退出回零失败则取消退出，不直接发送六个零位置，也不自动重试。
- **已支撑 · 紧急失能**：取消当前轨迹，调用原驱动失能，确认后恢复只读、撤销许可；不宣称完成回零。
- **只读退出**：未接管或已确认释放、且没有轨迹执行中的会话直接关闭，无论是否勾选许可、规划是否失败；不要求新鲜反馈、不使能、不回零，报告 `home_stage=not_required`，不伪报回零完成。已经接管但撤销许可时，正常退出要求恢复回零许可，或支撑后明确紧急失能。释放未确认时不走只读退出分支。

正常退出保护只覆盖程序仍正常运行、反馈有效且路径可规划的情况。断电、USB 断开、强杀进程、驱动或 ROS 进程崩溃无法保证回零。驱动和控制器的故障处理沿用原程序；它们不是独立的安全控制器。

## 本机 DDS 兼容性（运行适配）

复测在 Fast DDS 下发现 controller_manager 服务响应发布时偶发 `failed to send response: cannot publish data` 并退出。当前启动脚本为整个会话统一选择本机已安装的 `rmw_cyclonedds_cpp`，避免沿用出现该故障的运行配置。它只改变 ROS 通信实现，不改变 MoveIt 轨迹或 ros2_control 控制算法，也没有修改系统安装包。`session.json` 记录本次 RMW；外部调试节点应使用相同实现和 ROS_DOMAIN_ID（默认 196）。

错误文本对应 [ROS 2 Fast DDS 服务响应实现](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/rmw_response.cpp) 的发送失败分支；这说明异常位置，不等于已经确定底层失败的唯一原因。CycloneDDS 的对照结果来自本机测试报告。

## 四臂配置与日志

| 选择 | USB 序列号 |
| --- | --- |
| 运动端左臂 | `304C33733036` |
| 运动端右臂 | `209B3971534B` |
| 示教端左臂 | `304F33733036` |
| 示教端右臂 | `209D3982534B` |

沿用 `diagnostics/arm_roles.yaml`、`tools/arm_visualization/joint_mapping.yaml` 和当前电机参数。USB 身份核验和进程占用检查保留；不会靠 ttyACM 编号猜机械臂。左右臂不自动镜像。

每次会话在 `diagnostics/arm_moveit_native_reports/<角色>_<编号>/` 保存：

- `session.json`、`events.jsonl`：状态、错误、映射及回零结果。
- `move_group.log`、`controller_manager.log`、`rviz2.log`、各 spawner 日志。
- 生成的 URDF/SRDF/参数与 `last_native_plan.json`（MoveIt 返回的原始轨迹）。

无硬件配置检查：

```bash
bash "/home/taoqiu/文档/ChatGPT/A6Y远程遥操作/A6-Y-G/scripts/run_arm_moveit.bash" --check-config
```

## 验证边界

新链路使用 `test_native.py`、`test_window.py`，以及 `verify_native.py`（真实 MoveIt/JTC + C++ dry_run）、`verify_native_serial.py`（真实 C++ SLCAN + 虚拟 PTY）、`verify_native_interaction.py`（交互与失败流程）。旧 Python 后端测试不计入本次移植验收；旧实现保留作历史文件，启动入口不再使用。

已通过 8 项配置／窗口测试；集成报告包括：

- `diagnostics/arm_moveit_native_reports/motion_left_xxwopgq_/verification.json`：RViz、机械臂、夹爪闭合／全开／中间开度、停止保持、回零退出。该目录的 `rviz.png` 是实际运行截图。
- `diagnostics/arm_moveit_native_reports/motion_left_ojmu0rht/interaction_verification.json`：7 个交互控件、固定 TCP 的三轴旋转目标、位置模式、不可达目标、碰撞阻止回零退出、停止取消退出、原生执行失败保留界面。
- 四臂虚拟 PTY 均通过原版 C++ 驱动的只读、接管、机械臂与夹爪执行、回零和失能确认：`motion_left_ur6qh_uz/serial_verification.json`、`motion_right_6aq6yo3h/serial_verification.json`、`teaching_left_5tppmgcr/serial_verification.json`、`teaching_right_p63k6360/serial_verification.json`。

最终测试索引为 `diagnostics/arm_moveit_native_reports/port_validation.json`。以上最终集成测试统一使用 CycloneDDS；没有在 controller_manager 日志中复现此前的服务响应发布异常。

本次没有操作实体机械臂。仿真和虚拟串口验证不证明实机跟踪精度、PID 或机械安装正确。当前 ROS Humble / MoveIt 2.5.9 的 `move_group` 在收到退出信号后的对象析构阶段仍观测到段错误；这是完成运动和失能之后的清理阶段，已保留日志，本次未替换系统 MoveIt 二进制，也不能据此宣称系统库问题已修复。
