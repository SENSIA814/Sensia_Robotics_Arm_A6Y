# 四臂 MoveIt 规划与执行

本入口选择一条机械臂，规划并执行末端位姿、六轴回零和夹爪开合。它使用独立移植的 MoveIt、标准 JointTrajectoryController 和 C++ 达妙驱动；动力示教、重力补偿与 VR 使用其他入口。

[工程入口](../../README.md) · [环境与配置](../../docs/reference/runtime-and-configuration.md) · [底层开发与测试](../../docs/how-to/develop-and-test.md)

## 启动与默认模式

以下命令从仓库根目录执行：

```bash
# 配置检查，不连接实体串口；首次会构建独立依赖与面板
bash A6-Y-G/scripts/run_arm_moveit.bash --check-config

# 打开选择窗口，默认模拟模式
bash A6-Y-G/scripts/run_arm_moveit.bash

# 取消默认启动回零，先只读检查
bash A6-Y-G/scripts/run_arm_moveit.bash --read-only-start

# 使用完整碰撞配置
bash A6-Y-G/scripts/run_arm_moveit.bash --collision-profile full
```

可选择 `motion_left`、`motion_right`、`teaching_left`、`teaching_right`。模拟模式运行真实 ROS 节点、C++ 驱动 `dry_run` 和两个轨迹控制器，不打开串口。切换真机模式后，**启动并回零**会授予本次执行许可、接管并使能七个电机；打开窗口本身不会执行此动作。

默认勾选“启动后自动规划回零并保持”：读取反馈，用 MoveIt 规划 J1～J6=0，预览 1 秒后执行；六轴进入 ±0.005 rad 后保持，夹爪保持启动开度。失败保留原因与会话，不自动重试。先核对实物时取消此选项或使用 `--read-only-start`，执行前再授予许可。

首次启动构建到 `/tmp/a6y-moveit-build-<uid>/`，RViz 面板由独立脚本构建。使用 `rmw_cyclonedds_cpp`、本机通信及默认 ROS domain 196，可用 `A6Y_MOVEIT_DOMAIN_ID` 覆盖。主工程与此目录均有 `a6_y_ros2_control` 包，不能混入同一次 colcon 构建。

## 规划机械臂与夹爪

1. 选择臂、模式与碰撞配置，再启动。身份来自仓库根目录 `diagnostics/arm_roles.yaml`；同一实体串口只能由一个会话占用，不靠 `ttyACM` 编号猜臂。
2. RViz 选择 **Interact**。球／箭头控制 XYZ，圆环控制 Roll/Pitch/Yaw；“仅位置”隐藏旋转控件，并允许规划器选择朝向。
3. 检查绿色目标模型和有效性提示。拖动只更新预览，不控制实体机械臂。
4. 设置速度、加速度比例，点击 **规划 Plan**，检查轨迹动画，再点 **执行 Execute**。
5. 夹爪滑条或开合按钮只设置目标，依次点击 **夹爪规划**、**执行夹爪**才执行。

速度和加速度默认各 10%，可分别设置 1%～100%。有效规划保留 30 秒；目标或比例改变会清除旧规划。按钮不可用时查看“等待授权”“重新规划”或操作尚未结束提示。只读启动后 JTC 为 inactive 是正常状态，许可和规划满足后首次执行才接管；勾选许可本身不运动。

| MoveIt 规划关节 | 最大速度 | 最大加速度 |
| --- | ---: | ---: |
| J1～J3 | 1 rad/s | 2 rad/s² |
| J4～J5 | 1.2 rad/s | 2.5 rad/s² |
| J6 | 1.5 rad/s | 3 rad/s² |
| 夹爪单指 | 0.05 m/s | 0.2 m/s² |

比例作用于规划限值，底层 POS_VEL 速度字段另受电机配置限制。轨迹预计用时不是规划计算耗时，也不是实机跟踪保证。此入口不调整电机固件 PID。

### MoveIt 的独立夹爪范围

[`gripper_limits.yaml`](gripper_limits.yaml) 从原模型两端各内收 0.7 mm：0% 对应单指 −0.7 mm，100% 对应 −34.3 mm。它是 MoveIt 目标／轨迹电子限位，**没有使用动力示教的四臂实测百分比端点**；不是硬件限位或标定后的物理夹口宽度。

反馈不裁剪，范围外可显示负百分比或超过 100%。范围外起点只允许向内恢复，轨迹检查失败则取消动作。可用 `role_overrides` 配置各臂余量，修改后重启；不执行自动顶住机械端点寻界。另一手指由 URDF mimic 联动。

## 碰撞配置与执行链路

[`collision_policy.yaml`](collision_policy.yaml) 默认 `bare`，会话运行期间不能切换。

| 配置 | 额外忽略的自碰撞连杆对 |
| --- | --- |
| `bare`，全部四臂 | `link1 ↔ link4`、`link1 ↔ link5` |
| `bare`，仅运动右臂 | 再忽略 `link2 ↔ link4` |
| `full` | 使用移植版完整矩阵，不添加上述例外 |

裸机配置来自未装装饰外壳的设备条件，但忽略的是**整对连杆**，不会区分外壳、电机和承力件。其余自碰撞与环境检查继续生效；这不等于精确裸机网格。目标通过逆解不代表起点及完整轨迹都有效，仍需看规划后复核结果。

```text
选择窗口 / RViz 目标与面板
  → CachedKDL 逆解与 OMPL 规划
  → MoveGroup / ExecuteTrajectory
  → MoveItSimpleControllerManager / FollowJointTrajectory
  → arm_controller + gripper_controller（标准 JTC，100 Hz）
  → 独立移植的 A6YDamiaoSystem（C++ SLCAN，POS_VEL）
```

Python 管理目标、原生 ROS action/service、界面和生命周期，不重写返回轨迹、不自行插值或发控制帧。Pinocchio 用于 A5 正运动学和 TCP 显示。此栈沿用 `command_timeout=0`、`feedback_timeout=0` 等参数，不能套用动力示教的看门狗描述。

## 停止、回零与关闭

| 操作／状态 | 当前行为 |
| --- | --- |
| 规划并执行回零 | 规划 J1～J6=0，预览后执行；夹爪保持开度，完成后保持零位 |
| 停止 | 取消 action 和排队执行，保留控制器；回零中停止会取消退出 |
| 已接管会话正常关闭 | 回零并核验六轴误差，再退出驱动并只读回查失能 |
| 未接管的只读会话关闭 | 直接关闭，不为退出首次使能或回零；不受许可勾选状态影响 |
| 规划／执行或退出回零失败 | 显示原因并保留会话；不跳过碰撞、不直接发六个零目标、不自动重试 |
| 已支撑 · 紧急失能 | 取消轨迹、请求失能，确认后恢复只读并撤销许可；不表示回零完成 |

已接管后撤销许可时，正常退出需恢复回零许可，或支撑后明确紧急失能。释放未确认不能当作只读退出。断电、USB 断开或强杀进程无法保证回零；“仅关闭显示”不证明控制端已失能。

## 曲线与日志

选择窗口展示 TCP、六关节、七电机和夹爪目标／反馈曲线。日志采样设为 20 Hz，界面约 10 Hz，支持 30／60／120 秒窗口；暂停曲线不停止记录。电机角度由关节反馈按方向、比例、零偏反算，不是原始 CAN 编码值。

首次回零后，以实测 TCP 建立会话零参考：XYZ 为 `p-p0`，RPY 为 `RPY(R0ᵀR)`。再次回零不重设参考；绝对 TCP 另行保留。只读启动且未回零时显示绝对参考；反馈过期、故障或无速度时留空，不用零替代缺失值。

会话位于仓库根目录 `diagnostics/arm_moveit_native_reports/<角色>_<编号>/`：

| 文件 | 用途 |
| --- | --- |
| `application.log`、`gripper.log` | 操作结果、许可、规划、夹爪事件 |
| `events.jsonl` | 结构化事件、错误码、碰撞连杆和轨迹关联 |
| `telemetry.csv`、`telemetry_metadata.json` | 连续反馈、单位、有效性、坐标与模型信息 |
| `session.json`、`live_status.json` | 会话报告和状态快照 |
| `plan_*.json`、`last_native_plan.json` | 原生 MoveIt 轨迹 |
| `move_group.log`、`controller_manager.log`、`rviz2.log`、`ros_logs/` | ROS 进程输出 |

通过“查看本次日志”“打开日志目录 / CSV”“查看历史日志”复核。预览有界，完整文件留磁盘；后台记录不进入实时控制循环，跨进程时序以原始时间戳为准。历史日志不自动清理。

## 开发、离线回归与来源

从仓库根目录准备独立环境，再顺序执行；不与实体会话共用 domain：

```bash
source A6-Y-G/scripts/environment.bash
source A6-Y-G/scripts/build_moveit_stack.bash
source A6-Y-G/scripts/build_moveit_panel.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=196
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m unittest discover \
  -s A6-Y-G/tools/arm_moveit -p 'test_*.py'

# 真实 MoveIt/JTC + C++ dry_run／PTY，不打开实体串口
/usr/bin/python3 A6-Y-G/tools/arm_moveit/verify_native.py
/usr/bin/python3 A6-Y-G/tools/arm_moveit/verify_native_serial.py --role motion_left
```

`verify_native_serial.py` 只打开它创建的 PTY，四角色可分别指定；`--readonly-close-test` 检查未接管退出，`--rviz` 需要显示环境。另有 [`verify_native_interaction.py`](verify_native_interaction.py) 和 [`verify_startup_home.py`](verify_startup_home.py)。`verify_bare_collision.py`／`verify_right_collision.py` 依赖本地历史会话，不是新克隆的通用自检。旧 Python 后端保留作参考，默认窗口使用 `native_session.py`；旧后端测试不能替代原生链路验证。

软件测试覆盖规划、模式、指令与退出，不证明实机跟踪精度、夹持力或碰撞网格准确性。本页不将历史 dry_run／PTY／Qt 结果扩展为全部功能真机验收。历史环境曾记录 `move_group` 退出析构异常，不能据此宣称系统 MoveIt 库已修复。

移植来源为提供给项目的 `A6_Y-main(1).zip`，保留归档包作者与许可。来源清单见 [`upstream_manifest.json`](ros2_stack/upstream_manifest.json)，适配差异见 [`upstream_changes.patch`](ros2_stack/upstream_changes.patch)。当前 A5 URDF、方向、零偏、夹爪比例与 PMAX 由本工程关联，不能直接用旧归档模型覆盖。
