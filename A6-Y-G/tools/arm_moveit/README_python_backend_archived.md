> 历史记录：此文档对应已停用的 Python 执行后端，不是当前启动入口。当前说明见 README.md。

# 四臂 MoveIt / Pinocchio / RViz 规划执行

这个入口支持选择运动端／示教端的左臂或右臂，在 RViz 拖动末端目标，先规划和预览，再执行。
一次连接一条臂，身份、位置量程和电机→模型映射复用前一步只读检查的配置。

## 直接复制启动

在 Ubuntu 图形桌面的终端复制下面整行，不用切换目录。脚本会在首次启动或面板代码更新后自动构建 RViz 插件，后续直接启动：

```bash
bash "/home/taoqiu/文档/ChatGPT/A6Y远程遥操作/A6-Y-G/scripts/run_arm_moveit.bash"
```

1. 选择运动端左臂／右臂或示教端左臂／右臂。
2. 先用默认的“模拟规划执行（不连接硬件）”，点击启动，等待状态显示“拖动 RViz 末端坐标轴”。
3. RViz 顶部使用 **Interact** 工具。默认显示 **XYZ + Roll / Pitch / Yaw**：拖球或直线箭头改变 XYZ，红／绿／蓝圆环分别绕 X／Y／Z 轴调整姿态。绿色目标模型实时跟随逆解，灰色实体模型显示反馈。**仅位置**默认不勾选；需要姿态自由时才勾选，此时隐藏旋转圆环。
4. 绿色目标表示逆解和碰撞检查通过；红色目标表示正在检查或不可达，面板显示原因，目标模型保留上一次有效姿态。可以缩小拖动距离、切换仅位置，或复位到当前末端。
5. RViz 左侧 **A6Y 规划与夹爪** 面板里点击 **① 规划 Plan**。蓝色半透明模型播放经过重定时和碰撞检查的路径，灰色实体模型显示当前反馈。
6. 在同一 RViz 面板点击 **② 执行 Execute**。模拟模式下仅更新模拟关节，不连接串口。
7. 真机使用时先关闭模拟会话，重新选臂，将模式切换为 **真机反馈与规划执行**，然后启动。
8. 核对选定臂、当前模型和实物后，勾选 **已核对本臂零位与方向，允许真机执行**。拖动、规划、预览之后，点击执行才开始电机准备和运动。

RViz 原生面板、独立选择窗口以及目标右键菜单共用同一个规划与执行后端。
RViz 面板和选择窗口的真机许可同步；只有对应分组的有效规划才会点亮执行按钮。
若仍显示旧界面，请先关闭旧会话，再运行上面的启动命令。

### 夹爪可视化操作

1. 在 RViz 面板的 **夹爪开合 · MoveIt** 区域拖动开度滑条，或点击 **闭合目标 0% / 张开目标 100%**。
2. 松开滑条后，绿色半透明手指显示目标位置，实心手指仍显示反馈。设置目标不会发送运动指令。
3. 点击 **① 夹爪规划**，预览夹爪路径，再点击 **② 执行夹爪**。
4. 机械臂与夹爪顺序执行；新的目标或规划会替换上一条待执行规划。

开度百分比对应模型中单指行程：当前映射为 0% → 0 mm、100% → −35 mm；
第二根手指由 URDF mimic 联动。这不是标定后的两指物理间距，也不提供夹持力控制。
运行时还会与该臂的电机 PMAX、方向和比例取交集，不越过已知位置范围。
单独执行夹爪只准备并使能 ID 7；若六轴已由本会话使能，则保持原位。
选择窗口中也补有开度、张开／闭合目标和夹爪规划，通用执行按钮会注明当前待执行的是机械臂还是夹爪。
窗口始终显示当前 TCP、目标 TCP、六个关节角、夹爪位置、规划状态和执行状态。
关闭会话后需重新选择机械臂；许可不会继承到下一次启动。

“目标复位到当前末端”把目标操作柄和预览复位到当前反馈，不驱动真机回零。
拖动时后台连续求解最近的目标，显示受 URDF 关节范围及整机碰撞检查约束的模型；拖动不驱动真机。执行中拒绝修改目标，先停止再重新规划。
规划为关节空间路径，末端不保证沿直线行走。

## 回零、停止与退出

- **规划并执行回零 · J1～J6 = 0**：先停止当前轨迹，读取新鲜反馈，由 MoveIt 规划六个关节到标定后的零位；检查整条路径后展示 1 秒，再自动执行。成功后保持零位，不失能。夹爪保持当前开度。
- **回零后关闭会话**、选择窗口的关闭按钮、RViz 的关闭／Exit 和正常 SIGINT/SIGTERM：执行同一回零流程，核对六轴反馈均在 ±0.005 rad（约 ±0.29°）后，再失能并等待确认，最后关闭 ROS 进程和串口。重复关闭请求不会重新开始回零。
- **停止轨迹 · 保持当前位置**：中断普通运动或回零，并取消等待中的退出。迟到的规划和碰撞检查结果不能重新启动运动。
- **规划失败、起点或路径碰撞**：保留会话、错误和已有的位置保持，取消退出；不改为直接发送六个零位置。到位反馈复核失败也会取消正常退出；执行端故障按下面的故障停止流程处理。
- **已支撑 · 紧急失能（跳过回零）**：用于无法回零时的明确人工退出途径，停止并请求失能，同时撤销执行许可。确认反馈后可关闭只读会话。此操作不宣称已回零。
- **只读且未授予执行许可的会话**：关闭时不使能电机、不自动运动；若仍有本会话已使能的电机，则阻止正常退出，提示恢复回零许可或支撑后紧急失能。
- **跟踪偏差超限、终点超时、未规划关节移动**：在反馈新鲜且电机状态正常时中止本次轨迹，已使能关节保持当前反馈位置；保留 MoveIt、RViz 和反馈。旧规划及排队动作作废，撤销真机执行许可；若正在回零退出，则取消退出。检查后重新授予许可、复位目标、规划和执行，不自动重试。
- **反馈丢失或驱动故障**：停止并尝试失能，保留界面和错误记录，禁用规划／执行。反馈模型冻结并标记失联。通信线程结束且确认释放后，用户可关闭会话并重新连接；失能回读未确认时不宣称电机已释放，也不强行回零。
- **规划进程故障、异常崩溃**：停止并尝试失能，不在状态未知时规划运动。正常退出保护不能保证断电、USB 断开、SIGKILL 或进程崩溃时仍能回零。

这里的零位是映射后的关节角 J1～J6 全部为 0，不是启动时采样的任意姿态，也不是 TCP 的 XYZ 全为 0。回零使用现有标定和 PID，不修改电机参数；更紧的回零反馈判定只决定是否允许退出。尚未进行本次真机回零验证。

回零按钮是一键规划并执行，需要现有真机执行许可。回零期间其他目标和参数操作锁定，停止与紧急失能始终保留。失能可能失去重力支撑，操作前仍需可靠支撑机械臂。

## 规划与控制架构

```text
RViz 六轴末端操作柄
  → Pinocchio 4：TCP 正运动学、带关节范围的末端位姿逆解
  → MoveIt 2 / OMPL RRTConnect：关节空间规划、URDF/场景碰撞检查、时间参数化
  → Pinocchio FK + RViz MarkerArray：绿色目标模型、蓝色轨迹动画及 TCP 路径
  → 用户点击执行
  → 复核反馈、起点、夹爪位置、插值轨迹的碰撞与数值范围
  → 原 CANable2 传输驱动 / DM POS_VEL：J1～J6 或夹爪 ID 7 位置控制
```

Pinocchio 运动学实现在 `motion_model.py`，处理原 URDF 的原生 mimic，并以 `tool_tcp` 作为末端。
MoveIt 接收 Pinocchio 求解得到的关节目标；自定义 RViz 操作柄和选择窗口调用规划服务。
本入口加载原生 RViz `a6y_rviz_panel/PlanningPanel` 控制面板；模型预览由 `preview.py` 使用 Pinocchio 计算每个 URDF visual 的位置，并通过标准 MarkerArray 显示。移除了独立的 MoveIt RobotState/Trajectory 显示插件，避免继续使用用户日志中出现崩溃的显示组合；尚无证据断定旧版 -11 的唯一原因。
面板通过本会话的 `ui/command` / `ui/status` 与同一个执行后端通信；反馈超过 2 秒未更新时禁用规划／执行按钮。
InteractiveMarkers 使用 Humble 的 `Interactive Markers Namespace` 字段；绿色球支持三维平移，六轴操作柄支持轴向平移与旋转。
`move_group` 的通用轨迹执行功能关闭，所有真机输出由选定臂的独立执行模块负责。

原 CAD 的关节速度字段为 0，运行时模型副本为六轴设置明确的速度上限。
URDF 原文件、零位标定文件、旧控制器及安装目录均不修改。
SRDF 排除相邻连杆和两手指之间的接触，其他碰撞检查保留。
URDF 没有周围物体的信息；本入口不自动识别桌面、人员或其他机械臂。
可通过当前会话的 `planning_scene` / `collision_object` 话题添加已建模障碍物。
执行前复核完成后发生的环境变化仍需要操作员停止；这里不是动态避障系统。

## 四臂映射

| 角色 | USB 序列号 |
| --- | --- |
| 运动端左臂 `motion_left` | `304C33733036` |
| 运动端右臂 `motion_right` | `209B3971534B` |
| 示教端左臂 `teaching_left` | `304F33733036` |
| 示教端右臂 `teaching_right` | `209D3982534B` |

继续使用 `diagnostics/arm_roles.yaml` 和 `tools/arm_visualization/joint_mapping.yaml`。
连接时核验 USB 序列号、by-id 路径和占用，不依赖 ttyACM 编号。
四臂暂共用 A5 模型；左右臂不自动镜像，方向、比例和本次零位必须沿用前一步实物核对结果。
执行使用映射的逆变换，将模型角度换算为电机角度；不自动写零或抵消初始偏移。
夹爪沿用所选臂的 ID 7 映射；未执行夹爪时仍只读。完整反馈参与模型和碰撞检查。

## 执行条件与数值范围

- 真机启动只有七电机状态查询。只有点击执行且界面许可有效时，才进入位置控制准备。
- 首次执行要求本次要使能的目标电机为失能状态；不接管其他程序已经使能的电机。
- 只允许选定六轴或夹爪切换 RAM 控制模式为 POS_VEL，并要求匹配 ID 的模式响应。先预载当前位置，再使能并回读确认。
- 不写 Flash、不写零位、不修改 ID、增益、PMAX 或其他参数。退出不自动恢复原控制模式；后续原项目控制器会自行准备所需模式。
- 机械臂最大关节速度 **0.25 rad/s**、最大规划加速度 **0.5 rad/s²**；夹爪单指上限 **3 mm/s**、**6 mm/s²**。速度和加速度比例分别设置，均默认 50%；RViz 面板和选择窗口同步。修改任一参数会使旧规划失效，必须重新规划。
- 原生电机 POS_VEL 的速度参数是速度上限。发送值使用轨迹速度加位置误差修正，并受用户速度上限、加速度包络限制；终点有偏差时继续收敛。加速度设置作用于规划轨迹及发送速度包络，不是写入电机固件的加速度寄存器，也不保证实测加速度精确等于设定值。实际跟踪取决于内部位置环、负载和通信。
- 反馈新鲜度上限 250 ms；目标控制周期 50 ms，周期超过 250 ms 时中止。这里不宣称硬实时保证。
- 轨迹必须从零时刻单调递增，最长 120 秒；位置、速度、时间必须有限，端点速度接近零。
- 三次 Hermite 插值检查位置、速度和加速度的极值，位置范围仅留 0.001 rad（臂）或 0.05 mm（单指）数值容差。
- 执行前由 MoveIt 按不超过 40 ms 的轨迹时间间隔检查插值路径（最高速度对应不超过约 0.01 rad 的关节步进）。这是离散碰撞检查。
- 规划生成后 30 秒内才能执行；当前六轴与规划起点的最大偏差不得超过 0.03 rad；夹爪位移变化不得超过 0.5 mm。
- 执行跟踪偏差超过 0.15 rad（臂）或 3 mm（单指）时中止；终点误差需在 0.02 rad 或 0.5 mm 内，否则最多等 3 秒后报告各关节误差。
- 停止操作会使在途检查和已排队的执行请求失效，迟到的服务回复不能重新启动轨迹。

规划失败不会启用任何电机。遇到不可达目标、自碰撞、范围冲突或通信问题，窗口显示具体错误。
轨迹中止后可以在同一会话内重新规划；总线／驱动故障需要排查并重新连接。两种情况均不自动重新执行。

## 环境、记录与验证

当前电脑已有 ROS 2 Humble、MoveIt 2、OMPL、Pinocchio 4.0、PyQt5、SciPy、python-can 和 pyserial。
启动脚本加载系统 ROS 和本入口的独立面板安装目录。
Python 后端直接运行源码；C++ 面板源码在 `rviz_panel/`，自动构建产物在 `build/arm_moveit_ui/`，不重建原硬件控制包。
需要系统的 `colcon`、CMake、Qt5 开发文件及 ROS Humble 的 RViz 开发包；当前电脑已具备。
ROS 仅在本机通信，默认 domain 196；可以用 `A6Y_MOVEIT_DOMAIN_ID` 修改。
每次会话使用独立 `/a6y_moveit/<角色>/s<随机值>` 命名空间隔离模型、TF、规划服务和目标。
串口与前一步只读显示／扫描／零位标定使用相同互斥机制，需要先停止其他占用程序。

每次会话的模型参数、SRDF、RViz 配置、ROS 日志、最后状态和最后 100 条电机发送记录保存在
`diagnostics/arm_moveit_reports/<角色>_<唯一目录>/`。错误按顺序保留在 `events.jsonl` 及 `session.json` 的 events 字段，避免后续显示退出覆盖前面的规划失败原因。轨迹中止当时即写入 `execution_failure`，记录分组、轨迹时间、速度／加速度比例以及各关节目标、实测位置、误差、限值和单位，不必等关闭窗口才能查看。RViz 错误区域可滚动，长错误不会挤掉停止按钮。这些是排查记录，不是实机验收或连续轨迹记录。

从 `A6-Y-G` 目录运行配置检查：

```bash
bash scripts/run_arm_moveit.bash --check-config
```

可选的终端数字菜单只启动模拟模式，规划／执行可用 RViz 面板及目标右键菜单：

```bash
bash scripts/run_arm_moveit.bash --cli
```

无硬件自动测试：

```bash
source scripts/environment.bash
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -s -m unittest discover \
  -s tools/arm_moveit -p 'test_*.py' -v

# 实际 MoveIt 服务、Pinocchio IK、场景碰撞和模拟执行；强制禁止连接硬件
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=197 ROS_LOG_DIR=/tmp/a6y-moveit-verification-logs
/usr/bin/python3 tools/arm_moveit/verify_planning.py

# 回放日志中的零位附近关节状态和 +0.38953 mm 夹爪残差（模拟电机）
/usr/bin/python3 tools/arm_moveit/verify_boundary.py

# 回零、碰撞阻挡、停止取消退出及退出前失能顺序（模拟电机）
/usr/bin/python3 tools/arm_moveit/verify_home.py

# 跟踪中止后继续反馈／重新规划、回零失败取消退出、通信故障保留界面
/usr/bin/python3 tools/arm_moveit/verify_execution_failure.py
# 有图形显示时加 --rviz，可同时检查 RViz 子进程仍在运行
```

测试包含运动学往返、非法轨迹和极值、执行指令白名单、PTY 模拟实际串口收发、只读到执行的切换、使能/失能回读、取消与保持、过期反馈、窗口选择和执行许可。
ROS 集成测试直接启动 MoveIt，验证规划服务和合成轨迹执行，并注入/移除虚拟障碍物检查碰撞处理。
本次开发没有执行真实电机运动。42 项单元／PTY／窗口测试通过，包括零位边界附近的位置逆解、独立加速度重定时、真实日志夹爪残差、70% 速度响应和完全不动的电机模拟，以及失败后的反馈与窗口存活。MoveIt 集成测试覆盖连续目标预览、不可达目标保持、场景障碍物、机械臂和夹爪规划执行。真机动力学、夹爪物理开度与方向仍以实物反馈为准。

接口参考：[MoveIt Humble 运动学文档](https://moveit.picknik.ai/humble/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html)、
[MoveIt Humble 示例](https://moveit.picknik.ai/humble/doc/examples/examples.html)。

夹爪使用确定性的一维开合轨迹，不调用六轴 IK 或依赖 OMPL 搜索。时间参数化使用端点静止、单调的三次曲线，速度和加速度独立限幅；MoveIt 对规划前和执行前的完整模型逐点检查碰撞，RViz 动画与实际执行采样使用同一条曲线。
这样避免 Humble TOTG 对毫米级路径的末段减速度突变。执行前仍逐段复核完整模型碰撞、数值范围、反馈与起点。

本次 RViz 配置修复参考：[Humble InteractiveMarkerDisplay 源码](https://github.com/ros2/rviz/blob/humble/rviz_default_plugins/src/rviz_default_plugins/displays/interactive_markers/interactive_marker_display.cpp)。

新版本界面记录和截图：`diagnostics/arm_moveit_reports/motion_left_ahv36jnc/`；MoveIt 集成记录：`motion_left_01ct1exo/`；真实反馈数值的夹爪模拟回放：`teaching_left_6_4nnb6h/`（均位于项目上级目录的 `diagnostics/arm_moveit_reports/`）。

## 对照用户旧程序的本次修改

参考 `A6_Y-main(1).zip` 中的 `end_effector_control_server.cpp`（独立速度、加速度和 position_only）、`a6_y_damiao_system.cpp`（位置模式速度处理）、MoveIt SRDF 和配置。当前模型与旧包的关节名称、夹爪正负方向、TCP 和部分关节范围不同，因此沿用当前已核对的映射，不直接覆盖旧包 URDF、零位或放宽碰撞排除。

日志中的示教左臂夹爪闭合反馈为 +0.38953 mm，而当前模型上限为 0。此前直接用这个起点构造严格范围内的轨迹会失败。本版保留原始反馈和警告：规划副本只对臂 ±0.001 rad、夹爪 ±0.5 mm 内的边界残差做归一；夹爪执行轨迹则从原始反馈开始，仅允许单调恢复到合法目标。不会扩大可选择的目标范围或修改标定偏移。更大的残差拒绝规划并提示重新标定。建议根据警告复核零位，尤其是持续出现同方向偏差时。

### 参数接口

RViz 和选择窗口的速度、加速度为 10%～100%，分别作用于上述物理上限。默认 50% 对应六轴 0.125 rad/s、0.25 rad/s²，单指 1.5 mm/s、3 mm/s²。改变后重新规划，执行时使用这份规划记录的比例。

本会话命名空间下 `ui/command` 为 `std_msgs/String`，JSON 示例：

```json
{"command":"settings","value":{"velocity_scaling":0.5,"acceleration_scaling":0.2,"position_only":true}}
```

`plan` 和 `gripper_plan` 也接受同样的 value 对象；不带 value 使用面板当前参数。`execute_arm` / `execute_gripper` 只执行匹配分组的有效预览轨迹。`home` 一键规划执行六轴回零并保持；`shutdown` 回零后退出；兼容的 `release` 也改为回零保持。`stop` 取消回零和退出，`emergency_release` 明确跳过回零并请求失能。普通用户直接用界面，无需手动发送话题。

时间参数化接口参考：[MoveIt Humble 独立速度与加速度比例](https://moveit.picknik.ai/humble/doc/examples/time_parameterization/time_parameterization_tutorial.html)。

回零、姿态入口和执行异常处理修改均未调整电机位置环／速度环 PID、增益寄存器或控制周期。回零模拟测试记录：`diagnostics/arm_moveit_reports/motion_left_tylvvf5k/session.json`（相对项目上级目录）。

当前 RViz 姿态拖动和 Quit 回零流程的截图及记录：`diagnostics/arm_moveit_reports/motion_left_5b857_fl/`（相对项目上级目录），包括三轴旋转拖动和退出回零过程。

### 2026-11-25：执行失败导致窗口退出的定位

示教右臂 `teaching_right_fo9yt4hv` 和 `teaching_right_qeezqpsl` 的首个执行故障都是“轨迹跟踪偏差超过允许范围”。旧会话循环发现硬件线程 error 后抛异常，退出上下文并向 MoveIt／RViz 发送关闭信号。后一份日志先出现该执行错误，再出现 move_group 的 `signal_handler(SIGINT/SIGTERM)`，最后才有析构时的段错误。因此这两次退出的触发点是自定义执行层及异常传播；析构段错误仍是独立的关闭阶段问题，本次不宣称已修复它。

用户原压缩包的 `A6_Y_Moveitconfig/config/ros2_controllers.yaml` 使用 100 Hz 的 `joint_trajectory_controller`、position 指令接口；当前入口则是 50 ms 周期的 Python POS_VEL 执行层，另有 0.15 rad 跟踪门限和速度包络。它们并非相同的执行实现。最新记录里 J2／J3 的实测位置与最后已发送位置存在约 0.14 rad 量级差距，但旧日志未保存触发瞬间的插值目标，不能据此精确断定是哪个关节先超限，也不能直接断定 PID 是原因。

本次保留原跟踪门限、速度／加速度上限和控制周期，修复执行结果与界面生命周期耦合，并记录触发时的具体数值。实际电机滞后的原因仍需结合下一次记录定位，不把“窗口不再退出”等同于“真机跟踪已修好”。

本次无硬件集成验证记录（相对项目上级目录）：

- `diagnostics/arm_moveit_reports/motion_left_j80rp4a7/`：实际 MoveIt／RViz、模拟执行故障、恢复后重新规划执行、退出回零故障保留界面、通信故障锁定。包含 `tracking_failure_alive.json`、`home_failure_alive.json` 和 `rviz_tracking_failure_alive.png`；最终 `TEST ONLY` 通信错误是测试注入。
- `diagnostics/arm_moveit_reports/motion_left_wz3u7loi/session.json`：正常回零退出、碰撞阻挡和停止取消退出复测通过。两次测试均未连接物理电机，CAN 指令数量为 0。
