# A6-Y-G：A5 机械臂与夹爪重力补偿

仓库安装、当前四臂入口和目录说明见 [A6Yarm 首页](../README.md)。本文保留主工作区及早期控制器调试细节；当前四臂行为以对应模块文档为准。

基于 ROS 2 Humble、ros2_control 和 Pinocchio 的七电机控制项目，使用完整 A5 CAD 模型，实现六轴机械臂重力补偿、夹爪反馈、只读诊断和无硬件验证。

项目名为 A6-Y-G，实际模型为 `src/A5/urdf/A5.urdf`。模型包含六个旋转关节和两个联动手指，共七个独立自由度。第二手指由 URDF mimic 描述，不对应额外电机。

文档已按中文文件夹分类，入口见 [文档导航](docs/README.md)。

## 四臂零位标定后的只读模型检查

在本机桌面终端执行 `bash scripts/run_arm_rviz.bash`，选择运动／示教端及左／右臂，
打开实时只读反馈与 RViz；也可先预览 URDF 零姿态。此入口无需构建，不加载控制器。
使用方法、显示换算与验证边界见 [四臂 RViz 检查说明](tools/arm_visualization/README.md)。

## 四臂 MoveIt 末端规划与执行

运行 `bash scripts/run_arm_moveit.bash`，选择运动／示教端的左／右臂和模拟／真机模式。
当前入口已移植用户提供的 `A6_Y-main(1).zip`：原版 MoveIt/CachedKDL → ExecuteTrajectory → FollowJointTrajectory → 标准 JointTrajectoryController → 原版 C++ ros2_control 驱动，100 Hz。Pinocchio 保留用于正运动学和模型绘制；旧 Python 轨迹执行器不再使用。

保留 RViz XYZ/RPY 拖动与受约束的目标模型、规划／执行、夹爪滑条、独立速度与加速度比例（原配置默认均 10%）、四臂选择、回零及正常退出回零失能。当前默认模拟模式，启动后自动规划回零可取消；只检查反馈时使用 `--read-only-start`。规划或动作失败保留会话并显示错误，不自动重试。零位映射不变，未调整 PID。

复制即用的命令、移植差异、首次执行接管七电机的行为和验证边界见 [当前 MoveIt / ros2_control 说明](tools/arm_moveit/README.md)。

## 功能与控制行为

动力示教入口：`bash scripts/run_arm_teaching.bash`。支持上位机选择 1 对 1 / 2 对 2 配对，默认左对左、右对右；启动不回零，只有示教端重力补偿，运动端目标固定 100 Hz 直接跟随示教端当前位置，底层控制与每轴 CAN 指令为 250 Hz，CAN 保持 1 Mbps；退出各臂同时原生位置回零并核验失能，超时保留窗口处理。详见 [动力示教说明](tools/arm_gravity/TEACHING.md)。

动力示教界面为暗黑风格，支持实时/历史日志、按臂筛选、复制诊断摘要与导出完整 ZIP 诊断包。运动端采用参考程序的 POS_VEL 模式；首次故障和退出故障分开保留，故障后不自动再次回零。

- **dry-run**：不打开串口，将命令回填为状态，检查模型、控制器和 ROS 通信；不是动力学仿真。
- **readonly**：读取七台真实电机的反馈，不自动使能。
- **gravity**：六轴输出重力补偿与速度阻尼，可逐轴加入连续摩擦补偿，夹爪默认零力；不会自动开合或保持夹持力。
- **诊断与标定**：总线扫描、单电机 ID 修改与保存、原始反馈连续采样、运行状态查看、STL TCP 推导和实测记录检查。

重力模式使用下式，MIT Kp/Kd 均为零：

```text
effort[i] = clamp(scale[i] * G(q)[i] - damping[i] * dq[i], -limit[i], limit[i])
```

夹爪位置参与整个模型的重力计算，但夹爪自身的重力比例与阻尼均为零。控制器通过 28 个命令接口控制七个独立关节，驱动等待完整命令后使能。

## 末端位姿阻抗控制

新增独立阻抗模式：启用时捕获 `tool_tcp` 当前的位置和姿态，手推允许偏移，松手后朝目标恢复。沿用本项目通信、50 Hz 控制周期及现有重力参数；夹爪保持零力。原重力模式入口不变。

```bash
# 已构建并加载本项目环境后，无硬件运行：
ros2 launch a6_y_controllers a5_cartesian_impedance.launch.py \
  dry_run:=true auto_enable:=false use_rviz:=true
```

构建、实机启动、参数和验证边界见 [固定末端位姿阻抗控制](docs/阻抗控制/CARTESIAN_IMPEDANCE.md)。无硬件验证使用 `tools/verify_impedance_dry_run.py`；dry-run 不是动力学仿真。

## 关节阻抗控制

独立的关节阻抗模式会捕获 joint1～joint6 的当前角度，手推允许偏移，松手后朝原角度恢复。刚度和阻尼按关节配置，夹爪保持零力；沿用当前重力补偿、通信及 50 Hz 周期。与重力、笛卡尔阻抗控制器互斥。

```bash
ros2 launch a6_y_controllers a5_joint_impedance.launch.py \
  dry_run:=false auto_enable:=true use_rviz:=true
```


参数文件为 `src/a6_y_controllers/config/a5_joint_impedance_controllers.yaml`。构建、实机入口和测试见 [固定关节角度阻抗控制](docs/阻抗控制/JOINT_IMPEDANCE.md)；无硬件验证脚本是 `tools/verify_joint_impedance_dry_run.py`。

## 环境与依赖

VR 虚拟遥操作已增加独立入口：Quest 右手柄经 Vuer、A5 逆运动学控制 RViz，默认使用 ROS domain 84，不加载硬件驱动。启动、无头显演示和录制回放见 [VR 虚拟机械臂使用说明](docs/VR遥操作/虚拟机械臂/VR_RVIZ_USAGE.md)；设计与参考项目分析见 [移植方案](docs/VR遥操作/虚拟机械臂/VR_RVIZ_PLAN.md)。

[VR 实机入口与使用说明](docs/VR遥操作/实机控制/VR_REAL_ROBOT_USAGE.md)已实现关节阻抗动态参考、实机姿态重锚定、prepare/arm 许可、松手保持与失联暂停。`scripts/run_vr_real.bash` 默认只读；`mode:=mock` 不连接 CAN。`mode:=execute` 要求通过标定与停止验收检查，当前记录仍待完成。设计见 [VR 实机接入方案](docs/VR遥操作/实机控制/VR_REAL_ROBOT_PLAN.md)。

2026-10-08 按用户要求新增原生位控入口 `bash scripts/run_vr_position.bash`（默认只读）：使用电机 `pos_vel` 位置闭环，不计算重力补偿/阻抗力矩。完整实机快捷入口为 `bash scripts/run_vr_position_full.bash`，开放末端平移、旋转和夹爪；使用 `bash scripts/vr_position_control.bash prepare`、`arm` 接管。完整配置参考 Piper 精简：关闭未标定 CAD 网格碰撞硬检查，取消局部试验位移约束、8 ms 单拍超时锁停与 IK 暂时不可达锁停；IK 数值失败保持当前参考并重试。保留行程、限速、失联/反馈超时、跟踪偏差和故障失能，但不提供自动自碰/环境避障。原 trial/阻抗配置仍执行 CAD 检查。网关与完整启动步骤见上述使用说明；实机动态效果由用户继续验证。

完整位控短暂丢失追踪后，同一手柄连接恢复并松开侧握把即可自动回到待跟随状态，再按下时从实际位置重新接管；首次启动、手动暂停、故障和网关重连仍需本地操作。

项目使用 Ubuntu 22.04、ROS 2 Humble，以及支持原生 mimic 解析的 **Pinocchio ≥ 4.0**。先准备好 `/opt/ros/humble/setup.bash` 和对应软件源，再在项目根目录执行：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-rosdep python3-can \
  python3-serial python3-yaml python3-numpy python3-scipy ros-humble-pinocchio
# 仅尚未初始化 rosdep 时执行下一条
sudo rosdep init
rosdep update
source scripts/environment.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
rosdep check --from-paths src --ignore-src --rosdistro humble
/usr/bin/python3 -c 'import pinocchio; print(pinocchio.__version__)'
```

若软件源提供的 Pinocchio 低于 4.0，需要先准备满足版本要求的安装；CMake 会拒绝旧版本。可选的 TCP 绘图还需要 `python3-matplotlib`。

`environment.bash` 加载系统 ROS，清理旧 overlay、Conda 和 Python 用户路径。建议在专用终端中使用；构建后通过 `a6y_load_install` 加载本项目并检查包来源。

## 构建

所有命令均从项目根目录运行：

```bash
source scripts/environment.bash
export MAKEFLAGS='-j2 -l2'
colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
a6y_load_install
```

内存不足时改为 `MAKEFLAGS='-j1 -l1'`。根目录存在 `COLCON_IGNORE`，因此必须显式使用 `--base-paths src`。

源码中以 `a6` 开头的文件已统一使用 `A6` 前缀；ROS 包名、目录、C++ 命名空间和控制器实例名仍为小写。构建与安装目录由 colcon 自动生成，不作为源码维护。

## 无硬件验证与显示

```bash
source scripts/environment.bash
a6y_load_install
export ROS_LOG_DIR="$PWD/validation/ros_logs"
export ROS_DOMAIN_ID=193
export ROS_LOCALHOST_ONLY=1
mkdir -p "$ROS_LOG_DIR"
ctest --test-dir build/a6_y_ros2_control --output-on-failure
ctest --test-dir build/a6_y_controllers --output-on-failure
/usr/bin/python3 tools/validate_bundle.py
/usr/bin/python3 tools/derive_a5_tcp.py
/usr/bin/python3 tools/test_s0.py
/usr/bin/python3 tools/verify_s0.py
/usr/bin/python3 tools/verify_dry_run.py
```

检查范围及结果文件见 [验证说明](docs/标定与验证/VALIDATION.md)。集成检查默认使用 ROS domain 193，可设置 `A6Y_TEST_ROS_DOMAIN_ID`。实物标定尚未完成不会使普通模型检查失败；严格检查使用 `--require-field-calibration`。

打开无硬件 RViz：

```bash
ros2 launch a6_y_controllers a5_gravity_comp.launch.py \
  dry_run:=true auto_enable:=false use_rviz:=true
```

## 真机操作

先支撑机械臂，核对电机 ID、方向、零点、基座安装与夹爪行程。扫描、反馈采样和硬件 launch 不能同时占用同一个串口。当前配置的串口为 `/dev/ttyACM0`，串口波特率为 115200，CAN 速率为 1000000 bit/s。

每个新终端先执行：

```bash
source scripts/environment.bash
a6y_load_install
```

1. 扫描七台电机，仅发送状态查询：

```bash
ros2 run a6_y_ros2_control damiao_bus_scan.py \
  --channel /dev/ttyACM0 --passes 10 \
  --config src/a6_y_controllers/config/a5_motors.yaml
```

2. 启动只读反馈与模型显示：

```bash
ros2 launch a6_y_controllers a5_gravity_comp.launch.py \
  dry_run:=false auto_enable:=false can_channel:=/dev/ttyACM0 use_rviz:=true
```

3. 停止只读 launch，确认机械臂有支撑后启用重力补偿：

```bash
ros2 launch a6_y_controllers a5_gravity_comp.launch.py \
  dry_run:=false auto_enable:=true can_channel:=/dev/ttyACM0 use_rviz:=true
```

结束前托住机械臂，再按 Ctrl+C；停用会撤去补偿并失能。夹爪同样参与使能，但默认输出零力。软件位置范围不等于实体限位保护。本项目的离线验证不代表实物方向、补偿比例或负载已完成验收。

启动参数还包括 `can_bitrate`、`tty_baudrate`、`motor_config_file` 和 `controllers_file`。后两项可传入自定义 YAML 的绝对路径。默认 `dry_run=true`、`auto_enable=false`、`use_rviz=false`。

## 状态与反馈诊断

运行中的 ROS 节点位于 `/arm6` 命名空间：

```bash
ros2 run a6_y_ros2_control motor_status.py \
  --topic /arm6/dynamic_joint_states --watch --require-feedback \
  --fail-on-fault --max-feedback-age 0.5
```

`/arm6/joint_states` 包含七个独立关节；按消息中的 `name` 配对位置，不假设数组顺序。第二手指的 TF 由 robot_state_publisher 根据 mimic 生成。

独立采样 J4 原始反馈前，先退出占用串口的 launch。下列工具不会使能或标零；日志文件必须尚不存在：

```bash
ros2 run a6_y_ros2_control damiao_feedback_watch.py \
  --channel /dev/ttyACM0 --config src/a6_y_controllers/config/a5_motors.yaml \
  --duration 20 --expect-joint joint4 --log /tmp/A6_joint4_feedback.jsonl
```

看到连接就绪后，缓慢改变 J4 两侧连杆的相对角度并在不同角度停留。只有确认实物确实运动，位置跨度结果才可用于判断动态反馈。

## 修改并保存电机 ID

新增 `damiao_set_id.py`，设置 `ESC_ID` 和 `MST_ID`，确认失能、逐步读回并保存。每次只连接一台待设置电机，停止占用串口的程序并支撑好机械臂。以下示例把实际旧 ID `0x01` 设置为 J4 的 `0x04 / 0x14`：

```bash
ros2 run a6_y_ros2_control damiao_set_id.py \
  --channel /dev/ttyACM0 --current-id 0x01 --new-id 0x04 --write
```

省略 `--write` 时只做离线预览，目标反馈 ID 默认是新电机 ID 加 `0x10`。结束后将电机断电再上电，用 `--current-id 0x04 --current-master-id 0x14 --verify` 只读核对。从未知 ID 扫描、逐台编号、写入保存到断电验证、整臂对应核对及失败恢复，见 [电机 ID 标定完整流程](docs/电机调试/MOTOR_ID_SETUP.md)。

## 电机映射与单位

电机内部的位置环/速度环增益、加减速参数现在可通过移植的三个工具读取、临时修改和保存，具体命令见 [电机参数编辑](docs/电机调试/MOTOR_PARAMETERS.md)。这些寄存器与下面的关节映射、ROS 控制器限速分别生效。

原生位置环的实测数据与适用范围见 [J1 低速调参](docs/电机调试/J1_POSITION_TUNING_20261009.md)、[J1 90°/s 调参](docs/电机调试/J1_HIGH_SPEED_TUNING_20261009.md)、[J2 低速调参](docs/电机调试/J2_POSITION_TUNING_20261009.md)、[J2 90°/s 调参](docs/电机调试/J2_HIGH_SPEED_TUNING_20261009.md)、[J3 低速调参](docs/电机调试/J3_POSITION_TUNING_20261009.md) 和 [J3 90°/s 调参](docs/电机调试/J3_HIGH_SPEED_TUNING_20261009.md)。[J4 高速调参](docs/电机调试/J4_HIGH_SPEED_TUNING_20261009.md)仍在进行，尚未达到 90°/s 目标。J2 最新选值为 `KP_APR=2500`、`KP_ASR=0.007296`、`KI_ASR=0.0016`；J3 为 `KP_APR=2500`、`KP_ASR=0.004608`、`KI_ASR=0.002`，上述参数随后已按用户要求执行 Flash 保存并确认应答。日常 VR 速度上限未随单轴试验改变。

**最新保存状态（2026-10-10）：**用户确认 J4 当前在 **40°/s 规划下运动稳定**，实读参数为 **`KP_APR=220、KP_ASR=0.001096、KI_ASR=0.0005、KI_APR=0、ACC/DEC=2/−2`**。J1～J4 当前全部电机 RAM 参数已逐台保存到 Flash，均收到保存应答，保存前后各 24 项参数一致，七电机独立确认失能。掉电保持待断电重启复核。当前共享位置跟踪门限为用户调整后的 `0.25 rad`，本次未改；现场平稳评价与数值验收分开，详见 [当前参数与 Flash 保存记录](docs/电机调试/J1_J4_FLASH_SAVE_20261010.md)。以下段落保留此前阶段记录。

2026-10-10 的 J4 续调新增 8 次实机试验：4 次完成 10°/s 往返，4 次 20°/s 均触发速度保护，尚未完成高速验收。已恢复低速候选 `KP_APR=54`、`KP_ASR=0.008184`、`KI_ASR=0.0005`、`ACC/DEC=2/−2`（仅 RAM），独立确认七电机失能。下一步用新增故障拍日志核对速度突跳，详见上述 J4 记录。

随后完成 [J4 速度反馈离线标定](docs/电机调试/J4_SPEED_FEEDBACK_CALIBRATION_20261010.md)：`VMAX=50` 匹配，模型坐标零偏约 `+0.700°/s`，倍率保留 1；11 次往返扣零后的整段积分最大误差为 1.04%。在线反馈和保护链未改，短时速度峰值仍需动态核验。

标定后继续 4 次 20°/s 试验，均去程中止。按用户要求，**J4 高速测试脚本的实际速度门限下限已从 0.50 调为 0.55 rad/s**；保留超速停机、指令速度及其他保护，日常 VR 配置未改。新门限轮走到约 73.90°，在 0.57387 rad/s 再次保护停止，尚未完成往返。两种增益对照现场抖动未改善，已恢复上述低速候选，七电机独立确认失能。结果与 47 项检查、mock 验证见 [J4 最新记录](docs/电机调试/J4_HIGH_SPEED_TUNING_20261009.md)。

**此前阶段配置：**随后按用户明确要求，J4 测试的实际速度门限下限由 0.55 改为 **1.0 rad/s（57.30°/s）**，20°/s 档采用该值；90°/s 档仍为 1.9 rad/s。指令速度、其他保护及日常 VR 配置未改。该门限下已完成 6 次实机对照，3 次完整往返、3 次中止。当前 RAM 候选为 **`KP_APR=60、KP_ASR=0.006696、KI_ASR=0.0005、ACC/DEC=2/−2`**：90°往返现场运动和到位都平稳，但 20°/s 档双向实测峰值仅约 15.10/14.15°/s，尚未通过速度验收。位置增益继续升到 68 后重新抖动并保护停止，已恢复 60；七电机独立确认失能，未保存 Flash。详见 [J4 最新记录](docs/电机调试/J4_HIGH_SPEED_TUNING_20261009.md)。

此前继续调试已修复 J4 失能确认等待被力矩检查打断的问题，69 项离线检查和 mock 通过，随后 3 次实机均收到失能确认。速度积分 0.0006 导致近端严重抖动并跟踪保护停止；恢复 0.0005 后完整往返且现场平稳；位置比例 62 虽走完全程但到位晃动，已恢复 60。当时恢复为上述 `60 / 0.006696 / 0.0005` 候选，最新复测去/回峰值约 **15.79/14.04°/s**，未达到 20°/s 数值要求。最终七电机独立确认失能，未保存 Flash，详见 [J4 最新记录](docs/电机调试/J4_HIGH_SPEED_TUNING_20261009.md)。

配置位于 `src/a6_y_controllers/config/a5_motors.yaml`。关节顺序固定为 `joint1`～`joint6`、`gripper_finger_joint1`。

| 关节 | 电机 | slave / master ID | sign | position_scale |
| --- | --- | --- | --- | --- |
| J1 | DM4340P | 0x01 / 0x11 | −1 | 1 |
| J2 | DM4340P | 0x02 / 0x12 | +1 | 1 |
| J3 | DM4340P | 0x03 / 0x13 | +1 | 1 |
| J4 | DM4310 | 0x04 / 0x14 | −1 | 1 |
| J5 | DM4310 | 0x05 / 0x15 | −1 | 1 |
| J6 | DM4310 | 0x06 / 0x16 | −1 | 1 |
| 第一手指 | DM3507 | 0x07 / 0x17 | +1 | 0.0239 m/rad |

```text
q_joint = sign * q_motor * position_scale + offset
effort_motor = sign * effort_joint * position_scale
q_finger2 = -q_finger1
```

当前 offset 均为零。机械臂位置、速度和 effort 单位为 rad、rad/s、N·m；夹爪为 m、m/s、N。夹爪工作范围为 `[-0.03, 0] m`，CAD 几何范围为 `[-0.035, 0] m`；负方向打开。夹爪 30 N 广义力约对应 0.717 N·m 电机力矩，不代表接触力已标定。

## 调参与模型标定

重力参数位于 `src/a6_y_controllers/config/a5_gravity_controllers.yaml`。下表是当前配置快照，实机参数尚未完成标定：

| 参数 | J1～J6、夹爪 |
| --- | --- |
| `torque_scales` | `[1.3, 1.1, 1.0, 1.1, 1.3, 1.3, 0]` |
| `torque_limits` | `[10, 20, 20, 3, 4, 4, 30]`，末项单位 N；不是额定或允许持续力矩的声明 |
| `damping_gains` | `[0.01, 0.01, 0.01, 0.01, 0.01, 0.03, 0]` |
| `friction_compensations` | `[0.15, 0, 0.15, 0, 0, 0, 0]`，J1/J3 的试调值，单位 N·m，夹爪单位 N |
| `update_rate` | 50 Hz |

先停止 launch，再修改参数并重启。运行中修改 ROS 参数不会刷新控制器缓存。逐轴倍率覆盖标量倍率；连续摩擦补偿已开启，旧摩擦脉冲和外部力反馈默认关闭。驱动保留反馈检查、反馈超时、写周期检查与停用失能。当前 J2/J3 的控制器力矩上限与 `a5_motion_limits.yaml` 中的 10 N·m 不一致，实物验收前需核实；不要为了减轻拖动阻力继续提高上限。

### 连续摩擦补偿

数组顺序为 J1、J2、J3、J4、J5、J6、夹爪。补偿在**关节坐标系**中沿测得的运动方向输出，由驱动统一映射到电机方向；不要按电机 `sign` 给补偿幅值加负号。重力比例 `torque_scales` 和 `effort_signs` 不作用于摩擦项。

```yaml
friction_compensation_enabled: true
friction_compensations: [0.15, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0]
friction_velocity_deadbands: [0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.001]
friction_velocity_scales: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.005]
friction_ramp_duration: 0.2
friction_pulse_enabled: false
```

| 参数 | 含义 |
| --- | --- |
| `friction_compensation_enabled` | 连续补偿总开关；插件默认 false，此配置显式开启 |
| `friction_compensations` | 每轴最大补偿幅值，非负；0 表示该轴关闭，非零值必须不超过该轴的正力矩上限 |
| `friction_velocity_deadbands` | 死区内补偿为 0；机械臂单位 rad/s，夹爪单位 m/s，必须大于 0 |
| `friction_velocity_scales` | 从死区边缘到满幅补偿的速度宽度，单位同上，必须大于 0 |
| `friction_ramp_duration` | 从零增长到满幅的最短时间（秒），必须大于 0；降低助力不受此限制 |

当前机械臂速度不超过 0.02 rad/s 时无摩擦助力，0.02～0.07 rad/s 平滑增加，达到 0.07 rad/s 后目标为设定幅值；满幅还需满足 0.2 秒渐增限制。计算为 `x=clamp((abs(dq)-deadband)/scale, 0, 1)`、`tau_f=sign(dq)*amplitude*x*x*(3-2*x)`，然后限制幅值增长速度。减速时及时减小补偿，回到死区立即清零，跨方向时先清零一周期再向新方向渐增。控制周期无效或超过 0.1 秒时撤掉摩擦项并重置渐增状态；异常关节状态走原有零力/失能处理。

最终命令为 `clamp(sign_g*scale_g*G(q) - damping*dq + tau_f, ±torque_limit)`（外部力反馈默认关闭）。连续模式和旧脉冲模式不能同时开启。该实现无需力传感器，但只能补偿已经发生滑动时的部分阻力，不能在静止时判断人的用力方向，也不补偿惯性。

调试时可靠托住机械臂，逐轴比较同一姿态、相近速度下的正反向拖动。先测试 J1，再测试 J3；每次只改对应的 `friction_compensations`，可按 0.05 N·m 小步调整并重启。J4～J6 和夹爪初始为零。若松手后出现自行持续运动、加速或抖动，停止测试并把该轴幅值恢复为零；提高死区不能替代修正重力误差。

J2 的下坠问题尚未解决，因此本次 J2 补偿幅值保持为零。速度补偿无法区分手动拖动与重力下坠，过补偿可能加重下坠；先修正 J2 的重力/零点/质量模型，再单独试调摩擦项。所有初始幅值都不是实测辨识结果。停用整个功能可设 `friction_compensation_enabled: false` 并重启。

摩擦建模背景可参考 [MathWorks 的摩擦模型说明](https://www.mathworks.com/help/simscape/ref/translationalfriction.html)；本项目采用上面的死区和平滑过渡函数，未实现该文档中的完整 Stribeck 模型。

TCP 为两个手指 STL 在 `link6` 下合并包围盒的中心：`xyz=[0.135300001892, 0.00000415, 0] m`，轴向继承 `link6`。详见 [模型与实物标定](docs/标定与验证/S0_CALIBRATION.md)，包括重力方向、ID/方向/零点、夹爪间距和 TCP 实测记录。

## 目录与文档

| 路径 | 内容 |
| --- | --- |
| `src/A5/` | `a5_description` 模型包：URDF、九份 STL、模型约定 |
| `src/a6_y_controllers/` | 重力控制器、launch、参数、控制器测试 |
| `src/a6_y_ros2_control/` | 达妙硬件插件、SLCAN、协议与诊断工具 |
| `scripts/environment.bash` | ROS 环境加载与本项目包来源检查 |
| `tools/` | 模型、几何、标定和完整启动验证 |
| `docs/标定与验证/S0_CALIBRATION.md` | 模型定义与实测标定操作 |
| `docs/标定与验证/VALIDATION.md` | 验证命令、报告与验收边界 |
| `validation/` | 软件验证报告 |

常见问题：串口打不开时检查设备名、访问权限及是否被其他进程占用；找不到包时确认构建成功并执行 `a6y_load_install`；模型或插件找不到时检查是否混入其他工作区；Pinocchio mimic 报错时检查版本是否满足 4.0 要求。
