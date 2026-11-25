# 固定末端位姿阻抗控制

第一版实现末端位置、姿态的弹性保持：手推时允许偏移，松手后朝启动时的目标位姿恢复。控制点为夹爪中心 `tool_tcp`；坐标轴为 `base_link` 轴向。目标在控制器激活后的首次有效反馈中捕获，运行期间保持固定；停止后重新启动会捕获新的目标。

独立插件 `a6_y_controllers/A6YCartesianImpedanceController` 与原重力控制器共用 A6-Y-G 的硬件驱动、七台电机反馈、URDF、Pinocchio mimic 模型和 MIT 力矩指令通道。两者占用相同命令接口，不能同时激活。通信方式、50 Hz 控制频率、电机方向、零点、夹爪传动比例均沿用当前工程。

## 控制行为

2026-10-07 修正了实机反馈“左右摆”后发现的两个软件问题：输出力矩变化率限制会让阻尼在速度换向后继续助推；原显式弹簧阻尼计算在 50 Hz 下的模型动态测试也不能收敛。现在使用惯性矩阵及当前控制周期进行隐式一步预测，取消输出历史限速，保留启动刚度渐增和输出幅值限制。

```text
e = [目标位置 - 当前位置; log(R_target * R_current^T)]
Kq = J^T K J
Dq = J^T D J + B
(M + h Dq + h² Kq) a = J^T K e - (Dq + h Kq) dq
v_predict = dq + h a
wrench = K (e - h J v_predict) - D J v_predict
tau_task = J^T wrench - B v_predict
输出 = 限幅后的实时重力补偿 + scale * tau_task
```

`h` 是当前控制周期；`M` 为 Pinocchio 计算的六轴惯性子矩阵，夹爪开度视为已测得的固定坐标；`J` 为 TCP 原点处、以基坐标系轴向表达的六轴雅可比；`B` 为导入的关节阻尼。`K` 包含启动渐增系数。姿态使用最短旋转误差，其时间预测为目标附近的局部近似。

这种基于下一步状态的 PD 计算思路可参见 [Tan、Liu、Turk 的原始论文](https://faculty.cc.gatech.edu/~turk/my_papers/stable_pd.pdf)。本项目具体公式为上述任务空间局部线性化，论文的仿真结果不能作为本机械臂实机稳定性的保证。

每周期重新计算重力，夹爪及联动第二手指的质量和当前开度都参与模型；夹爪输出保持零力。启动时弹簧项在 2 秒内渐增，重力补偿立即完整给出。用一个共同的 `scale` 缩放整组任务力矩，满足末端各分量限幅、各轴附加力矩上限及重力之外的剩余总力矩空间；避免逐关节削波改变耦合阻尼的方向。`scale < 1` 表示限幅正在削弱任务输出。

`impedance_torque_rates` 已废弃；旧自定义 YAML 仍可加载，但会警告该参数被忽略。继续调大这个参数或刚度不能代替离散控制修正。

该模式仍是位置、姿态的弹性保持，没有恒定接触力控制或避障。持续外力、重力模型误差、摩擦和限幅都可能造成稳态位姿偏差。精确相差 180° 时仍存在旋转轴选择的几何歧义。

## 构建和无硬件验证

在项目根目录执行：

```bash
source scripts/environment.bash
export MAKEFLAGS='-j2 -l2'
colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
a6y_load_install
export ROS_LOG_DIR="$PWD/validation/ros_logs"
export ROS_DOMAIN_ID=193
export ROS_LOCALHOST_ONLY=1
ctest --test-dir build/a6_y_controllers --output-on-failure
/usr/bin/python3 tools/verify_impedance_dry_run.py
```

查看无硬件模型：

```bash
ros2 launch a6_y_controllers a5_cartesian_impedance.launch.py \
  dry_run:=true auto_enable:=false use_rviz:=true
```

dry-run 不连接串口，只回填状态；它可验证插件加载、目标捕获、重力参数继承及使能状态，不能模拟手推回弹或验证闭环稳定性。控制器测试另行通过连续时间极限下的势能有限差分检查恢复力，通过数值状态检查阻尼反向与饱和时的耗能、旋转跨界、目标重捕获、力矩限制和故障失能，并用独立八坐标动力学模型进行释放后收敛测试。模型测试假定理想重力补偿，不等于实机验收。

## 实机使用

先停止原重力补偿 launch 和所有占用同一串口的工具。确认目标姿态周围有运动空间并支撑机械臂，在希望保持的姿态启动：

```bash
cd ~/Code/A6-Y-G
source scripts/environment.bash
a6y_load_install
ros2 launch a6_y_controllers a5_cartesian_impedance.launch.py \
  dry_run:=false auto_enable:=true can_channel:=/dev/ttyACM0 use_rviz:=true
```

日志出现 `Captured fixed TCP target` 表示已捕获当前位置和朝向。初次验证从小幅、缓慢推动开始，分别观察位置和姿态的回位；不要以大幅推离或接触硬限位来测试。停止前托住机械臂，Ctrl+C 会撤去补偿并失能。需要更换保持目标时，停止、调整到新姿态后重新启动。

## 参数

任务参数文件：`src/a6_y_controllers/config/a5_impedance_controllers.yaml`。下表记录 2026-10-07 修复时保留的用户参数；本次修正没有继续提高刚度或输出上限，仍需实机小幅复测。后续数值以 YAML 为准。

| 参数 | 当前 YAML | 含义 |
| --- | --- | --- |
| `cartesian_stiffness` | `[100, 100, 100, 12, 12, 12]` | XYZ 为 N/m，旋转为 N·m/rad；增大后恢复作用增强 |
| `cartesian_damping` | `[18, 18, 12, 0.63, 0.63, 0.63]` | 平移 N·s/m，旋转 N·m·s/rad |
| `wrench_limits` | `[5, 5, 5, 0.5, 0.5, 0.5]` | 六个末端分量的指令上限；不是经过标定的实测接触力限制 |
| `impedance_torque_limits` | `[0.8, 0.8, 0.8, 0.3, 0.3, 0.3]` | 各轴附加阻抗力矩上限，N·m |
| `impedance_ramp_duration` | `2.0` | 弹簧项渐增时间，s |
| `max_control_period` | `0.1` | 超过此控制周期则清零命令并请求失能，s |
| `max_feedback_age` | `0.1` | 位置或反馈过旧则清零命令并请求失能，s |

启动入口自动从原 `a5_gravity_controllers.yaml` 导入 `torque_scales`、`effort_signs`、`damping_gains` 和 `torque_limits`，确保使用当前已调好的重力参数；如使用自定义重力文件，传入 `gravity_parameters_file:=绝对路径`。任务文件可用 `controllers_file:=绝对路径` 替换。这四个重力参数以导入文件为准。

阻抗模式不叠加原先沿运动方向助力的摩擦补偿，以免把手推和回弹运动同时放大。原重力模式和其摩擦参数没有改变。重力补偿效果已由用户反馈良好，本次软件验证不重新宣称完成了实物模型或力矩限值标定。

参数在 configure 时读取，修改后需重启。优先观察回位是否平稳，再小步调整相应方向刚度；不要直接照搬参考机械臂的高刚度数值。速度反馈、真实通信周期和机械摩擦决定最终可用参数。

反馈无效、过期、控制周期异常或计算结果非有限值时，控制器写入零力矩及 `motor_enable=0`，并锁定故障，需停用后重新激活；原驱动的反馈校验和失能流程继续生效。这不是硬件急停、碰撞检测或全行程安全保证。

## 观察运行日志

新版 configure 时会打印 `Implicit TCP impedance`，用于确认已加载修正后的二进制。随后每两秒打印一次 `Impedance status`：

- `xyz_error_mm`：相对启动目标的三个位置误差，mm。
- `rotation_error_deg`：朝向误差角度。
- `joint_speed_norm`：六轴关节速度向量的范数，rad/s。
- `task_scale`：共同限幅系数；长期远小于 1 时，单独增加刚度通常不能解决偏移。
- `period_ms`：该次控制更新的实际周期。
- `limit`：决定共同缩放系数的最严格限制。`wrench:Z` 表示末端 Z 力，`wrench:Rx/Ry/Rz` 表示旋转力矩，`joint_task:jointN` 表示附加关节力矩，`joint_total:jointN` 表示重力补偿后该方向的剩余总力矩空间；`none:none` 表示未缩小任务输出。多个限制并列时显示遍历中最先遇到的一个。
- `limit_requested_abs` / `limit_available`：对应限制的请求绝对值与可用幅值；平移分量单位为 N，其余为 N·m。这里的请求值在共同缩放之前。
- `task_Fz_requested_N` / `task_Fz_scaled_N`：隐式预测算出的 Z 任务力指令，以及共同缩放后的值；不包含重力补偿，也不是传感器测得的末端作用力。关节阻尼仍单独计入任务关节力矩。

排查 Z 保持时，在同一目标姿态下区分完全松手后的静态偏差和手推过程中的偏移。`xyz_error_mm` 为目标减实测，因此 Z 正值表示当前位置低于目标；只有确认机械臂基坐标系 Z 轴竖直向上时，才直接对应物理下沉。日志每两秒采样，不能据此排除采样间的快速摆动。2026-10-07 增加这些诊断字段时没有修改刚度、阻尼、力矩上限或控制公式。

若仍出现明显往复摆动，托住后停用，并保留这些日志以及同期关节反馈；不要继续单独增加刚度或输出上限。已有软件模型测试不能覆盖真实电机响应、减速器摩擦、未测得的传输延迟和模型误差。
