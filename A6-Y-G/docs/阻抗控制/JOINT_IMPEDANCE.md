# 固定关节角度阻抗控制

> 历史设计与实验记录：本页保留早期算法、参数与验证过程，正文中的频率、参数快照和本机路径不作为当前运行入口。当前原生控制器配置为 **125 Hz**，以 [关节阻抗 YAML](../../src/a6_y_controllers/config/a5_joint_impedance_controllers.yaml) 为准。当前功能、接口与边界见 [控制器 README](../../src/a6_y_controllers/README.md)，构建及测试入口见 [底层开发与测试](../how-to/develop-and-test.md)。

`a6_y_controllers/A6YJointImpedanceController` 保持激活后首次有效反馈中的六个关节角度。手推关节时允许偏移，松手后产生回到原角度的恢复力矩。重新启动会重新捕获目标。夹爪开度参与整机质量、重力和惯性计算，夹爪命令始终为零力矩，不保持开度。

这与笛卡尔阻抗的目标不同：这里的六组刚度、阻尼对应 joint1～joint6，不是 X/Y/Z 和三个旋转方向。末端手感随姿态及各关节参数变化。当前版本只支持启动时捕获的固定目标，没有运行中轨迹或目标话题。默认配置还启用有上限的有速度门槛的位置积分补偿，以改善静摩擦造成的回位残差；因此准确说是“关节阻抗 + 有界积分回位补偿”。

## 启动与参数

在 `~/Code/A6-Y-G` 使用已有环境和通信：

```bash
source scripts/environment.bash
export MAKEFLAGS='-j2'
colcon build --base-paths src --packages-select a6_y_controllers \
  --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
a6y_load_install

# 不打开串口，也不使能电机；仅验证 ROS 接口和启动。
ros2 launch a6_y_controllers a5_joint_impedance.launch.py \
  dry_run:=true auto_enable:=false use_rviz:=true
```

实机测试前停止原来的重力或笛卡尔阻抗 launch，托住机械臂并放到希望保持的姿态。三种控制器占用相同的 28 个命令接口，不能同时激活或启动多个进程访问同一串口。

```bash
ros2 launch a6_y_controllers a5_joint_impedance.launch.py \
  dry_run:=false auto_enable:=true can_channel:=/dev/ttyACM0 use_rviz:=true
```

`Captured fixed joint target` 表示已捕获六轴角度；启动弹簧刚度在 2 秒内渐增，重力立即给出。先在目标附近小幅推动一个关节再释放。停止前托住机械臂；Ctrl+C 会撤去补偿并失能。

任务配置是 `src/a6_y_controllers/config/a5_joint_impedance_controllers.yaml`：

```yaml
# 顺序：joint1 joint2 joint3 joint4 joint5 joint6
joint_stiffness: [24.0, 20.0, 24.0, 3.0, 4.5, 1.2]
joint_damping: [6.0, 4.5, 3.6, 0.8, 0.8, 0.12]
spring_error_limits: [0.2, 0.12, 0.1, 0.12, 0.22, 0.1]
joint_integral_gains: [4.0, 4.0, 8.0, 0.8, 1.2, 1.0]
integral_torque_limits: [1.5, 2.2, 3.0, 0.4, 0.4, 0.35]
integral_deadband: 0.005
integral_error_window: 0.35
integral_velocity_threshold: 0.15
impedance_torque_limits: [2.4, 2.8, 3.6, 0.6, 0.6, 0.6]
```

刚度单位 N·m/rad，阻尼 N·m·s/rad，积分增益 N·m/(rad·s)，力矩上限 N·m。死区 0.005 rad 约 0.29°。这是按下述离线工况试调的参数，不是实机标定值。本次增强目标附近的即时弹簧作用并降低阻尼，同时增加弹簧输入偏差上限以约束大角度释放。J3/J5/J6 的积分增益另作配合调整，以减少小偏差下等待静摩擦补偿的时间；积分额度和最终力矩上限保持上一版数值。关节 2 看第二项，腕部看第四至第六项；不能与笛卡尔刚度直接比较。修改后需重启控制器，运行中修改 ROS 参数不会刷新缓存。

启动入口从 `gravity_parameters_file`（默认原 `a5_gravity_controllers.yaml`）导入 `torque_scales`、`effort_signs`、`damping_gains`、`torque_limits`。这里的总阻尼是 `joint_damping` 加导入的六轴 `damping_gains`，不要把两个数组重复设为同一组大值。重力系数仅作用于重力项，电机坐标方向由原驱动映射；本模式不叠加沿运动方向助力的摩擦补偿。

自定义任务配置可用 `controllers_file:=绝对路径`，自定义重力文件用 `gravity_parameters_file:=绝对路径`。继续使用原 SLCAN、50 Hz、MIT 力矩接口，硬件 MIT Kp/Kd 为零。

## 回不到位时怎样调整

先检查日志中的 `error_deg`、`dq_rad_s`、`impedance_nm`、`integral_nm` 和 `saturated`。2026-10-07 的实机反馈确认了两类问题：完全松手后，J3 仍偏约 17.7° 且附加指令达到原 1.8 N·m 上限；J6 误差已经变为 +23.17°，旧积分仍为 −0.15 N·m，最终附加指令为 −0.071 N·m。后者来自旧版“超过约 20° 就冻结积分”的规则，不能用提高 K 单独解决。

- 本版取消大偏差冻结。`integral_error_window` 仅截断积分输入：用 `clamp(error, -window, +window)` 控制累积速率，目标仍是启动时捕获的角度。
- 死区之外，误差与旧积分方向相反时，在本周期求解之前立即清零旧积分，包含快速推动时的换向。这样不会再让反向旧补偿把机械臂推离目标。重新累积仍受速度门槛和限幅约束。
- 弹簧启动渐增完成、误差超过死区、速度不高于 0.15 rad/s、输出未向同一方向饱和时，才正常累积。死区内保留平衡偏置；快速运动且没有换向时保留记忆，但不继续累积。
- 前三轴附加力矩上限由 `[1.8, 1.8, 1.8]` 调到 `[2.4, 2.8, 3.6] N·m`；六轴补偿额度见上述配置。导入的总力矩上限、重力系数和电机坐标映射继续沿用原配置。增大的额度需要实机复测，并非已辨识的摩擦值或持续额定力矩。
- `joint_stiffness` 影响即时弹簧作用，`joint_damping` 影响回弹；`joint_integral_gains` 影响停滞后的补偿速度。当前实现有采样、惯性耦合和限幅，手感不能直接按 YAML 的 K 推算。
- 需要纯 PD 关节阻抗时，将 `joint_integral_gains` 六项全部设为零并重启。积分开启时，持续按住偏离位置会增强恢复作用，手感不是严格固定刚度的线性弹簧。

目标重捕获、停用、故障时清零积分。任何参数修改均需重启控制器。

## 控制公式与限幅

连续时间的基本思路为 `tau = K(q_target - q) - D*dq + G(q)`。实现采用实际周期 `h` 和 Pinocchio 的六轴惯性子矩阵进行隐式一步计算，考虑夹爪当前开度但不控制其加速度：

```text
e = q_target - q
I_memory = previous_integral
if abs(e) > deadband and e*I_memory < 0: I_memory = 0  # 逐轴
I = clamp(I_memory, lower, upper)
e_spring = clamp(e, -spring_error_limits, +spring_error_limits) # 逐轴，值为0则不截断
v_next = dq + h * inverse(M) * tau_pd
tau_pd = clamp(K*(e_spring - h*v_next) - D*v_next, lower-I, upper-I)
tau_impedance = I + tau_pd
command = gravity_clamped + tau_impedance
I_next = clamp(I + h*Ki*clamp(e, -window, window), -integral_limit, integral_limit)
# 仅在上述累积条件满足时；反向时先清零旧补偿，死区内保持记忆
```

`spring_error_limits` 只限制送入弹簧求解的偏差，不移动捕获目标，不限制实际运动范围，也不冻结积分。大偏差下恢复力不再按完整误差继续增长；进入设定范围后恢复较强的局部弹簧作用。由于仍经过隐式求解，`K` 不是实测静态刚度，实际输出还与姿态、阻尼、周期和限幅有关。

`K` 包含启动渐增系数；`D` 是上述总阻尼。各轴 `lower/upper` 同时满足附加力矩上限和重力补偿之后的总力矩余量。积分项是缓慢建立的静态平衡偏置，先预留其力矩额度，再用剩余额度求解弹簧/阻尼；移位后的上下界仍包含零。隐式预测只用于 PD 增量，假设积分项补偿静态负载；实际负载与估计不匹配时仍会产生加速度，因此该设计不是速度限制器。六个关节是有界旋转关节，误差直接按测得角度相减，不跨机械限位做 ±π 环绕。

令 `A=D+h*K`，将此问题化为六变量带上下界的严格凸二次优化：`H=h*inverse(M)+inverse(A)`，`b=inverse(A)*K*e_spring-dq`，最小化 `0.5*tau_pdᵀ*H*tau_pd-bᵀ*tau_pd`，边界为上述 `lower-I/upper-I`。零刚度且零阻尼的轴固定 PD 增量为零；配置校验禁止零刚度轴启用积分。求解器使用定长矩阵和最多 96 次活动集迭代；无解/数值异常/未收敛时不输出中间结果，控制器锁定失能。

力矩约束参与求解，不使用输出历史变化率限制，也不使用“最紧腕关节决定整组共同缩放系数”。惯性耦合仍存在，不代表各关节物理上解耦。真实重力误差、静摩擦、饱和及采样会影响回位精度和手感；模型计算不等于实机稳定性证明。

## 状态日志与故障行为

每两秒输出 `Joint impedance status`：

- `error_deg`：目标角度减实测角度，按 joint1～joint6 排列。
- `dq_rad_s`：六轴速度。
- `impedance_nm`：实际求解出的六轴附加阻抗力矩指令，不含重力，不是传感器实测力矩。
- `integral_nm`：当前使用的积分回位力矩，已包含在 `impedance_nm` 内。
- `saturated`：各轴是否因上下界限制而偏离其隐式弹簧阻尼请求；1 表示受限。
- `period_ms` / `solver_iterations`：当前更新周期和求解迭代次数。

任何一个关节（包括夹爪）的反馈无效/过旧、周期不在 `(0, max_control_period]`、接口错序或数值求解失败，都会清零力矩并写 `motor_enable=0`，故障锁定直到重新激活。原驱动的反馈检查与延迟使能握手继续生效。这个软件行为不能替代硬件急停或机械限位。

## 离线验证

```bash
source scripts/environment.bash
a6y_load_install
export ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$PWD/validation/ros_logs"
ctest --test-dir build/a6_y_controllers --output-on-failure
/usr/bin/python3 tools/verify_joint_impedance_dry_run.py
/usr/bin/python3 -s tools/verify_joint_impedance_return.py
```

求解器测试检查独立关节限幅、耦合饱和阻尼的耗能及上下界一致性。控制器测试检查目标捕获/重捕获、实时重力和夹爪模型、启动渐增、力矩余量、反向阻尼、参数与反馈故障。动力学测试调用真实控制器，用独立八坐标 URDF 模型、固定手指、理想重力抵消、0.05 N·m·s/rad 物理黏性摩擦和 1 ms 积分，测试六个关节分别推开后释放，以及控制周期/反馈延迟变化。

dry-run 脚本固定 `dry_run=true`、`auto_enable=false` 和不存在的串口，核对插件、参数继承、目标捕获、关节/TF 发布及所有电机未使能；它本身不是动力学仿真。验证结果记录在 `validation/joint_impedance_check.json`、`validation/joint_impedance_dry_run_check.json`。实机调参另行验证。

### 含静摩擦的回位仿真

`tools/verify_joint_impedance_return.py` 调用真实 C++ 控制器测试程序，读取当前关节 YAML，完全使用模拟接口，不加载串口驱动。结果输出到 `validation/joint_spring_strengthening/`：`results.json`、`trajectories.csv`、`comparison.png` 和完整日志。

最初软参数（不是此次修复前版本）的负对照在早期日志姿态下推开 J1/J5 0.15 rad 后，残差分别约 8.59°/6.73°。新配置检查 108 个选定工况：两种姿态、六个关节、20/40 ms 周期、零或一个周期反馈延迟；包含无摩擦、假设库仑/静摩擦、±30% 惯量偏差、重力残余力矩及反馈/指令量化。每次释放后运行 8 秒；要求被推开关节在 3 秒内进入初始误差的 50% 范围并持续保持，另外检查第 6～8 秒各轴最大误差小于 2°、最大速度小于 0.03 rad/s（含量化工况为 0.04 rad/s），以及全过程指令限幅和小幅释放峰值速度低于 2 rad/s。覆盖的是测试表中的组合，不是所有因素的笛卡尔积。

物理模型以 1 ms 步长积分，包含完整六轴耦合惯量和科氏项，固定手指；用速度空间的非光滑摩擦求解模拟静止保持和滑动摩擦。基准摩擦假设为 `[0.55, 0.25, 0.85, 0.08, 0.10, 0.05] N·m`，测试中取 0、0.5、1、1.5 倍。J1/J3 的量级参考已有重力模式补偿，其他项为敏感性分析假设，均不是实机辨识值。重力按理想抵消加常值残差处理。该仿真不包含碰撞、串口丢帧、全部电机内环和机械柔性，也不证明硬件速度上限或任意工况稳定性；dry-run 仅验证 ROS 启动，实机未由此脚本运行。

### 大幅、连续反向推动的补充验证

同一验证脚本还运行 84 次大幅释放：使用本次实机捕获的目标姿态，六个轴分别推动以及六轴同时推动，每组先在反向偏差处保持 5 秒，再从约 14～47° 的偏差释放 20 秒，然后反方向重复。每组含零或一个 20 ms 周期的反馈延迟。新增阻力假设为 `[1.0, 1.6, 2.2, 0.25, 0.25, 0.20] N·m`，测试 0、1、1.25 倍；其中无摩擦工况也不添加物理黏性阻尼。

这些较大阻力用于检验“输出到原限幅仍停住”的失效模式，不能据此区分实机的静摩擦、重力误差和传动误差。测试要求被推开的各关节在 3 秒内进入初始误差的 50% 范围并持续保持，全过程峰值速度低于 3.5 rad/s；还检查释放后第 16～20 秒误差小于 2°、速度小于 0.03 rad/s，并记录进入 2° 误差带的耗时。3 秒内“明显回位”不等于所有工况在 3 秒内收敛到 2°。速度峰值取 1 ms 积分子步日志，曲线采样为控制周期；这些回归阈值不是实机速度保证。原来的 108 个小幅工况仍检查第 6～8 秒，两个窗口不能混为一谈。

`large_push_recovery.png` 给出双向回位曲线，`large_push_trajectories.csv` 保存完整轨迹，`results.json` 的 `repeated_pushes` 包含全部大幅工况及耗时。另有回归测试直接复现并检查旧积分换向时首周期清零、快速运动不累积、死区保留平衡偏置及参数校验。没有启动实机验证，动力学模型也不含机械碰撞与完整电机内环。

### 2026-10-07 手感排查与增强弹簧

实机日志显示上一版参数确已加载，所记录周期为 19.98～20.03 ms；23 个状态样本只有 J1/J2 各一次限幅，J3 没有限幅。用户确认 J3 基本没有直接扶住或推动；该轴从约 0.94° 到 0.63° 的缓慢变化持续约 40 秒，积分从 0.012 增至 2.451 N·m。其他关节仍被推动，因此这不是孤立的无外力试验，不能据此认定具体摩擦或重力误差。

使用原 C++ 求解器回放记录的角度、速度和积分，在假设夹爪开度 −0.015 m 时，与日志附加指令最大差约 0.0045 N·m。静态探针确认：上一版阻尼不仅抑制速度，也通过隐式预测削弱即时弹簧力；J1/J5/J6 的局部指令刚度约为 YAML K 的 17%/15%/4.7%。本次同时调整 K 与 D，并用 spring_error_limits 抑制大角度恢复请求。原重力模式的沿运动方向摩擦助力仍没有在阻抗模式启用；机械摩擦尚未辨识。

新加入四个 5° 释放工况，覆盖 J1/J3/J5/J6，使用本次记录的目标姿态与较大阻力假设 `[1,1.6,2.2,0.25,0.25,0.2] N·m`、20 ms 周期、无额外反馈延迟、零初始积分。检查 3 秒时偏差至少减半，并记录移动超过 0.001 rad 的开始时间。旧版 J5/J6 分别约 4.1/5.7 秒才开始移动，暴露了原测试将小偏差与较小摩擦配对的覆盖缺口；新增工况的摩擦仍是假设值，不是实机测量值。原 108 个小幅与 84 次大幅释放检查继续保留。
