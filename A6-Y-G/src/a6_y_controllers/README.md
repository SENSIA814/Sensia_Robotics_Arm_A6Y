# A5 重力、阻抗与位置控制器

本 ROS 2 包使用完整 A5 URDF、Pinocchio mimic 模型及主工程达妙驱动，实现重力补偿、固定关节阻抗、固定末端阻抗和 VR／单关节实验的位置插件。夹爪及联动第二手指的开度、质量参与模型。

[工程入口](../../README.md) · [构建与底层测试](../../docs/how-to/develop-and-test.md) · [动力示教](../../tools/arm_gravity/TEACHING.md) · [VR 实机入口](../../docs/VR遥操作/实机控制/VR_REAL_ROBOT_USAGE.md)

## 插件与入口

| 插件 | 功能 | 配置／入口 |
| --- | --- | --- |
| `A6YGravityCompController` | 实时重力、阻尼、可选摩擦补偿及受门控触觉力矩 | [`a5_gravity_comp.launch.py`](launch/a5_gravity_comp.launch.py)、[`a5_gravity_controllers.yaml`](config/a5_gravity_controllers.yaml) |
| `A6YJointImpedanceController` | 捕获六轴关节角，隐式弹簧阻尼及有界积分回位 | [`a5_joint_impedance.launch.py`](launch/a5_joint_impedance.launch.py)、[`a5_joint_impedance_controllers.yaml`](config/a5_joint_impedance_controllers.yaml) |
| `A6YCartesianImpedanceController` | 捕获 `tool_tcp` 位姿，按基坐标轴方向保持位置与朝向 | [`a5_cartesian_impedance.launch.py`](launch/a5_cartesian_impedance.launch.py)、[`a5_impedance_controllers.yaml`](config/a5_impedance_controllers.yaml) |
| `A6YVRPositionController` | VR 状态机门控的原生位置目标；单关节实验也复用此插件 | [`a5_vr_position_full_controllers.yaml`](config/a5_vr_position_full_controllers.yaml)、VR 入口 |

插件全名带 `a6_y_controllers/` 前缀，见 [`A6_y_controllers_plugin.xml`](A6_y_controllers_plugin.xml)。VR 位置插件与关节阻抗共用源码。控制器竞争相同命令接口，不能同时激活或由多个进程打开同一串口。

当前三份原生重力／阻抗 YAML 的 `controller_manager.update_rate` 均为 **125 Hz**。动力示教另生成 **250 Hz** 配置，MoveIt 独立栈为 **100 Hz** JTC；不要把历史说明中的 50 Hz 当作当前默认值。

## 无硬件启动

按[开发指南](../../docs/how-to/develop-and-test.md)构建本地 `A6-Y-G/install` 后，在 `A6-Y-G/` 下执行，每次只启动一种模式：

```bash
source scripts/environment.bash
a6y_load_install
ros2 launch a6_y_controllers a5_joint_impedance.launch.py \
  dry_run:=true auto_enable:=false use_rviz:=true
```

停止后可用同样参数改为 `a5_cartesian_impedance.launch.py` 或 `a5_gravity_comp.launch.py`。三个入口默认 `dry_run=true`、`auto_enable=false`、`use_rviz=false`。dry_run 不访问 CAN，只核对接口和启动，不验证手推回弹或实机稳定性。

真机部署需显式设置 `dry_run=false`、`auto_enable=true` 和核对过的 `can_channel`。底层 launch 默认 `/dev/ttyACM0`，没有四臂桌面选择器的角色确认；实际应传完整 by-id 路径及该臂的 `motor_config_file`，不能把默认串口编号当作臂身份。

## 固定关节阻抗

激活后首次有效反馈捕获六轴目标，六组参数对应 J1～J6。默认运行不跟踪轨迹，重新激活会重捕获；VR 接入是另行配置的模式。

| 参数 | 当前 YAML | 单位／作用 |
| --- | --- | --- |
| `joint_stiffness` | `[54, 50, 54, 45, 38.5, 5.2]` | N·m/rad |
| `joint_damping` | `[6, 4.5, 3.6, 1.8, 1.8, 0.52]` | N·m·s/rad，另加导入的关节阻尼 |
| `spring_error_limits` | `[0.20, 0.12, 0.10, 0.12, 0.22, 0.10]` | rad；限制弹簧输入，不是机械行程 |
| `joint_integral_gains` | `[4, 4, 8, 0.8, 1.2, 1]` | N·m/(rad·s)，全零关闭积分 |
| `integral_torque_limits` | `[1.5, 2.2, 3, 0.4, 0.4, 0.35]` | N·m，积分占附加预算 |
| `impedance_torque_limits` | `[2.4, 2.8, 3.6, 0.6, 0.6, 0.6]` | N·m，包含积分与阻尼 |

求解器按惯性矩阵和实际周期计算隐式弹簧／阻尼，将逐轴边界放入优化。积分受死区、速度与饱和限制；误差和旧积分反向时先清零旧偏置。`integral_error_window` 限制积分输入，不重置固定目标。持续按住可能增强恢复作用，默认行为不是纯线性弹簧。

日志包括 `error_deg`、`dq_rad_s`、`impedance_nm`、`integral_nm`、`saturated` 和周期／迭代次数。推导及历史工况保留于 [`JOINT_IMPEDANCE.md`](../../docs/阻抗控制/JOINT_IMPEDANCE.md)，当前运行值以 YAML 为准。

## 固定末端阻抗

首次有效反馈捕获 `tool_tcp` 位置和朝向；误差经雅可比映射成关节任务力矩，结合惯性与周期做隐式预测，再叠加实时重力。

| 参数 | 当前 YAML | 单位／作用 |
| --- | --- | --- |
| `cartesian_stiffness` | `[230, 230, 270, 18, 18, 18]` | XYZ 为 N/m，旋转为 N·m/rad |
| `cartesian_damping` | `[28, 28, 28, 0.6, 0.6, 0.6]` | XYZ 为 N·s/m，旋转为 N·m·s/rad |
| `wrench_limits` | `[25, 25, 25, 0.75, 0.75, 0.75]` | 前三项 N，后三项 N·m；任务指令上限 |
| `impedance_torque_limits` | `[1.8, 1.8, 1.8, 0.6, 0.6, 0.6]` | N·m，附加关节力矩上限 |

共同缩放同时满足任务、附加关节力矩和重力后的总力矩余量。长期 `task_scale < 1` 时不能仅靠增加刚度解决偏移。该模式是位姿弹性保持，没有恒力控制或避障。日志包含 `xyz_error_mm`、`rotation_error_deg`、`task_scale`、`limit` 和任务力分量；它们不是标定后的末端测力。

算法背景、原始论文来源及历史工况保留于 [`CARTESIAN_IMPEDANCE.md`](../../docs/阻抗控制/CARTESIAN_IMPEDANCE.md)，历史数值不覆盖当前配置。

## 共享参数、夹爪与停止

阻抗 launch 从 `gravity_parameters_file` 导入 `torque_scales`、`effort_signs`、`damping_gains`、`torque_limits`，任务参数来自 `controllers_file`。默认引用本包 YAML，不自动加载四臂桌面的角色 profile。参数在 configure 读取，修改后重启。

两种阻抗弹簧项默认在 2 秒内渐增，重力立即给出，不叠加沿运动方向助力的摩擦补偿。夹爪反馈参与模型，但固定阻抗 J7 输出为零，不保持开度。动力示教通过生成配置专门接入 J7 触觉通道，与此行为不同。

反馈无效／过旧、周期异常或求解失败会清零力矩并请求 `motor_enable=0`，故障锁定后需重新激活。停止前支撑机械臂；底层阻抗 launch 停止撤去支撑，**不执行桌面示教的回零流程**。这不是碰撞检测、硬件急停或任意工况稳定性保证。

## 开发、测试与来源

源码在 [`src/`](src/)，头文件与有界求解器在 [`include/`](include/)，测试在 [`test/`](test/)。[开发与测试指南](../../docs/how-to/develop-and-test.md)覆盖全部 CTest、两种阻抗 dry_run、模型回位和单关节实验。软件模型或 PTY 通过不能替代实机摩擦、手感和时延验证；此次文档更新没有驱动实体设备。

依赖、作者及许可声明保留于 [`package.xml`](package.xml)，源码延续 A6-Y robot team 与各文件原声明。CMake 要求 Pinocchio ≥ 4.0，系统 Python 绑定也需支持当前 mimic 模型。
