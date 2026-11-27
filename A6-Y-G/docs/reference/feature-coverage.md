# 功能覆盖索引

本页用于查找七类功能的源码、启动器、配置与测试位置。命令默认从仓库根目录执行；链接到的模块文档注明自己的工作目录。当前模型为 A5，每臂六轴加一个夹爪电机，第二手指是 `mimic` 关节。

[项目首页](../../../README.md) · [工程入口](../../README.md) · [文档索引](../README.md)

## 1. 通信与标定

| 项目 | 位置与用途 |
| --- | --- |
| 使用说明 | [诊断 README](../../tools/arm_diagnostics/README.md)：Qt、CLI、可选 Web、标零事务和报告 |
| 启动器 | [run_arm_scan.bash](../../scripts/run_arm_scan.bash) |
| 实现 | [arm_diagnostics](../../tools/arm_diagnostics)：身份、扫描、反馈、串口占用与零位操作 |
| 设备绑定 | [arm_roles.yaml](../../../diagnostics/arm_roles.yaml)；换板见[迁移指南](../how-to/replace-usb-can-adapter.md) |
| 电机配置 | [motor_position_limits.yaml](../../tools/arm_diagnostics/motor_position_limits.yaml) |
| 底层工具 | [驱动 tools](../../src/a6_y_ros2_control/tools)：总线扫描、ID、反馈、读写寄存器、Flash 保存 |
| 离线检查 | `bash A6-Y-G/scripts/run_arm_scan.bash --cli --arms all --dry-run`；模块内 `test_*.py` |

只读扫描与显式零位／参数写入分开。电机参数工具的操作条件见[开发指南](../how-to/develop-and-test.md)。

## 2. 模型核对

| 项目 | 位置与用途 |
| --- | --- |
| 使用说明 | [模型 README](../../tools/arm_visualization/README.md)：预览、实时、无头显示与失联处理 |
| 启动器 | [run_arm_rviz.bash](../../scripts/run_arm_rviz.bash) |
| 实现 | [arm_visualization](../../tools/arm_visualization)：反馈到关节状态、角度和夹爪位移换算 |
| 机械模型 | [src/A5](../../src/A5)：URDF 与网格 |
| 换算配置 | [joint_mapping.yaml](../../tools/arm_visualization/joint_mapping.yaml) |
| 离线检查 | `bash A6-Y-G/scripts/run_arm_rviz.bash --cli --arm motion_left --dry-run`；[test_visualization.py](../../tools/arm_visualization/test_visualization.py) |

模型角度、电机角度、单指位移、总开口是不同的量；夹爪当前示教端点另见示教配置。

## 3. MoveIt 控制

| 项目 | 位置与用途 |
| --- | --- |
| 使用说明 | [MoveIt README](../../tools/arm_moveit/README.md)：模拟／实机、目标、规划、执行、曲线与退出 |
| 启动与构建 | [run_arm_moveit.bash](../../scripts/run_arm_moveit.bash) · [build_moveit_stack.bash](../../scripts/build_moveit_stack.bash) · [build_moveit_panel.bash](../../scripts/build_moveit_panel.bash) |
| 桌面工具 | [arm_moveit](../../tools/arm_moveit)：原生会话、规划、轨迹与遥测 |
| ROS 实现 | [独立 ros2_stack](../../tools/arm_moveit/ros2_stack)：MoveIt、控制面板与驱动 |
| 配置 | [碰撞策略](../../tools/arm_moveit/collision_policy.yaml) · [夹爪限制](../../tools/arm_moveit/gripper_limits.yaml) |
| 验证 | 模块内 `test_*.py`、`verify_*.py` 与独立驱动测试；具体依赖见模块 README |

独立 MoveIt 栈不能与主工程的同名驱动一起构建。旧 Python 后端记录保留在[归档说明](../../tools/arm_moveit/README_python_backend_archived.md)，不是当前默认路径。

## 4. 独立重力补偿

| 项目 | 位置与用途 |
| --- | --- |
| 使用说明 | [重力 README](../../tools/arm_gravity/README.md)：单／多臂 MIT、在线调参、profile 与退出 |
| 启动与构建 | [run_arm_gravity.bash](../../scripts/run_arm_gravity.bash) · [build_gravity_stack.bash](../../scripts/build_gravity_stack.bash) |
| 桌面实现 | [arm_gravity](../../tools/arm_gravity) 中的重力会话、界面与回零模块 |
| 原生控制器 | [a6_y_controllers](../../src/a6_y_controllers)：重力模型、补偿与控制接口 |
| 配置 | [a5_gravity_controllers.yaml](../../src/a6_y_controllers/config/a5_gravity_controllers.yaml)；各角色 profile 见模块 README |
| 验证 | [test_arm_gravity.bash](../../scripts/test_arm_gravity.bash) 和模块单测；集成测试依赖 ROS、模型与 GUI 环境 |

独立入口控制六轴，J7 只读；它与下述示教入口共享部分基础设施，但控制行为和频率不同。

## 5. 动力示教与夹爪反馈

| 项目 | 位置与用途 |
| --- | --- |
| 操作指南 | [运行动力示教](../how-to/run-power-teaching.md) · [日志检查](../how-to/inspect-teaching-logs.md) |
| 机制与参数 | [原理说明](../explanation/power-teaching-and-gripper-feedback.md) · [运行配置](runtime-and-configuration.md) |
| 启动器 | [run_arm_teaching.bash](../../scripts/run_arm_teaching.bash) |
| 桌面实现 | [arm_gravity](../../tools/arm_gravity) 中的 `teaching_*.py`：配对、会话、参数、频率与响应记录 |
| ROS 实现 | [a6_y_teleop](../../src/a6_y_teleop) · [a6_y_teleop_msgs](../../src/a6_y_teleop_msgs) · [原生控制器](../../src/a6_y_controllers) |
| 夹爪配置 | [teaching_gripper_limits.yaml](../../tools/arm_gravity/teaching_gripper_limits.yaml) |
| 验证 | [test_arm_teaching.bash](../../scripts/test_arm_teaching.bash) · [复现指南](../how-to/run-offline-tests.md) |

覆盖单对、双对、交叉配对、百分比夹爪跟随、接触保持与示教阻力。夹爪反馈上限是电机轴总力矩，当前并非指尖力控制。

## 6. VR 遥操作

| 项目 | 位置与用途 |
| --- | --- |
| 统一入口说明 | [VR README](../VR遥操作/README.md)：虚拟／位置／阻抗、网关、接管、停止、录制与回放 |
| 虚拟模型 | [run_vr_rviz.bash](../../scripts/run_vr_rviz.bash) |
| 实机入口 | [run_vr_position.bash](../../scripts/run_vr_position.bash) · [run_vr_position_full.bash](../../scripts/run_vr_position_full.bash) · [run_vr_real.bash](../../scripts/run_vr_real.bash) |
| 网关与状态控制 | [run_vr_gateway.bash](../../scripts/run_vr_gateway.bash) · [create_vr_certificate.bash](../../scripts/create_vr_certificate.bash) · [vr_position_control.bash](../../scripts/vr_position_control.bash) |
| ROS 实现 | [a6_y_vr_teleop](../../src/a6_y_vr_teleop)：输入协议、映射、IK、状态机与桥接 |
| 网关实现 | [vr_gateway](../../vr_gateway)：WebXR、TLS、输入与录制工具 |
| 配置 | [VR 配置目录](../../src/a6_y_vr_teleop/config) 与[控制器配置目录](../../src/a6_y_controllers/config)中的 `a5_vr_*` |
| 验证 | [VR 单测](../../src/a6_y_vr_teleop/test)、mock 与 commission 检查；入口见 VR README |

VR 控制单条选定机械臂，默认串口必须由操作者核对；四臂角色绑定与示教夹爪反馈不会自动应用到这些入口。

## 7. 底层开发

| 项目 | 位置与用途 |
| --- | --- |
| 开发指南 | [构建、选择模块测试与硬件实验](../how-to/develop-and-test.md) |
| 控制器参考 | [控制器 README](../../src/a6_y_controllers/README.md)：重力、关节／笛卡尔阻抗、VR 位置与接口 |
| 驱动 | [a6_y_ros2_control](../../src/a6_y_ros2_control)：串口 CAN 协议、反馈与命令检查 |
| 工具与实验 | [scripts](../../scripts) 的 `run_j*_position_test.bash`、`run_j*_highspeed_test.bash`；[tools](../../tools) 中对应执行与分析工具 |
| 电机专题 | [ID 设置](../电机调试/MOTOR_ID_SETUP.md) · [电机参数](../电机调试/MOTOR_PARAMETERS.md) |
| 模型标定 | [模型、TCP 与实物标定](../标定与验证/S0_CALIBRATION.md) · [软件验证与实物验收](../标定与验证/VALIDATION.md) |
| 验证层次 | Python 算法／工具单测、原生 CTest、PTY 虚拟电机、GUI 与 ROS 离线集成；现场验收另行进行 |

单关节实验可能同时保持其余五轴；具体使能范围、写参数和 Flash 行为必须依据对应脚本与开发指南。带日期的调参记录是历史资料，当前参数来自配置文件。
