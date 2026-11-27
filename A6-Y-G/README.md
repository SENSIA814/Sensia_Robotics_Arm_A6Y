# A6-Y-G 工程入口

本目录包含 A6Y 四臂系统的 ROS 2 包、桌面工具与启动脚本。当前机械模型来自 A5 CAD：J1～J6 为旋转关节，J7 驱动夹爪，另一手指通过 URDF `mimic` 联动。

[项目首页](../README.md) · [首次离线运行](docs/tutorials/first-offline-run.md) · [运行配置参考](docs/reference/runtime-and-configuration.md) · [完整文档索引](docs/README.md)

## 启动入口

以下命令从**仓库根目录**执行。各图形入口的模式和接管流程见对应说明。

| 用途 | 命令 | 文档 |
| --- | --- | --- |
| 诊断界面 | `bash A6-Y-G/scripts/run_arm_scan.bash` | [扫描、身份与标零](tools/arm_diagnostics/README.md) |
| 只读模型 | `bash A6-Y-G/scripts/run_arm_rviz.bash` | [反馈与 RViz](tools/arm_visualization/README.md) |
| 动力示教 | `bash A6-Y-G/scripts/run_arm_teaching.bash` | [配对、启动与退出](docs/how-to/run-power-teaching.md) |
| 独立重力补偿 | `bash A6-Y-G/scripts/run_arm_gravity.bash` | [在线调参](tools/arm_gravity/README.md) |
| MoveIt 规划 | `bash A6-Y-G/scripts/run_arm_moveit.bash` | [规划与执行](tools/arm_moveit/README.md) |
| VR 虚拟模型 | `bash A6-Y-G/scripts/run_vr_rviz.bash` | [虚拟模型使用说明](docs/VR遥操作/虚拟机械臂/VR_RVIZ_USAGE.md) |

真机 VR 的位置与阻抗入口独立配置，见[VR 实机文档](docs/VR遥操作/实机控制/VR_REAL_ROBOT_USAGE.md)。

## 当前动力示教行为

示教 J1～J6 使用 MIT 重力补偿；示教 J7 使用零刚度、轻阻尼及接触反馈。运动七电机使用 `POS_VEL`，六轴跟随绝对关节角，夹爪按各臂内收行程的百分比跟随。接触后保持运动夹爪位置，并将有限阻力反馈到示教 J7。

启动不回零，平滑接入跟随；正常退出六轴回零、夹爪保持实测开度，随后全部失能。独立重力补偿入口仍为六轴控制、J7 只读。完整机制与边界见[原理说明](docs/explanation/power-teaching-and-gripper-feedback.md)。

## 包与构建边界

| 路径 | 职责 |
| --- | --- |
| `src/A5/` | `a5_description`：URDF、网格与关节模型 |
| `src/a6_y_ros2_control/` | 主工程驱动、协议和电机工具 |
| `src/a6_y_controllers/` | 重力补偿、关节／笛卡尔阻抗与 VR 控制器 |
| `src/a6_y_teleop/` | 六轴与夹爪映射、夹爪接触反馈 |
| `src/a6_y_teleop_msgs/` | 示教消息与服务 |
| `src/a6_y_vr_teleop/` | VR 输入、IK 与桥接 |
| `tools/arm_moveit/ros2_stack/` | 独立 MoveIt 配置与移植驱动 |

示教和重力启动器会调用 `scripts/build_gravity_stack.bash`，构建依赖到 `/tmp/a6y-gravity-build-<uid>`，并核验实际加载的包与 Python 跟随脚本。MoveIt 使用自己的构建器。两套栈均含名为 `a6_y_ros2_control` 的包，不要合并构建。

`scripts/environment.bash` 加载 ROS 2 Humble，清理旧 overlay、Conda 与用户 Python 路径，使用系统工具链。只在专用终端中加载；依赖与版本要求见[首次运行教程](docs/tutorials/first-offline-run.md)。

## 开发与验证

先运行配置检查，再运行对应的离线测试。动力示教完整入口：

```bash
bash A6-Y-G/scripts/run_arm_teaching.bash --check
bash A6-Y-G/scripts/test_arm_teaching.bash
```

测试范围、诊断测试命令及失败定位见[离线测试指南](docs/how-to/run-offline-tests.md)。修改控制逻辑时同步注明角色、模式、单位与验证范围；参数页中的默认值以当前源码和配置为准。

既有[关节阻抗](docs/阻抗控制/JOINT_IMPEDANCE.md)、[笛卡尔阻抗](docs/阻抗控制/CARTESIAN_IMPEDANCE.md)、[标定与验证](docs/标定与验证/S0_CALIBRATION.md)文档保留独立入口。带日期的调参与实验资料属于历史记录，不作为当前动力示教的参数来源。
