# A6-Y-G 工程入口

本目录包含 A6Y 四臂系统的 ROS 2 包、桌面工具、Quest 网关与开发脚本。当前机械模型来自 **A5 CAD**：J1～J6 为旋转关节，J7 驱动夹爪，另一手指通过 URDF `mimic` 联动。运动左／右与示教左／右是四种设备角色。

[项目首页](../README.md) · [功能覆盖索引](docs/reference/feature-coverage.md) · [首次离线运行](docs/tutorials/first-offline-run.md) · [完整文档索引](docs/README.md)

## 功能与启动入口

以下命令从**仓库根目录**执行。图形入口打开后，按对应文档选择模拟或实机模式；一个串口同一时间由一个入口占用。

| 用途 | 命令 | 功能说明 |
| --- | --- | --- |
| 通信与标定 | `bash A6-Y-G/scripts/run_arm_scan.bash` | [USB 身份、七电机扫描、反馈与显式标零](tools/arm_diagnostics/README.md) |
| 模型核对 | `bash A6-Y-G/scripts/run_arm_rviz.bash` | [只读反馈、角度／夹爪换算与 RViz](tools/arm_visualization/README.md) |
| MoveIt 控制 | `bash A6-Y-G/scripts/run_arm_moveit.bash` | [目标预览、规划、执行、夹爪和曲线](tools/arm_moveit/README.md) |
| 重力补偿 | `bash A6-Y-G/scripts/run_arm_gravity.bash` | [单／多臂 MIT、在线调参与参数保存](tools/arm_gravity/README.md) |
| 动力示教 | `bash A6-Y-G/scripts/run_arm_teaching.bash` | [单／双对、交叉配对、夹爪反馈与日志](docs/how-to/run-power-teaching.md) |
| VR 虚拟模型 | `bash A6-Y-G/scripts/run_vr_rviz.bash` | [Quest 网关、虚拟与实机路线](docs/VR遥操作/README.md) |
| 底层开发 | 按所选控制器或测试分别启动 | [控制器参考](src/a6_y_controllers/README.md) · [开发与测试](docs/how-to/develop-and-test.md) |

VR 还包含 `run_vr_position.bash`、`run_vr_position_full.bash`、`run_vr_real.bash`、网关和本地状态控制工具。它们使用单臂稳定串口路径及独立配置，完整流程见 [VR README](docs/VR遥操作/README.md)。

## 各控制入口的边界

| 入口 | 控制对象与默认行为 |
| --- | --- |
| 扫描／模型核对 | 只读查询；扫描工具另有显式零位写入流程 |
| MoveIt | 默认模拟模式；实机可规划并执行六轴与夹爪运动 |
| 独立重力补偿 | J1～J6 MIT，默认 125 Hz；J7 只读 |
| 动力示教 | 默认离线模拟；实机示教六轴 MIT，示教 J7 轻阻尼与接触反馈，运动七电机 POS_VEL；底层 250 Hz |
| VR 位置／阻抗 | 普通入口默认 readonly；完整位置试运行入口启动为 DISABLED，须 PREPARE／ARM；不复用示教夹爪反馈 |
| 单关节实验 | 按实验设计控制；部分脚本会接管六轴保持其他关节 |

示教夹爪按各臂内收行程百分比跟随，接触后保持运动夹爪位置，并向示教 J7 反馈有限阻力。示教启动不回零，平滑接入绝对关节角跟随；正常退出六轴回零、夹爪保持实测开度，随后全部失能。原理与参数见[示教说明](docs/explanation/power-teaching-and-gripper-feedback.md)及[运行配置](docs/reference/runtime-and-configuration.md)。

## 包与构建边界

| 路径 | 职责 |
| --- | --- |
| [src/A5](src/A5) | `a5_description`：A5 URDF、网格与关节模型 |
| [src/a6_y_ros2_control](src/a6_y_ros2_control) | 主工程驱动、协议、电机寄存器／ID／状态工具 |
| [src/a6_y_controllers](src/a6_y_controllers) | 重力补偿、关节／笛卡尔阻抗、VR 位置与阻抗控制器 |
| [src/a6_y_teleop](src/a6_y_teleop) | 六轴映射、夹爪接触检测与反馈节点 |
| [src/a6_y_teleop_msgs](src/a6_y_teleop_msgs) | 示教消息与服务 |
| [src/a6_y_vr_teleop](src/a6_y_vr_teleop) | VR 输入、IK、状态机与实机桥接 |
| [vr_gateway](vr_gateway) | HTTPS / WebXR 输入、录制与回放 |
| [tools](tools) | 桌面工具、实验、分析与验证工具 |
| [tools/arm_moveit/ros2_stack](tools/arm_moveit/ros2_stack) | 独立 MoveIt 配置、面板与移植驱动 |

示教和重力启动器调用 `scripts/build_gravity_stack.bash`，构建到 `/tmp/a6y-gravity-build-<uid>`，并核验实际加载的包与 Python 跟随脚本。MoveIt 使用自己的构建器。**两套栈都含 `a6_y_ros2_control`，不能合并构建。** VR 和控制器开发使用主工程 `src` 构建，具体命令见[开发指南](docs/how-to/develop-and-test.md)。

`scripts/environment.bash` 加载 ROS 2 Humble，清理旧 overlay、Conda 与用户 Python 路径，使用系统工具链。只在专用终端中加载。软件依赖见[首次运行教程](docs/tutorials/first-offline-run.md)。

## 配置与运行数据

四臂身份位于相邻 `diagnostics/arm_roles.yaml`；保留 `A6-Y-G/` 与 `diagnostics/` 的层级关系。模型映射、重力 profile、示教夹爪端点、MoveIt 和 VR 限制分别由各模块配置管理，不能把一处参数视作所有入口的全局设置。路径映射见[功能覆盖索引](docs/reference/feature-coverage.md)。

运行会在本地生成报告、曲线、录制和验证输出。仓库保留必要运行配置与既有参考资料，原始运行日志、私钥和参数恢复副本不随发布上传。已有截图与带日期的实验报告仅描述对应版本。

## 开发与验证

先选模块再运行其离线检查。以下命令不连接实体串口：

```bash
bash A6-Y-G/scripts/run_arm_scan.bash --cli --arms all --dry-run
bash A6-Y-G/scripts/run_arm_rviz.bash --cli --arm motion_left --dry-run
bash A6-Y-G/scripts/run_arm_teaching.bash --check
```

需要修改驱动、控制器或 VR 时，按[开发与测试指南](docs/how-to/develop-and-test.md)构建并选择模块单测、CTest 或 mock 验证；示教完整回归见[离线测试指南](docs/how-to/run-offline-tests.md)。测试命令与硬件实验分开列出，离线通过不等于新设备已完成实机验收。
