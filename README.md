# SENSIA · A6Y Robotics

**四臂动力示教，让关节动作与夹爪开合一起跟随。**

基于 ROS 2 Humble 的机械臂控制与遥操作工程。示教端提供六轴重力补偿，运动端跟随关节角与夹爪开度；抓到物体后，将夹爪接触阻力反馈到示教端。支持单对或双对配对，并提供诊断、MoveIt 规划与独立 VR 入口。

[快速启动](#quick-start) · [示教原理](#teaching) · [文档导航](#docs) · [验证范围](#validation)

![动力示教实际程序界面：两组配对、离线模拟、日志与停止控制；当前未连接机械臂](A6-Y-G/docs/images/teaching-current.png)

*当前发布源码的 PyQt5 界面，2026-11-27 离屏截取。窗口处于离线空闲状态，未连接硬件、未填充演示反馈；截图展示界面功能。*

## 一套工程，四种机械臂角色

默认配对为 **示教左 → 运动左、示教右 → 运动右**，也可在界面选择单对或交叉配对。每臂有 6 个旋转关节和 1 个夹爪电机；第二手指由 URDF `mimic` 联动。当前使用 A5 CAD 模型，工程目录和 ROS 包保留 A6Y 命名。

| 能力 | 当前行为 | 入口 |
| --- | --- | --- |
| 动力示教 | 六轴重力补偿、绝对关节角跟随、启动平滑接入 | [开始示教](A6-Y-G/docs/how-to/run-power-teaching.md) |
| 夹爪跟随与接触反馈 | 按各臂行程百分比跟随；接触后保持，示教夹爪反馈阻力 | [工作原理](A6-Y-G/docs/explanation/power-teaching-and-gripper-feedback.md) |
| 四臂诊断 | USB 身份核验、七电机扫描、反馈与零位工具 | [诊断工具](A6-Y-G/tools/arm_diagnostics/README.md) |
| 只读模型检查 | 电机／模型坐标换算与 RViz 姿态核对 | [模型检查](A6-Y-G/tools/arm_visualization/README.md) |
| 独立重力补偿 | 单臂或多臂调参、按臂保存；此入口 J7 只读 | [重力补偿](A6-Y-G/tools/arm_gravity/README.md) |
| MoveIt 规划 | XYZ / RPY 目标、轨迹规划与执行、夹爪控制 | [MoveIt](A6-Y-G/tools/arm_moveit/README.md) |
| VR 遥操作 | Quest 虚拟模型，以及独立实机位置／阻抗入口 | [VR 文档](A6-Y-G/docs/README.md#vr) |

<a id="quick-start"></a>

## 快速启动：先看到离线界面

软件基线为 **Ubuntu 22.04 · ROS 2 Humble · 系统 Python 3.10 · Pinocchio ≥ 4.0**，桌面工具使用 PyQt5。依赖准备见[首次离线运行教程](A6-Y-G/docs/tutorials/first-offline-run.md)。下面先获取代码，进入仓库后执行离线检查与启动：

```bash
git clone https://github.com/SENSIA814/Sensia_Robotics_Arm_A6Y.git
cd Sensia_Robotics_Arm_A6Y

# 不连接实体串口：核对角色配置与模拟反馈
bash A6-Y-G/scripts/run_arm_scan.bash --cli --arms motion_left teaching_right --dry-run

# 不连接硬件：核对模型、身份、端点与速度参数
bash A6-Y-G/scripts/run_arm_teaching.bash --check

# 打开窗口，默认选择“离线模拟”
bash A6-Y-G/scripts/run_arm_teaching.bash
```

首次示教检查会自动构建所需 ROS 包。出现“检查通过；没有连接硬件”后，即可打开窗口了解配对与日志界面。准备操作实体机械臂时，按[真机示教指南](A6-Y-G/docs/how-to/run-power-teaching.md)核对串口、标定与停止流程。

仓库内的 USB 身份和夹爪端点对应当前这套设备。更换机械臂或适配器时，先按[身份与配置迁移指南](A6-Y-G/docs/how-to/replace-usb-can-adapter.md)更新绑定并验证。

<a id="teaching"></a>

## 从动作跟随到夹爪接触反馈

![两组独立示教回路：示教六轴重力补偿和J7触觉，向运动臂发送关节位置及夹爪开度，运动夹爪的J7接触阻力反向回传](assets/readme/teaching-loop.svg)

示教臂 J1～J6 以 MIT 模式提供重力补偿，运动臂七个电机以 `POS_VEL` 模式跟随。六轴目标使用绝对关节角；夹爪将示教开度归一化到 **0～100%**，再映射到配对运动夹爪自己的可用行程。每端保留按模型单指换算的 **0.7 mm** 余量。

空载时，示教夹爪采用零刚度与轻微速度阻尼。运动夹爪检测到接触后捕获保持位置，继续捏合示教夹爪会产生反馈阻力，张开则释放。当前示教 J7 的**总力矩上限为 2 N·m**，包含阻尼与接触反馈，并非持续输出值。

启动不回零：先接管当前姿态，再平滑进入跟随。正常退出时，六轴回零、夹爪保持当时开度，随后全部失能；立即失能会跳过回零并释放力矩支撑与夹持力。

详细机制见[动力示教与夹爪反馈原理](A6-Y-G/docs/explanation/power-teaching-and-gripper-feedback.md)，参数和单位见[运行配置参考](A6-Y-G/docs/reference/runtime-and-configuration.md)。

<a id="docs"></a>

## 按任务找文档

| 你现在要做什么 | 文档 |
| --- | --- |
| 第一次安装并打开离线界面 | **教程** · [首次离线运行](A6-Y-G/docs/tutorials/first-offline-run.md) |
| 操作实体设备，开始或停止示教 | **操作指南** · [运行动力示教](A6-Y-G/docs/how-to/run-power-teaching.md) |
| 查看跟随滞后、接触反馈或导出日志 | **操作指南** · [检查示教日志](A6-Y-G/docs/how-to/inspect-teaching-logs.md) |
| 更换 USB-CAN，核对设备身份 | **操作指南** · [更换适配器](A6-Y-G/docs/how-to/replace-usb-can-adapter.md) |
| 修改代码后运行离线回归 | **操作指南** · [运行离线测试](A6-Y-G/docs/how-to/run-offline-tests.md) |
| 查询控制频率、力矩单位、配置文件 | **参数参考** · [运行与配置](A6-Y-G/docs/reference/runtime-and-configuration.md) |
| 理解百分比映射、接触保持和阻力回传 | **原理说明** · [动力示教与夹爪反馈](A6-Y-G/docs/explanation/power-teaching-and-gripper-feedback.md) |

更多模型、阻抗、VR 和历史实验资料见[完整文档索引](A6-Y-G/docs/README.md)。

<a id="validation"></a>

## 验证范围

当前发布副本已通过 **201 项 Python 单元测试、12 项 CTest 检查和 7 组离线集成验证**。覆盖四臂跟随、夹爪接触保持与阻力回传、反馈故障、模式切换失败、退出重试、回零中失能，以及界面刷新和日志导出；复现入口见[离线测试指南](A6-Y-G/docs/how-to/run-offline-tests.md)。

现场操作者已确认当前这套设备完成试用。发布检查使用配置核验、PTY 虚拟电机和离线界面，没有驱动实体机械臂。使用时需区分以下边界：

- **100 Hz 目标发送、250 Hz 底层控制是配置频率**，不能直接换算成端到端机械时延。
- 接触反馈基于电机力矩与运动状态估计；N·m 表示电机轴力矩，尚未标定为指尖夹持力。
- 运动夹爪接触后采用位置保持；当前实现不提供恒力夹持，也未建立六轴力反馈回路。
- 张开动作会优先撤销夹爪阻力，开向微动可能使反馈手感间歇变化。软物体、摩擦与不同机构需要分别验证。
- 旧调参记录中的速度、增益和测试结论仅对应当时版本。当前值以[运行配置](A6-Y-G/docs/reference/runtime-and-configuration.md)为准。

## 项目结构

```text
Sensia_Robotics_Arm_A6Y/
├── A6-Y-G/
│   ├── src/          # A5 模型、驱动、控制器、示教与 VR 节点
│   ├── tools/        # 诊断、只读模型、MoveIt、重力与示教桌面工具
│   ├── scripts/      # 环境隔离、构建、启动与测试入口
│   ├── docs/         # 教程、操作指南、参数参考与原理说明
│   └── vr_gateway/   # Quest WebXR 网关
├── diagnostics/      # 角色绑定与已有参考资料；运行日志在本地生成
└── assets/readme/    # 可编辑的 README 技术图
```

保留 `A6-Y-G/` 与 `diagnostics/` 相邻。MoveIt 独立栈与主工程包含不同实现的同名驱动包，应使用各自启动器，避免混入同一次构建。开发入口见[工程说明](A6-Y-G/README.md)。

## 来源与许可

工程保留 A6-Y-G 源码与 A5 模型，并包含提供给本项目的 MoveIt 移植代码；对应[上游文件清单](A6-Y-G/tools/arm_moveit/ros2_stack/upstream_manifest.json)和[适配差异](A6-Y-G/tools/arm_moveit/ros2_stack/upstream_changes.patch)可供查阅。

各 ROS 包保留作者与许可证声明。仓库目前没有统一的根目录 `LICENSE`；使用和分发前请查阅具体组件的许可。
