# 文档索引

[项目首页](../../README.md) · [工程入口](../README.md)

当前动力示教文档按读者任务组织。命令默认从仓库根目录执行；较早的模块文档可能以 `A6-Y-G/` 为工作目录，请以该页说明为准。

## 教程：第一次运行

[首次离线运行](tutorials/first-offline-run.md)从依赖核验开始，带你完成配置检查、扫描 dry-run 和离线示教界面。

## 操作指南：完成一项任务

| 任务 | 指南 |
| --- | --- |
| 配对、真机启动、正常退出与立即失能 | [运行动力示教](how-to/run-power-teaching.md) |
| 检查跟随与接触记录，导出诊断包 | [检查示教日志](how-to/inspect-teaching-logs.md) |
| 核验新 USB-CAN 并迁移绑定 | [更换适配器](how-to/replace-usb-can-adapter.md) |
| 修改代码后执行软件回归 | [运行离线测试](how-to/run-offline-tests.md) |

## 参数参考：查入口、文件与单位

[运行与配置](reference/runtime-and-configuration.md)汇总环境、角色、控制参数、配置文件和日志字段。设备相关的夹爪端点保存在[运行 YAML](../tools/arm_gravity/teaching_gripper_limits.yaml)，原始标定采样保留在本地。

## 原理说明：理解控制行为

[动力示教与夹爪反馈](explanation/power-teaching-and-gripper-feedback.md)解释绝对关节角跟随、夹爪百分比映射、接触保持、阻力回传与释放条件。

## 独立工具与控制入口

| 模块 | 文档 |
| --- | --- |
| 电机诊断与零位 | [七电机扫描、USB 身份与标零](../tools/arm_diagnostics/README.md) |
| 只读模型 | [实时反馈、坐标换算与 RViz](../tools/arm_visualization/README.md) |
| 独立重力补偿 | [多臂调参、参数保存与停止](../tools/arm_gravity/README.md) |
| MoveIt | [轨迹规划与执行、夹爪控制](../tools/arm_moveit/README.md) |
| 控制器开发 | [控制器说明](../src/a6_y_controllers/README.md) |
| 截图 | [截图来源与状态](images/README.md) |
| 历史实验数据 | [diagnostics 数据范围](../../diagnostics/README.md) |

<a id="vr"></a>

## VR、标定与历史专题

以下为已有模块说明与专题资料。带日期的调参报告仅描述对应版本；方案文档用于理解设计，不替代当前入口的操作说明。它们保留原目录和链接，避免已有引用失效。

| 分类 | 文档 |
| --- | --- |
| 电机调试 | [电机 ID 设置](电机调试/MOTOR_ID_SETUP.md) · [电机参数编辑](电机调试/MOTOR_PARAMETERS.md) · [J1～J4 当前参数与 Flash 保存](电机调试/J1_J4_FLASH_SAVE_20261010.md) · [J1 低速调参](电机调试/J1_POSITION_TUNING_20261009.md) · [J1 90°/s 调参](电机调试/J1_HIGH_SPEED_TUNING_20261009.md) · [J2 低速调参](电机调试/J2_POSITION_TUNING_20261009.md) · [J2 90°/s 调参](电机调试/J2_HIGH_SPEED_TUNING_20261009.md) · [J3 低速调参](电机调试/J3_POSITION_TUNING_20261009.md) · [J3 90°/s 调参](电机调试/J3_HIGH_SPEED_TUNING_20261009.md) · [J4 高速调参（进行中）](电机调试/J4_HIGH_SPEED_TUNING_20261009.md) |
| 阻抗控制 | [末端位姿阻抗控制](阻抗控制/CARTESIAN_IMPEDANCE.md) · [关节阻抗控制](阻抗控制/JOINT_IMPEDANCE.md) |
| 标定与验证 | [模型、TCP 与实物标定](标定与验证/S0_CALIBRATION.md) · [软件验证与实物验收](标定与验证/VALIDATION.md) |
| VR遥操作／虚拟机械臂 | [使用说明](VR遥操作/虚拟机械臂/VR_RVIZ_USAGE.md) · [接入方案](VR遥操作/虚拟机械臂/VR_RVIZ_PLAN.md) |
| VR遥操作／实机控制 | [使用说明](VR遥操作/实机控制/VR_REAL_ROBOT_USAGE.md) · [接入方案](VR遥操作/实机控制/VR_REAL_ROBOT_PLAN.md) |
| VR遥操作／参考方案 | [Meta to Piper 源码分析](VR遥操作/参考方案/VR_REFERENCE_ANALYSIS.md) · [Piper 与 A5 控制对比](VR遥操作/参考方案/VR_PIPER_CONTROL_COMPARISON.md) |
| 图片资源 | [A5 TCP 几何示意图](图片资源/a5_tcp_geometry.png) |

实测运行日志、下载包和原始标定采样不随本次发布上传。当前示教参数以[运行与配置](reference/runtime-and-configuration.md)及其链接的源码为准。
