# 文档导航

文档按用途放入中文分类文件夹，原文件名保持不变。启动和使用优先阅读“使用说明”；方案、参考分析和带日期的实测报告用于查阅设计依据与历史记录。

| 分类 | 文档 |
| --- | --- |
| 电机调试 | [电机 ID 设置](电机调试/MOTOR_ID_SETUP.md) · [电机参数编辑](电机调试/MOTOR_PARAMETERS.md) · [J1～J4 当前参数与 Flash 保存](电机调试/J1_J4_FLASH_SAVE_20261010.md) · [J1 低速调参](电机调试/J1_POSITION_TUNING_20261009.md) · [J1 90°/s 调参](电机调试/J1_HIGH_SPEED_TUNING_20261009.md) · [J2 低速调参](电机调试/J2_POSITION_TUNING_20261009.md) · [J2 90°/s 调参](电机调试/J2_HIGH_SPEED_TUNING_20261009.md) · [J3 低速调参](电机调试/J3_POSITION_TUNING_20261009.md) · [J3 90°/s 调参](电机调试/J3_HIGH_SPEED_TUNING_20261009.md) · [J4 高速调参（进行中）](电机调试/J4_HIGH_SPEED_TUNING_20261009.md) |
| 阻抗控制 | [末端位姿阻抗控制](阻抗控制/CARTESIAN_IMPEDANCE.md) · [关节阻抗控制](阻抗控制/JOINT_IMPEDANCE.md) |
| 标定与验证 | [模型、TCP 与实物标定](标定与验证/S0_CALIBRATION.md) · [软件验证与实物验收](标定与验证/VALIDATION.md) |
| VR遥操作／虚拟机械臂 | [使用说明](VR遥操作/虚拟机械臂/VR_RVIZ_USAGE.md) · [接入方案](VR遥操作/虚拟机械臂/VR_RVIZ_PLAN.md) |
| VR遥操作／实机控制 | [使用说明](VR遥操作/实机控制/VR_REAL_ROBOT_USAGE.md) · [接入方案](VR遥操作/实机控制/VR_REAL_ROBOT_PLAN.md) |
| VR遥操作／参考方案 | [Meta to Piper 源码分析](VR遥操作/参考方案/VR_REFERENCE_ANALYSIS.md) · [Piper 与 A5 控制对比](VR遥操作/参考方案/VR_PIPER_CONTROL_COMPARISON.md) |
| 图片资源 | [A5 TCP 几何示意图](图片资源/a5_tcp_geometry.png) |

文档中的终端命令默认在项目根目录 `/home/alce/Code/A6-Y-G` 执行。完整项目介绍、环境准备和构建命令见 [项目 README](../README.md)。实测数据仍保存在项目根目录的 `validation/`。
