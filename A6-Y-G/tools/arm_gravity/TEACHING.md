# 动力示教操作入口

本页用于启动并找到当前示教操作文档。命令从仓库根目录执行：

```bash
# 配置检查，不连接硬件；首次会构建依赖
bash A6-Y-G/scripts/run_arm_teaching.bash --check

# 打开窗口，默认为“离线模拟”
bash A6-Y-G/scripts/run_arm_teaching.bash
```

首次使用先完成[离线教程](../../docs/tutorials/first-offline-run.md)。操作实体设备时，按[动力示教指南](../../docs/how-to/run-power-teaching.md)核对 USB、配对和反馈，再在窗口选择“真机动力示教”并手动启动。

当前支持 1 对 1、2 对 2 及交叉配对。示教六轴 MIT 重力补偿，示教 J7 零刚度、轻阻尼与接触反馈；运动七轴 POS_VEL，夹爪按各臂内收端点的 0～100% 映射。接触后捕获位置目标，张开释放；运动端采用位置保持。示教 J7 的 2 N·m 是反馈与阻尼总上限，空载阻尼上限为 0.015 N·m。

启动不回零，六轴跟随绝对角度。点击“停止示教 · 回零并失能”后，参与臂的 J1～J6 回零、参与臂的夹爪保持实测开度，最后七轴失能并释放夹持力。需要跳过回零时先托住机械臂和夹持物，再用“已托住全部 · 立即失能”。

| 要完成的工作 | 文档 |
| --- | --- |
| 真机配对、跟随和退出 | [运行动力示教](../../docs/how-to/run-power-teaching.md) |
| 核对接触触发、跟随和故障 | [查看日志](../../docs/how-to/inspect-teaching-logs.md) |
| 更换同一臂的 USB2CAN | [身份与配置迁移](../../docs/how-to/replace-usb-can-adapter.md) |
| 检查离线软件链路 | [离线测试](../../docs/how-to/run-offline-tests.md) |
| 查询频率、单位、端点及力矩配置 | [运行与配置参考](../../docs/reference/runtime-and-configuration.md) |
| 理解百分比映射、接触保持和反馈 | [原理说明](../../docs/explanation/power-teaching-and-gripper-feedback.md) |

本套设备的端点保存在 [teaching_gripper_limits.yaml](teaching_gripper_limits.yaml)，原始采样和现场日志留在本地。修改后需关闭旧窗口并重新打开；独立重力补偿入口仍维持六轴控制、夹爪只读。
