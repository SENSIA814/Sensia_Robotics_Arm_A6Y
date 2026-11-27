# 更换同一机械臂的 USB2CAN

本指南用于**机械臂、电机和零位均未改变，只更换 USB2CAN**的情况。更换机械臂、电机、传动或重新标零时，不能沿用本流程假定原夹爪端点仍有效。命令从仓库根目录执行。

## 1. 确认新板及其所属角色

退出所有占用该臂的控制与诊断程序，确认机械臂有支撑、电机失能。先列出设备：

```bash
ls -l /dev/serial/by-id/
```

记录新板的完整 by-id 路径和 USB 序列号，再核对它的 CAN 线确实接向待替换角色。`ttyACM` 编号会随插拔改变，不能用于判断左右臂。没有串口或设备处于 DFU 模式时，先解决 USB／固件运行模式问题；修改绑定不能恢复未枚举的设备。

## 2. 使用临时角色文件只读检查

保留正式配置，制作临时副本：

```bash
cp diagnostics/arm_roles.yaml /tmp/a6y-replacement-roles.yaml
```

编辑 `/tmp/a6y-replacement-roles.yaml` 中对应角色的 `usb_serial` 和 `by_id`，其余角色不变。以下以 `teaching_right` 为例，实际替换其他臂时改角色名：

```bash
bash A6-Y-G/scripts/run_arm_scan.bash --cli \
  --roles /tmp/a6y-replacement-roles.yaml \
  --arms teaching_right --passes 5
```

需确认 7 台电机都有反馈且处于失能状态。扫描只查询状态；更换序列号后暂时缺少 PMAX 缓存时，不要把尚未核验量程的角度当成标定结果。

## 3. 读取并核对电机身份与协议量程

将 `A6Y_ADAPTER` 改为上一步已经确认的新板完整路径。此工具只发送状态和寄存器读取，不使能、不写零位、不写 Flash。

```bash
A6Y_ADAPTER='/dev/serial/by-id/替换为已核对的新板完整名称'
A6Y_READBACK_DIR=$(mktemp -d /tmp/a6y-adapter-readback.XXXXXX)
for A6Y_JOINT in joint1 joint2 joint3 joint4 joint5 joint6 gripper_finger_joint1; do
  /usr/bin/python3 A6-Y-G/tools/read_motor_position_params.py \
    --channel "$A6Y_ADAPTER" --joint "$A6Y_JOINT" \
    --config A6-Y-G/src/a6_y_controllers/config/a5_motors.yaml \
    --output "$A6Y_READBACK_DIR/$A6Y_JOINT.json" || break
done
```

检查七份报告的 `complete`、`errors`、前后状态和参数。指令 ID 应为 1～7，反馈 ID 为 17～23；逐台比较 PMAX、VMAX、TMAX 与该臂原记录。失败或量程不一致时先定位原因，不把旧值复制过去掩盖差异。读回报告留在本地。

## 4. 同步六处身份关联

备份以下文件后，只迁移该角色的 USB 身份及路径，保持已经核验的数值参数。此步骤需人工核对，没有一键按 tty 编号重绑定功能。

| 文件 | 需要同步的内容 |
| --- | --- |
| `diagnostics/arm_roles.yaml` | 对应角色 `usb_serial`、完整 `by_id` |
| `A6-Y-G/tools/arm_visualization/joint_mapping.yaml` | 对应角色 `usb_serial`；方向、比例和零偏保持原值 |
| `A6-Y-G/tools/arm_diagnostics/motor_position_limits.yaml` | 将该臂已核验的 ID→PMAX 项关联至新序列号 |
| `A6-Y-G/tools/arm_gravity/motor_limits.json` | 对应臂 `usb_serial`、`port`；ID 和量程应与刚才读回一致 |
| `A6-Y-G/tools/arm_gravity/profiles/<role>.yaml` | 已有 profile 时同步 `usb_serial`，保留数值；没有该文件时继续继承默认，不必为换板新建 |
| `A6-Y-G/tools/arm_gravity/teaching_gripper_limits.yaml` | 对应角色 `usb_serial`；同一电机、零位和量程确认不变后保留端点 |

在 `arm_roles.yaml` 的该角色下追加最小迁移记录，保留原序列号的关联；已有 `adapter_history` 时追加一项，不覆盖历史：

```yaml
adapter_history:
  - previous_binding:
      usb_serial: '旧板的序列号'
```

运行时身份读取不依赖这一历史字段，但诊断测试使用它把原有 PMAX 测量记录映射到当前适配器。不要为通过测试改写原始测量中的序列号或数值。

`motor_limits.json` 还保留只读来源与历史参数；不要把旧的 `observed_at` 改成新日期后宣称全部寄存器均已更新。新的读回报告应独立保存。运行 YAML 中的端点是本套设备的参数，不适用于未经核验的其他夹爪。

## 5. 核验并重新加载

```bash
bash A6-Y-G/scripts/run_arm_scan.bash --cli --arms teaching_right --passes 5
bash A6-Y-G/scripts/run_arm_teaching.bash --check
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -s -m unittest discover \
  -s A6-Y-G/tools/arm_diagnostics -p 'test_position_feedback.py' -v
```

确认正式绑定能收到 7 台电机反馈，且离线配置检查通过。关闭旧示教窗口并重新打开后，才会加载新绑定。下一步按[真机操作指南](run-power-teaching.md)手动选择配对和启动；本替换流程不自动使能设备。
