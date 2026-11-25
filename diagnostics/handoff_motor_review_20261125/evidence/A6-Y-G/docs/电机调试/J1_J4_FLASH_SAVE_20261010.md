# J1～J4 当前参数与 Flash 保存（2026-10-10）

用户确认“现在的在40°每秒已经稳定运动”，并要求记录当前 J4 参数、将 J1～J4 当前参数永久写入。该反馈记录为 **J4 在 40°/s 规划下现场运动稳定**，不等同于所有数值性能指标通过。本次只读取、保存电机参数，没有使能或执行运动试验，没有改写环路增益。

## 当前参数

下表来自本次硬件读回，显示值按常用有效位数整理；精确 float32 值、原始寄存器位模式和固件版本见各轴 24 项 JSON 快照。

| 关节 | KP_APR（位置 P） | KI_APR（位置 I） | KP_ASR（速度 P） | KI_ASR（速度 I） | ACC / DEC | CTRL_MODE |
| --- | ---: | ---: | ---: | ---: | --- | ---: |
| J1 | 2500 | 0 | 0.00384 | 0.002 | 2 / −2 | 2 |
| J2 | 2500 | 0 | 0.007296 | 0.0016 | 2 / −2 | 2 |
| J3 | 2500 | 0 | 0.004608 | 0.002 | 2 / −2 | 2 |
| J4 | **220** | **0** | **0.001096** | **0.0005** | **2 / −2** | **2** |

J4 其余实读值包括 `MAX_SPD=600、PMAX=12.5、VMAX=50、TMAX=10、Gr=10、TIMEOUT=0、I_BW=1000、V_BW=40、Deta=4、IQ_c1=2500、VL_c1=100、OC_Value≈0.8`，ID 为 `0x04 / 0x14`。`PMAX/VMAX/TMAX` 是协议映射范围，不能当作本轮规划速度。

## 保存执行与核对

证据目录：[本次保存记录](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/)。

1. 保存前独立反馈对七台电机各收到 125 帧，状态均为 0（失能）；随后分别备份 J1～J4 的 24 项参数。
2. 逐台调用 `save_motor_params_flash.py --joints jointN --yes`，仅选择 J1、J2、J3、J4。每台发送一次已确认的 `AA 01` 保存操作；四次均正常退出，工具要求收到保存应答并核对读回。各台保存前后打印的 11 项参数一致。
3. 独立复读确认 J1～J4 各 24 项寄存器与保存前完全一致（含原始位模式）；最终独立反馈确认七电机全部失能，串口空闲。见[审计结果](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/audit.json)和[最终反馈](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/status_after.jsonl)。
4. **尚未进行电机电源断电重启，因此掉电保持验证待完成。** 保存应答与当前读回确认，不替代掉电后的复核。

保存的是各电机当前全部 RAM 参数；`--params` 只选择读回检查项，不限制 Flash 保存范围。J5、J6、夹爪没有收到 Flash 保存命令。主机 YAML（包括位置跟踪保护门限）不会因此写入电机 Flash。

| 关节 | 保存前 24 项 | Flash 保存日志 | 保存后 24 项 |
| --- | --- | --- | --- |
| J1 | [快照](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint1_before.json) | [日志](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint1_flash_save.log) | [快照](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint1_after.json) |
| J2 | [快照](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint2_before.json) | [日志](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint2_flash_save.log) | [快照](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint2_after.json) |
| J3 | [快照](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint3_before.json) | [日志](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint3_flash_save.log) | [快照](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint3_after.json) |
| J4 | [快照](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint4_before.json) | [日志](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint4_flash_save.log) | [快照](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/joint4_after.json) |

逐台命令、退出码与执行状态见 [保存清单](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/flash_save_manifest.json)。

## J4 40°/s 反馈的适用范围

用户确认的是当前参数下的现场平稳性。最近一份已完成的 40°/s 报告为 [1791614454298610983_real](../../validation/j4_highspeed/1791614454298610983_real/report.json)，90°往返、端点保持 4 秒，`passed=true、disarm_confirmed=true`。按 200 ms 编码器位置差分复算：去程峰值 **40.8697°/s（+2.17%）**，回程峰值 **42.2254°/s（+5.56%）**，因此这份记录的回程峰值仍超出此前 ±5% 数值条件。见[复算结果](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/latest_40dps_analysis.json)。

该历史试验没有电机增益快照，不能把本次读回的增益追溯标记为它已核实的试验参数。此前 45/50°/s 还有保护中止记录，不能据此宣布这些速度或 90°/s 已验证。

本次记录时共享完整位控配置的 `vr_tracking_error=0.25 rad`，`vr_tracking_governor=true`；它与较早的 `0.05 rad` 试验条件不同，已随[配置源码快照](../../validation/motor_flash/20261010_j1_j4_1791614694106986515/sources/src/a6_y_controllers/config/a5_vr_position_full_controllers.yaml)存档。本次没有修改该设置。

## 掉电后复核

保存命令已结束。现场确保机械臂有支撑并停止所有控制程序后，电机电源断电再上电；先只读核对参数，再进行运动。以下命令不会保存、使能或运动：

```bash
cd /home/alce/Code/A6-Y-G
source scripts/environment.bash
a6y_load_install
ros2 run a6_y_ros2_control read_motor_params.py \
  --joints joint1 joint2 joint3 joint4 \
  --params CTRL_MODE,KP_APR,KI_APR,KP_ASR,KI_ASR,ACC,DEC,MAX_SPD,PMAX,VMAX,TMAX
```

与上表及 JSON 快照比较；复核前不要再次运行参数写入命令，以免覆盖掉电后实际值。
