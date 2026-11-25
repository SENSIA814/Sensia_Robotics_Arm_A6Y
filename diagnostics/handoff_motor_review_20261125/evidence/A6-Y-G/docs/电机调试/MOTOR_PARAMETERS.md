# 电机内部参数读取、临时修改与保存

这三个工具移植自 `/home/alce/Code/A6-Y/src/a6_y_ros2_control/tools/`，用于直接访问达妙电机的内部寄存器，与 ROS 控制器 YAML 中的速度上限、阻抗参数不同。

| 命令 | 功能 |
| --- | --- |
| `read_motor_params.py` | 读取位置环、速度环、加减速等参数；只发送读取帧 |
| `write_motor_params_temp.py` | 写入 RAM 并逐项读回；不保存 Flash |
| `save_motor_params_flash.py` | 失能指定电机、保存当前全部参数到 Flash，检查保存应答 |

默认使用当前项目的 `a6_y_controllers/config/a5_motors.yaml`，J1 对应 `DM4340P / 0x01 / 0x11`。支持当前顶层 `joints` 和旧项目的嵌套 YAML；旧项目专用的 `--leader` 已移除。可用 `--config 路径` 指定其他映射，`--channel` 默认为 `/dev/ttyACM0`。所有工具与硬件驱动共用串口占用锁，先退出控制 launch、反馈采样等占用串口的程序。

## 环境与安装

在项目根目录执行：

```bash
source scripts/environment.bash
colcon build --base-paths src --packages-select a6_y_ros2_control --symlink-install
a6y_load_install
```

也可在项目根目录直接运行 `python3 src/a6_y_ros2_control/tools/工具名.py`。依赖沿用当前包已有的 `python3-can`、`python3-serial`、`python3-yaml` 和 `ament_index_python`。

最新 J1、J2、J3 单轴试验均已验证 90°/s 峰值。J1 选用 `KP_APR=2500`；J2 选用 `KP_APR=2500`、`KP_ASR=0.007296`、`KI_ASR=0.0016`；J3 选用 `KP_APR=2500`、`KP_ASR=0.004608`、`KI_ASR=0.002`。这些阶段结束时均仅写 RAM；2026-10-10 已按用户要求将 J1～J4 当前参数执行 Flash 保存并确认应答，掉电重启验证待完成，见 [当前参数与保存记录](J1_J4_FLASH_SAVE_20261010.md)。各轴验证姿态、行程和适用范围不同，不能视为六轴同时高速运动验收；结果及回退命令见 [J1 高速调参记录](J1_HIGH_SPEED_TUNING_20261009.md)、[J2 高速调参记录](J2_HIGH_SPEED_TUNING_20261009.md)和 [J3 高速调参记录](J3_HIGH_SPEED_TUNING_20261009.md)。

J4 当前实读 `KP_APR=220、KP_ASR=0.001096、KI_ASR=0.0005、KI_APR=0、ACC/DEC=2/−2`，已执行 Flash 保存并确认应答。用户确认 40°/s 规划下运动稳定；现场平稳评价不替代数值速度验收，90°/s 尚未验证。当前记录和历史结果见 [J4 调参记录](J4_HIGH_SPEED_TUNING_20261009.md)。

## 1. 读取 J1

```bash
ros2 run a6_y_ros2_control read_motor_params.py --joints joint1 \
  --params CTRL_MODE,KP_APR,KI_APR,KP_ASR,KI_ASR,ACC,DEC,MAX_SPD
```

不指定 `--params` 时读取四个位置环/速度环增益；不指定目标时读取 J1、J2、J3。`--all` 读取配置中的全部七台电机；`--ids 0x01` 可直接指定 ID。名称区分大小写，也可用寄存器编号。`--dry-run` 只显示读取计划，不打开串口。

需要保留完整的 J1 原始寄存器 JSON 快照时，继续用已有的专用工具：

```bash
python3 tools/read_motor_position_params.py --joint joint1
```

2026-10-09 的[只读快照](../../validation/j1_position/j1_motor_params_before_tuning.json)记录 `KP_APR=54`、`KI_APR=0`、`KP_ASR≈0.00384`、`KI_ASR≈0.002`。这是当时的实测值，后续调整前应重新读取。

## 2. 临时修改并读回

以下以 J1 的 `KP_APR=60` 演示命令语法；本轮实测选值及适用范围见 [J1 调参记录](J1_POSITION_TUNING_20261009.md)，示例值不是本轮最终选值。先预览：

```bash
ros2 run a6_y_ros2_control write_motor_params_temp.py \
  --joints joint1 --set KP_APR=60 --dry-run
```

决定执行该项试调时，退出控制程序并支撑机械臂，再执行：

```bash
ros2 run a6_y_ros2_control write_motor_params_temp.py \
  --joints joint1 --set KP_APR=60 --yes
```

输出包含 `before,after,ack`。只有读回与目标的 float32 数值一致才报告成功；失败即停止后续写入，不会自动回滚已写参数。写入立即改变 RAM，工具不会使能电机，也不会自动保存 Flash。需要恢复本次示例的原值时：

```bash
ros2 run a6_y_ros2_control write_motor_params_temp.py \
  --joints joint1 --set KP_APR=54 --yes
```

默认允许调整 `ACC,DEC,MAX_SPD,KP_ASR,KI_ASR,KP_APR,KI_APR`。重复 `--set` 可设置多项，`--joints joint1 joint2` 可选多电机。旧工具的 `--allow-any-param` 仍保留，但寄存器是否可写、单位和有效范围取决于电机固件；ID 变更使用专用 [ID 工具](MOTOR_ID_SETUP.md)。`PMAX/VMAX/TMAX` 是协议映射范围，不能当作提速参数。

2026-10-09 实读 J3 `VMAX=20`，与同型号电机默认的 10 不同。A5 三份电机 YAML 已为 `joint3` 设置 `protocol_vmax: 20.0`，C++ 硬件插件以及只读扫描/反馈采样工具按此量程解码；J1/J2 仍使用默认 10。这个字段必须匹配电机实际寄存器值，不会写电机参数，也不是运动限速。实际指令速度由 `max_velocity`、`position_velocity_limit` 等控制项限制。修复依据和验证见 [J3 调参记录](J3_POSITION_TUNING_20261009.md)。

随后 J2 高速试验实读 J4/J5/J6 `VMAX=50`，与 DM4310 旧默认解码量程 30 不同；三份 A5 电机 YAML 已为这三轴补上 `protocol_vmax: 50.0`。没有修改电机 VMAX 或日常 VR 速度上限。读取证据、准备流程修正和验证见 [J2 高速调参记录](J2_HIGH_SPEED_TUNING_20261009.md)。

试调后再运行相同的 [J1 30°/12 秒位置测试](../VR遥操作/实机控制/VR_REAL_ROBOT_USAGE.md)，比较跟踪误差、速度、过冲和静止抖动；一次改变一个参数，才能判断效果。

## 3. 将已验证的参数保存到 Flash

工具会失能指定电机；操作前应停止控制程序并支撑机械臂。先预览，再在需要永久保存时加 `--yes`：

```bash
ros2 run a6_y_ros2_control save_motor_params_flash.py --joints joint1 --dry-run
ros2 run a6_y_ros2_control save_motor_params_flash.py --joints joint1 --yes
```

保存的是该电机**当前全部 RAM 参数**。`--params` 仅指定保存前后打印哪些参数，不限制保存范围。已收到保存应答和 RAM 读回仍不能证明掉电保持；保存过程中保持供电，命令结束后断电再上电，重新执行读取命令核对。若提示 `SAVE_UNCONFIRMED`，不要连续盲目重发保存，先断电读回判断结果。

## 本次移植差异与验证

- 复用当前项目的 `dm_canable2` 驱动、配置查找和串口锁，没有覆盖现有 C++ 控制驱动。
- Flash 保存改用当前项目已实现的模式对应失能帧及 `AA 01` 应答检查；旧工具默认 `AA 00`。
- 临时写入排除 NaN/Inf、溢出和越界寄存器编号；对小增益按实际 float32 值校验，避免旧驱动宽松应答比较误报成功。
- 默认写入/保存仍需显式 `--yes`；`--dry-run` 即使同时带 `--yes` 也不访问串口。

2026-10-09 已完成包构建安装、10 项离线测试及三个 `ros2 run` 入口的预览验证。测试覆盖当前/旧 YAML、只读帧、RAM 写入与读回、写入拒绝、小增益不一致、串口锁、原生位置模式失能地址、Flash 帧和缺失应答，使用模拟寄存器与临时文件，不连接实机；本次移植未调整电机参数。


## 电机初始参数记录

以下保留调参前的原始读数。2026-10-09 首轮低速实机试验分别选用 J1、J2、J3 `KP_APR=250`，均只写 RAM、未保存 Flash；适用姿态、速度、逐轮结果和回退命令见 [J1 记录](J1_POSITION_TUNING_20261009.md)、[J2 记录](J2_POSITION_TUNING_20261009.md) 与 [J3 记录](J3_POSITION_TUNING_20261009.md)。掉电后以实际读回为准，不应将下方初始值 54 当成本轮结束值。

  alce@alce-B:~/Code/A6-Y-G$ ros2 run a6_y_ros2_control read_motor_params.py --joints joint1
  read targets:
  joint,motor_type,slave_id,master_id
  joint1,DM4340P,0x01,0x11

  params:
  KP_ASR,KI_ASR,KP_APR,KI_APR

  read result:
  joint,motor_type,slave_id,KP_ASR,KI_ASR,KP_APR,KI_APR
  joint1,DM4340P,0x01,0.00384,0.002,54,0
  alce@alce-B:~/Code/A6-Y-G$ ros2 run a6_y_ros2_control read_motor_params.py --joints joint2
  read targets:
  joint,motor_type,slave_id,master_id
  joint2,DM4340P,0x02,0x12

  params:
  KP_ASR,KI_ASR,KP_APR,KI_APR

  read result:
  joint,motor_type,slave_id,KP_ASR,KI_ASR,KP_APR,KI_APR
  joint2,DM4340P,0x02,0.00384,0.002,54,0
  alce@alce-B:~/Code/A6-Y-G$ ros2 run a6_y_ros2_control read_motor_params.py --joints joint3
  read targets:
  joint,motor_type,slave_id,master_id
  joint3,DM4340P,0x03,0x13

  params:
  KP_ASR,KI_ASR,KP_APR,KI_APR

  read result:
  joint,motor_type,slave_id,KP_ASR,KI_ASR,KP_APR,KI_APR
  joint3,DM4340P,0x03,0.00384,0.002,54,0
  alce@alce-B:~/Code/A6-Y-G$ ros2 run a6_y_ros2_control read_motor_params.py --joints joint4
  read targets:
  joint,motor_type,slave_id,master_id
  joint4,DM4310,0x04,0x14

  params:
  KP_ASR,KI_ASR,KP_APR,KI_APR

  read result:
  joint,motor_type,slave_id,KP_ASR,KI_ASR,KP_APR,KI_APR
  joint4,DM4310,0x04,0.00372,0.002,54,0
  alce@alce-B:~/Code/A6-Y-G$ ros2 run a6_y_ros2_control read_motor_params.py --joints joint5
  read targets:
  joint,motor_type,slave_id,master_id
  joint5,DM4310,0x05,0x15

  params:
  KP_ASR,KI_ASR,KP_APR,KI_APR

  read result:
  joint,motor_type,slave_id,KP_ASR,KI_ASR,KP_APR,KI_APR
  joint5,DM4310,0x05,0.00372,0.002,54,0
  alce@alce-B:~/Code/A6-Y-G$ ros2 run a6_y_ros2_control read_motor_params.py --joints joint6
  read targets:
  joint,motor_type,slave_id,master_id
  joint6,DM4310,0x06,0x16

  params:
  KP_ASR,KI_ASR,KP_APR,KI_APR

  read result:
  joint,motor_type,slave_id,KP_ASR,KI_ASR,KP_APR,KI_APR
  joint6,DM4310,0x06,0.00372,0.002,54,0
