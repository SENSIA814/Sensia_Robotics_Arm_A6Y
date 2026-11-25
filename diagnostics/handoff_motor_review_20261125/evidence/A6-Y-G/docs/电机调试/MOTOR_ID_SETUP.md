# 电机 ID 标定完整流程：从未知 ID 到整臂核对

`damiao_set_id.py` 通过现有 CANable2/SLCAN 接口设置接收 ID（`ESC_ID`，寄存器 8）和反馈 ID（`MST_ID`，寄存器 7），读回后保存到 Flash。默认只预览，`--write` 才访问硬件并写入；`--verify` 只读核对。

流程：**只接一台电机 → 扫描未知 ID → 读取两个实际 ID → 选择关节编号 → 预览 → 写入保存 → 断电重启验证 → 贴标签记录 → 逐台重复 → 接回整臂并核对关节。**

本流程中的“标定”指电机编号和实体关节的对应确认。方向、编码器零点、角度比例及 TCP 的实物标定另见 [S0 标定说明](../标定与验证/S0_CALIBRATION.md)。

## 1. 本次日志是否代表成功

2026-10-08 用户提供的实机输出包含：

```text
READ ESC_ID=0x01 MST_ID=0x00
STATE=0 (disabled)
WRITE ESC_ID=0x01 RID=7 -> 0x11
READ ESC_ID=0x01 MST_ID=0x11
STATE=0 (disabled)
READ ESC_ID=0x01 MST_ID=0x11
SAVED ESC_ID=0x01 MST_ID=0x11; save ACK and readback received. Power-cycle, then --verify to check persistence.
```

这表示：已确认失能，将反馈 ID 从 `0x00` 改成 `0x11`，收到了保存应答，并再次读回正确的 `0x01 / 0x11`。接收 ID 原本就是 `0x01`，所以本次没有重写接收 ID。

**本次写入和保存应答核对成功，掉电保持尚待验证。** 不必再执行 `--write`；现在将电机电源断开再重新上电，然后执行：

```bash
ros2 run a6_y_ros2_control damiao_set_id.py \
  --channel /dev/ttyACM0 \
  --current-id 0x01 --current-master-id 0x11 --verify
```

若输出 `READ ESC_ID=0x01 MST_ID=0x11` 和 `VERIFIED`，即可确认这台电机的两个 ID 在本次掉电后保持正确。日志没有提供电机安装位置，实体关节对应仍需第 8 节的检查。

## 2. 两个 ID 的含义与目标编号

| 名称 | YAML 字段 | 用途 |
| --- | --- | --- |
| `ESC_ID`，寄存器 8 | `slave_id` | 电机接收命令的 ID |
| `MST_ID`，寄存器 7 | `master_id` | 电机发送反馈的 CAN 帧 ID |

`--current-id` 是当前访问地址；`--current-master-id` 是对现有反馈 ID 的期望检查值，**不会修改它**。设置目标编号使用 `--new-id` 和 `--new-master-id`。

按当前 [a5_motors.yaml](../../src/a6_y_controllers/config/a5_motors.yaml) 设置：

| 实体关节 | 配置关节名 | 电机型号 | 目标 ESC_ID | 目标 MST_ID |
| --- | --- | --- | --- | --- |
| J1 | `joint1` | DM4340P | `0x01` | `0x11` |
| J2 | `joint2` | DM4340P | `0x02` | `0x12` |
| J3 | `joint3` | DM4340P | `0x03` | `0x13` |
| J4 | `joint4` | DM4310 | `0x04` | `0x14` |
| J5 | `joint5` | DM4310 | `0x05` | `0x15` |
| J6 | `joint6` | DM4310 | `0x06` | `0x16` |
| 夹爪 | `gripper_finger_joint1` | DM3507 | `0x07` | `0x17` |

本项目约定 `MST_ID = ESC_ID + 0x10`。`0x10` 是十六进制的 16，因此 `0x11` 是十进制的 17。ID 工具支持接收 ID `0x01..0x0F`；目标反馈 ID 范围为 `0x10..0x7FE`，旧反馈 ID 为零时可以读取并迁移。

## 3. 接线与环境准备

每次只连接并上电一台待设置电机，先停止实机 launch、扫描和反馈采样程序，支撑好机械臂。写入过程会失能电机。多台出厂同 ID 的电机无法通过地址区分，必须逐台设置；工具对 1～15 的扫描只能发现有应答的其他电机，不能替代物理隔离。

换接下一台前先断电。默认通信参数为 `/dev/ttyACM0`、串口 `115200`、CAN `1000000 bit/s`。本工具使用 CANable2/SLCAN 接口，不能直接套用其他 USB-CAN 厂商的串口协议。串口号不确定时可执行 `ls -l /dev/ttyACM*`，然后把后续命令中的路径改成实际设备。

首次安装整个项目时，先按 [README](../../README.md#构建) 完成依赖和全部包的构建。已经构建过项目、仅新增或更新本工具时，执行：

```bash
cd /home/alce/Code/A6-Y-G
source scripts/environment.bash
colcon build --base-paths src --packages-select a6_y_ros2_control \
  --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
a6y_load_install
```

以后每个新终端从项目根目录运行 `source scripts/environment.bash` 和 `a6y_load_install` 即可。用 `ros2 run a6_y_ros2_control damiao_set_id.py --help` 检查工具是否安装；此命令不连接电机。

也可直接运行 `/usr/bin/python3 src/a6_y_ros2_control/tools/damiao_set_id.py ...`，无需启动 ROS 节点。

## 4. 不知道旧 ID：扫描并读取实际配置

只连接这一台电机，扫描本工具支持的 `0x01..0x0F`：

```bash
ros2 run a6_y_ros2_control damiao_bus_scan.py \
  --channel /dev/ttyACM0 --can-bitrate 1000000 --tty-baudrate 115200 \
  --ids 0x01-0x0F --motor-type DM4340P --passes 10
```

已知型号时，将 `DM4340P` 替换为对应的 `DM4310` 或 `DM3507`。该参数用于解码反馈，工具不会自动识别型号；型号未知时，上述命令可用于找有应答的地址，但不要把显示的速度、力矩等当作该型号的准确测量值。

看输出下半部分的 `scan result`：`FOUND` 行的 `slave_id` 是候选旧 ID，`hits` 是收到应答的次数。例如 `FOUND` 对应 `0x01`，就用它继续读取参数。

- 只接一台时，通常只有一个地址 `FOUND`，其余 `MISS`。因为扫描列表包含所有 15 个地址，最后显示 `missing ids` 和退出码 `1` 是这种用法的预期结果，不表示已找到的地址也失败了。
- 开头 `scan targets` 中反馈 ID 显示 `-`，表示没有预设反馈 ID，不是测得反馈 ID 为零。实际 `MST_ID` 要用下一条命令读取。
- `FOUND` 只是候选地址，仍以参数读回为准。如果全部 `MISS`、出现多个候选或读回不一致，先检查通信和单台连接情况，不继续写入。

假设找到的候选旧 ID 是 `0x01`，此时尚不知道反馈 ID，不要先指定期望值 `0x11`：

```bash
ros2 run a6_y_ros2_control damiao_set_id.py \
  --channel /dev/ttyACM0 --current-id 0x01 \
  --timeout 0.5 --read-attempts 3 --verify
```

例如输出：

```text
READ ESC_ID=0x01 MST_ID=0x00
VERIFIED: current IDs read back; no disable/write/save sent.
```

记录这两个实际值，下一步使用它们作为旧 ID。这里的 `VERIFIED` 只表示读到了当前配置，不表示它已经符合目标编号，也不表示做过掉电验证。

上述命令只查询，不使能、不运动、不修改 ID。扫描不会自动搜索其他 CAN 速率。若旧接收 ID 为零、超过 15 或协议不同，本工具不提供完整发现/迁移能力，需要用适配的厂商工具先确认配置。

## 5. 选择关节编号、预览并写入

先确定这台电机将安装到哪个关节，再查第 2 节目标表。以下以“实测旧 ID 为 `0x01 / 0x00`，目标为 J1 的 `0x01 / 0x11`”为例。其他电机必须替换为各自实测的旧 ID 和对应关节的目标 ID；例如目标 J4 使用 `--new-id 0x04 --new-master-id 0x14`。

先预览（不打开串口）：

```bash
ros2 run a6_y_ros2_control damiao_set_id.py \
  --channel /dev/ttyACM0 \
  --current-id 0x01 --current-master-id 0x00 \
  --new-id 0x01 --new-master-id 0x11 --dry-run
```

执行写入并保存：

```bash
ros2 run a6_y_ros2_control damiao_set_id.py \
  --channel /dev/ttyACM0 \
  --current-id 0x01 --current-master-id 0x00 \
  --new-id 0x01 --new-master-id 0x11 \
  --timeout 0.5 --read-attempts 3 --write
```

`--current-id` 必须替换为待设置电机的实际旧 ID。省略 `--new-master-id` 时，按项目约定使用 `new-id + 0x10`。省略 `--current-master-id` 会自动读取旧反馈 ID；指定它时应填写实际旧值（本例为 `0x00`），不匹配则拒绝写入。允许把旧反馈 ID 为零的电机迁移到非零反馈 ID。

`--timeout 0.5` 时，检查其他 14 个地址的应答窗口合计 7 秒，整个过程还需读取、写入和保存时间，请等待输出完成。出现 `SAVED` 且两个 ID 正确才进入下一步；本次已经得到第 1 节的成功输出，就不需要重复写入。

## 6. 断电重启验证、记录并换接下一台

出现 `SAVED` 表示收到了保存应答，并再次读回了两个 ID。等待工具结束，将这台电机电源断开再重新上电，然后用**新** ID 核对持久化结果：

```bash
ros2 run a6_y_ros2_control damiao_set_id.py \
  --channel /dev/ttyACM0 --current-id 0x01 --current-master-id 0x11 \
  --timeout 0.5 --read-attempts 3 --verify
```

`--verify` 不失能、不写参数、不保存。只有实际断电再上电后的核对才能验证掉电保持。

只重启程序或重新插拔仍未切断电机供电的 USB 适配器，不能代替电机掉电测试。通过标准为：`READ` 显示两个目标 ID、随后出现 `VERIFIED`、命令正常结束。

通过后贴上“关节名 + ESC_ID / MST_ID”标签，保存终端输出并填写下表。它是待填写模板，不代表已完成验收：

| 关节 | 电机标签/型号 | 实测旧 ESC / MST | 目标 ESC / MST | SAVED 核对 | 断电重读 | 实体关节核对 | 日期/记录位置 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| J1 | 待填 | 待填 | 0x01 / 0x11 | 待核对 | 待核对 | 待核对 | 待填 |
| J2 | 待填 | 待填 | 0x02 / 0x12 | 待核对 | 待核对 | 待核对 | 待填 |
| J3 | 待填 | 待填 | 0x03 / 0x13 | 待核对 | 待核对 | 待核对 | 待填 |
| J4 | 待填 | 待填 | 0x04 / 0x14 | 待核对 | 待核对 | 待核对 | 待填 |
| J5 | 待填 | 待填 | 0x05 / 0x15 | 待核对 | 待核对 | 待核对 | 待填 |
| J6 | 待填 | 待填 | 0x06 / 0x16 | 待核对 | 待核对 | 待核对 | 待填 |
| 夹爪 | 待填 | 待填 | 0x07 / 0x17 | 待核对 | 待核对 | 待核对 | 待填 |

将已完成的电机断电并从设置总线移除，再换接下一台，重复第 4～6 节。每换一台都重新读取旧 ID、选择目标关节编号，不要把上一台的参数直接套用。七台逐台验证完成后再接回整臂总线。

## 7. 接回整臂并核对软件配置

按第 2 节标准表设置时，下列三份现有配置使用相同的 ID 映射：

- [a5_motors.yaml](../../src/a6_y_controllers/config/a5_motors.yaml)：常规控制和本流程的扫描配置。
- [a5_vr_position_motors.yaml](../../src/a6_y_controllers/config/a5_vr_position_motors.yaml)：VR 位控配置。
- [a5_vr_position_full_motors.yaml](../../src/a6_y_controllers/config/a5_vr_position_full_motors.yaml)：完整 VR 位控配置。

工具不会修改本地 YAML。采用自定义编号时，同步相应关节的 `slave_id` / `master_id`，保证 ID 唯一且反馈 ID 与所有接收 ID 分离，再重新构建 `a6_y_controllers` 并加载环境。不要为编号改动 `sign`、`offset`、比例或运动限位。

七台全部接回后，保持控制程序退出，按配置扫描：

```bash
ros2 run a6_y_ros2_control damiao_bus_scan.py \
  --channel /dev/ttyACM0 --passes 10 \
  --config src/a6_y_controllers/config/a5_motors.yaml
```

这一次七台都应为 `FOUND`，不应有 `missing ids`。扫描不是严格的反馈 ID 寄存器核对，还应逐台读回确认。标准 `1..7 / 0x11..0x17` 映射可执行以下**只读**循环，失败时停止：

```bash
MOTOR_VERIFY_OK=1
for MOTOR_TARGET_ID in 1 2 3 4 5 6 7; do
  MOTOR_EXPECTED_MASTER=$((MOTOR_TARGET_ID + 16))
  if ! ros2 run a6_y_ros2_control damiao_set_id.py \
    --channel /dev/ttyACM0 \
    --current-id "$MOTOR_TARGET_ID" --current-master-id "$MOTOR_EXPECTED_MASTER" \
    --timeout 0.5 --read-attempts 3 --verify; then
    MOTOR_VERIFY_OK=0
    break
  fi
done
if [ "$MOTOR_VERIFY_OK" -eq 1 ]; then
  echo '七台电机的接收 ID 和反馈 ID 均核对通过。'
fi
```

自定义映射时按实际配置逐台核对，不使用上述标准循环。整臂接回后如需重新编号，再将待改电机单独隔离；不要在这个循环中加入 `--write`。

## 8. 核对“实体关节 ↔ 电机 ID”

七个地址有应答不能单独证明安装位置正确。保持电机失能并支撑好机械臂，逐轴缓慢手动改变目标关节的相对角度，观察对应 ID 的反馈。以 J4 为例：

```bash
ros2 run a6_y_ros2_control damiao_feedback_watch.py \
  --channel /dev/ttyACM0 \
  --config src/a6_y_controllers/config/a5_motors.yaml \
  --duration 20 --expect-joint joint4 \
  --log "/tmp/joint4_id_check_$(date +%s).jsonl"
```

看到 `connection ready` 后再移动。观察 `ID04` 是否随实体 J4 运动而变化，并记录其他 ID 是否也变化，以排查标签或对应错误。`CHANGE_SEEN` 只表示期望 ID 的位置变化超过阈值，不能替代操作者确认“究竟动了哪个实体关节”。原始反馈角度单位为 rad。

其他关节将 `joint4` 换为第 2 节对应名称，同时更换日志文件名。日志必须尚不存在。使用完整七轴配置时，七台都须在线；只接一台时其他轴缺失会使该采样工具返回失败。该工具只查询状态，不会主动使能、移动或标零。

完成标准：每台两个 ID 掉电重读正确、全总线无缺失/地址冲突、实体关节对应确认正确、三个软件配置的映射一致。之后再按相应控制模式的说明启动机械臂；ID 标定完成不代表零点、方向及其他现场验收也完成。

## 参数与执行行为

- 当前和目标 `ESC_ID` 范围为 `1..15`，对应本项目的 4 位反馈电机 ID；目标 `MST_ID` 范围为 `0x10..0x7FE`，避开电机 ID 区和协议地址 `0x7FF`。支持十进制和 `0x` 十六进制。
- 默认串口 `/dev/ttyACM0`、串口波特率 `115200`、CAN 速率 `1000000`；可用 `--channel`、`--tty-baudrate`、`--can-bitrate` 修改。
- `--timeout` 是每次应答收集窗口，默认 `0.25` 秒。必需的参数/状态读取在无匹配应答时最多尝试 `--read-attempts` 次，默认 `3`，可设 `1..10`；冲突应答和错误 ID 不靠重试放行。探测其他地址时每个地址查询一次。
- 工具先锁定串口，再读取当前 ID、检查其他电机应答，然后查询当前状态。新鲜状态反馈为 `0`（失能）时即可继续，不需要控制模式寄存器；状态为 `1`（使能）时，必须成功读取 `CTRL_MODE`，按对应模式发送失能指令，再确认新鲜失能反馈。故障状态或无状态应答时拒绝写入，不能跳过失能检查。
- 先写 `MST_ID` 并读回，再写 `ESC_ID` 并在新地址读回，最后再次确认失能并发送一次 `AA 01` 保存。不会发送使能、运动或标零指令，也不会修改控制模式。
- 保存指令会保存电机当时的全部可保存参数，包括此前其他程序留下的未保存改动。即使两个 ID 已经相同，显式 `--write` 仍保存一次；日常检查使用 `--verify`，不要循环写 Flash。
- 工具不改本地 YAML。按标准 J1～J6、夹爪映射设置时，使用 `0x01/0x11`～`0x07/0x17`。采用自定义映射时，同步检查 `a5_motors.yaml`、`a5_vr_position_motors.yaml` 和 `a5_vr_position_full_motors.yaml` 中相应关节的 `slave_id` / `master_id`，重新构建配置所属包后再启动控制。

寄存器、立即生效和 `AA 01` 保存应答格式参考 [DM-J4340P-2EC 使用手册](https://damiao.enactic.ai/en/products/hardware/dm-j4340p-2ec-v1.0/#save-parameters)；修改和保存需失能的要求也见 [达妙 SDK 使用说明](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/USAGE.md)。固件必须支持 CAN 参数读写及保存应答；工具遇到不支持的应答会报错，不能把模拟验证视为所有型号/固件的实机验收。

## 失败后的核对

工具失败退出码为 `2`，中断为 `130`。错误包含当前执行阶段；已发送写入时，可能只修改了反馈 ID 或两个 ID，不能假定失败后仍保留旧地址。程序不自动回滚、不重复发送保存命令。

| 输出/现象 | 处理 |
| --- | --- |
| `Unexpected feedback ID 0x00; expected 0x11` | 省略 `--current-master-id` 读取实际反馈 ID；需要改成 `0x11` 时使用目标参数和 `--write`。 |
| 扫描全部 `MISS` 或参数始终无应答 | 核对实际串口、供电、CAN 接线、速率与地址范围；不要仅根据示例向 `0x01` 写参数。 |
| `Another motor replied` / `Conflicting replies` | 确认只有一台电机连接上电，排查其他总线发送者及重复 ID。 |
| `No fresh status reply` / `Disabled state not confirmed` | 状态无法确认或反馈含故障，先恢复可验证的电机状态，不跳过检查。 |
| 串口锁失败/权限错误 | 退出占用程序，确认设备路径和当前用户串口权限，不并行运行扫描、写入与控制。 |
| `interrupted`、退出码 130 | 操作者按了 Ctrl+C。打开/读取/扫描阶段尚未写参数；若已出现 `WRITE`，必须重新核对可能的部分改动。 |

保持单电机连接，分别用旧 ID 和目标 ID 执行 `--verify`；省略 `--current-master-id` 可读取当前反馈 ID。若提示 `SAVE_UNCONFIRMED`，只说明未收到可靠保存应答，即使读回正确也不能判断掉电保持。记录当前读数，断电再上电后重新核对。部分改动未保存时，电机通常恢复到之前保存的参数；必须以读回结果为准。

### 已读到 ID，但 `RID=10` 超时

`RID=10` 是控制模式寄存器。旧版本无论电机是否已失能，都会强制读取此寄存器，且只查询一次；超时发生在实际失能及写入之前。新版本已增加有界只读重试，并在已收到失能状态时去掉这项不必要的前置要求。仅凭超时不能确定是丢包、固件不应答还是通信中断。

例如当前电机为 `0x01 / 0x00`，目标为 `0x01 / 0x11`，单电机连接时执行：

```bash
ros2 run a6_y_ros2_control damiao_set_id.py \
  --channel /dev/ttyACM0 --current-id 0x01 --current-master-id 0x00 \
  --new-id 0x01 --new-master-id 0x11 --timeout 0.5 --read-attempts 3 --write
```

终端会显示 `OPEN`、`CONNECTED`、其他 ID 的查询进度和 `STATE`。`--timeout 0.5` 时，其他 14 个地址的应答窗口合计 7 秒，请等待检查结束。`STATE=0 (disabled)` 表示已确认失能；若仍为使能状态且模式读取超时，不会猜测控制模式继续写入。

若仍失败，用以下命令收集只读诊断；它在控制模式无应答时仍会继续查询状态，诊断不完整则退出 `2`：

```bash
ros2 run a6_y_ros2_control damiao_set_id.py \
  --channel /dev/ttyACM0 --current-id 0x01 \
  --timeout 0.5 --read-attempts 3 --diagnose --trace
```

该命令不会失能或写入。`TX/RX` 显示实际 CAN 帧；`DRAIN RX` 是发送新查询前清理的旧帧，不作为本次读回证据。如果写入流程已出现 `WRITE`，诊断时应按错误提示分别核对旧/新地址，不能假定 ID 未变。`--verify` 仍只核对两个 ID，不要求 `CTRL_MODE` 可读。

## 无硬件验证

```bash
/usr/bin/python3 src/a6_y_ros2_control/test/test_damiao_set_id.py
ctest --test-dir build/a6_y_ros2_control -R damiao_set_id --output-on-failure
```

测试使用模拟寄存器/Flash 和伪终端 SLCAN，覆盖旧/新地址写入应答、模式相关失能、地址冲突、只读核对、串口锁、部分写入、保存应答缺失、模拟断电重读，以及 RID=10 单次丢包/持续无应答、失能状态确认和只读诊断，不打开真实 CAN 设备。

本工具版本已通过 34 项模拟/伪终端测试。第 1 节实机成功记录来自用户提供的输出；掉电重读和整臂关节对应结果仍需实际记录。
