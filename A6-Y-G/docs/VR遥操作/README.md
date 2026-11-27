# VR 遥操作

Quest 右手柄通过 HTTPS / WebXR 网关发送位姿与按键，ROS 节点把相对手柄动作映射为 A5 末端目标，再通过 IK 生成关节参考。本目录包含 **RViz 虚拟模型、原生位置实机、关节阻抗实机** 三条入口，以及录制、回放、诊断和离线验证工具。

VR 当前控制一条选定机械臂，使用 `/arm6` 或 `/arm6_vr` 命名空间；它不是四臂动力示教的输入模式，也不提供 Quest 手柄力反馈。动力示教的夹爪端点和 2 N·m 反馈上限不会自动应用到 VR 配置。

## 选择入口

| 入口 | 启动命令 | 默认行为 |
| --- | --- | --- |
| 虚拟模型 | `bash scripts/run_vr_rviz.bash` | 仅发布运动学模型和 RViz 显示；没有串口、CAN 或 ros2_control |
| 原生位置 | `bash scripts/run_vr_position.bash` | `readonly`；读取实机反馈、预览参考，拒绝 PREPARE |
| 完整位置试运行 | `bash scripts/run_vr_position_full.bash` | `commission / follow / full / operator_trial`；启动为 DISABLED，须本地 PREPARE、ARM |
| 关节阻抗 | `bash scripts/run_vr_real.bash` | `impedance + readonly`；使能与跟随须另选调试模式并满足其配置检查 |

位置模式让电机以 `POS_VEL` 执行位置闭环；阻抗模式使用关节阻抗与重力补偿输出力矩。位置入口目前支持 `readonly`、`mock` 和 `commission`，**不支持 `mode:=execute`**。阻抗的 `execute` 会检查现场验收记录，当前配置仍有未完成项；不能只改状态标签绕过检查。

以下命令均从仓库的 **`A6-Y-G/` 目录**执行。多终端操作时，每个终端先进入这一目录。

## 准备构建与网关

先完成[软件环境安装](../tutorials/first-offline-run.md)。VR 使用主工程构建，不要把 MoveIt 独立栈同名驱动一起放入构建路径：

```bash
# 从仓库根目录进入主工程
cd A6-Y-G
source scripts/environment.bash
colcon build --base-paths src --packages-up-to a6_y_vr_teleop \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
a6y_load_install
```

网关使用独立 Python 环境。下列命令在未加载 ROS 的终端中执行，需要系统提供 `python3-venv`：

```bash
/usr/bin/python3 -m venv .venv-vr
.venv-vr/bin/python -m pip install -r vr_gateway/requirements.lock

read -r -p "主机局域网 IPv4 地址: " A6Y_VR_IP
bash scripts/create_vr_certificate.bash "$A6Y_VR_IP"
```

证书位于 `validation/vr_rviz/tls/`，脚本不覆盖已有文件。IP 变化后先另存旧证书，再生成包含新 IP 的证书。私钥和录制文件留在本地。网关第三方依赖及来源见 [NOTICE](../../vr_gateway/NOTICE.md) 与 [requirements.lock](../../vr_gateway/requirements.lock)。

Quest 与主机需要能够互通，浏览器访问 `https://<主机局域网IP>:<端口>`，完成本地证书信任后进入沉浸式 XR。网页传递手柄输入；机械臂显示在电脑 RViz，当前入口不传输电脑画面或摄像头图像。

| 用途 | ROS domain | UDP 输入 | HTTPS / WSS |
| --- | --- | --- | --- |
| 虚拟模型 | 84 | 15150 | 网关默认 8012、`--profile virtual` |
| 隔离 mock | 86 | 15154 | 测试工具提供模拟输入，无需 Quest 网关 |
| 实机两种后端 | 0，或 `A6Y_REAL_ROS_DOMAIN_ID` | 15153 | `--profile real --port 8014` |

ROS 限定 localhost，网关默认监听 `0.0.0.0`。同一 UDP 输入端口不要同时接 Quest、演示和回放。默认虚拟域与配置 `domain_id` 要一致；不能只改环境变量而保留旧配置。

## 先运行虚拟机械臂

终端一启动 RViz，终端二启动默认虚拟网关：

```bash
# 终端一
bash scripts/run_vr_rviz.bash

# 终端二
bash scripts/run_vr_gateway.bash
```

Quest 打开 8012 端口并进入 XR。首次连接或恢复连接后，先松开右侧握把，再按下建立相对参考。

| 操作 | 虚拟模型行为 |
| --- | --- |
| 按住右手侧握把 | 从当前虚拟 TCP 建立参考，跟随手柄平移和旋转 |
| 松开侧握把 | 冻结当前六轴与夹爪，进入 HOLD |
| TELEOP 中扣食指扳机 | 越扣越闭合，释放扳机张开 |
| HOLD 中按 A | 保持当前状态，下一次握把重新建参考；不会回零 |
| 输入超时或追踪异常 | 进入 PAUSED，检查后松开并重新握住 |

默认平移比例 0.5、旋转比例 1，VR 右 / 上 / 后映射为模型负 Y / 正 Z / 负 X。虚拟模型的 HOLD 只冻结关节位置，不计算承重力矩。参数、状态话题与原始验证说明见[虚拟机械臂使用说明](虚拟机械臂/VR_RVIZ_USAGE.md)。

## 原生位置实机：三个终端

VR 启动器的底层默认串口仍为 `/dev/ttyACM0`，**不会自动采用四臂角色选择器**。先查看 `/dev/serial/by-id/`，使用选定机械臂的稳定路径；该串口不得同时被诊断、MoveIt、重力补偿或动力示教占用。换板后的核对步骤见[适配器迁移指南](../how-to/replace-usb-can-adapter.md)。

终端一选定设备，启动完整位置试运行。此命令会连接实机，启动后保持 DISABLED；`prepare` 才会使能当前位置保持：

```bash
ls -l /dev/serial/by-id/
read -r -p "目标机械臂的 /dev/serial/by-id/... 路径: " A6Y_VR_CAN
bash scripts/run_vr_position_full.bash can_channel:="$A6Y_VR_CAN"
```

终端二启动实机网关。Quest 改为访问 8014 端口；8012 的默认虚拟网关不会向实机输入端口发送数据：

```bash
bash scripts/run_vr_gateway.bash --profile real --port 8014
```

终端三在机械臂已支撑、右手侧握把释放时进行接管：

```bash
bash scripts/vr_position_control.bash status
bash scripts/vr_position_control.bash prepare
bash scripts/vr_position_control.bash status
# 等待 prepare_ramp_complete=true，确认保持稳定且输入新鲜，再执行：
bash scripts/vr_position_control.bash arm
```

按住侧握把后，手柄平移 / 旋转控制 TCP，食指扳机控制夹爪。接管时捕获真实姿态和夹爪开度；扳机相对接管值变化超过 0.02 后才接入夹爪目标，避免首次接管立即跳到扳机端点。

`full` 与默认 `trial` 使用独立配置：

| 设置 | 默认 trial | full |
| --- | --- | --- |
| 末端控制 | 受限平移 | 平移与旋转 |
| 夹爪 | 只读 | 扳机控制，模型单指范围 −0.03～0 m |
| 六轴参考限速 | 0.03 rad/s | 1.0 rad/s |
| 夹爪参考限速 | 不下发夹爪目标 | 0.003 m/s |
| 局部范围 | TCP 5 mm；首次 PREPARE 后参考 / 实际累计 1° / 2° | 不采用这组局部范围限制 |
| CAD 几何检查 | 启用 | 关闭；仍保留关节、速度、反馈和跟踪检查 |
| 失联后重新接管 | 新鲜输入、释放握把后重新 ARM | 同一会话恢复且握把释放时可自动回到已许可保持状态；人工 PAUSE 仍需重新 ARM |

这些是软件配置，不是实际最大速度或停距指标。`full` 的 TCP 参考速度上限为 1.8 m/s、角速度 7.5 rad/s，实际运动还受 IK、关节限制、反馈减速和电机响应约束。当前它属于显式选择的现场试运行入口。

只想读取与预览时，用 `run_vr_position.bash can_channel:=...` 的默认 readonly；仅保持调试用 `mode:=commission stage:=hold`，该模式拒绝 ARM。详细参数和历史调试记录见[实机控制使用说明](实机控制/VR_REAL_ROBOT_USAGE.md)。

## 关节阻抗实机

`run_vr_real.bash` 保留原关节阻抗后端。它的默认 readonly 与位置入口一样不允许 PREPARE；只读模式仍会打开设备并查询反馈，驱动退出时会发送失能帧。

```bash
# 已在本终端设置并核对 A6Y_VR_CAN
bash scripts/run_vr_real.bash can_channel:="$A6Y_VR_CAN"

# 改用支撑下保持调试；两条 launch 不要同时运行
bash scripts/run_vr_real.bash mode:=commission stage:=hold \
  can_channel:="$A6Y_VR_CAN"
```

保持或跟随调试使用实机网关 `--profile real --port 8014`。本地命令需加载工程环境，指定阻抗控制器和与启动器相同的 ROS 域：

```bash
source scripts/environment.bash
a6y_load_install
export ROS_DOMAIN_ID="${A6Y_REAL_ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=1
/usr/bin/python3 tools/vr_control.py --controller impedance status
/usr/bin/python3 tools/vr_control.py --controller impedance prepare
```

PREPARE 捕获实际关节位置并进入 HOLD，刚度与重力补偿按配置渐增。`stage:=hold` 只验证保持，不能 ARM。满足阶段检查后可用 `mode:=commission stage:=follow`；受限 `operator_trial:=true` 是单独的现场试运行选项，不会把未完成验收标为通过。阻抗路径的前置条件、限制和 `vr_preflight.py` 输出解释见[实机控制说明](实机控制/VR_REAL_ROBOT_USAGE.md#输出前检查与本地操作)。

## 停止、失联与退出

- **松开握把**：停止当前跟随并保持，后续握抓重新捕获参考。它不会回零。
- **PAUSE**：撤销跟随许可，保留当前位置保持；需新鲜输入和释放握把后重新 ARM。
- **DISARM**：撤去电机输出，机械臂与夹爪可能失去支撑 / 夹持；先托住再执行。
- **输入失联或追踪异常**：进入停止 / 暂停流程；full 的同会话松手恢复与 trial / 阻抗的人工重新 ARM 不同。
- **反馈失效、越界、超速或停止超时**：可进入 FAULT 并走失能路径，不自动重新使能。
- **Ctrl+C 退出实机 launch**：停用控制器并失能。正常操作先松手或 PAUSE、托住机械臂，再 DISARM 和退出；这条 VR 流程不执行动力示教的六轴回零。

```bash
# 原生位置后端
bash scripts/vr_position_control.bash pause
bash scripts/vr_position_control.bash disarm

# 阻抗后端：在已加载环境、同域终端执行
/usr/bin/python3 tools/vr_control.py --controller impedance pause
/usr/bin/python3 tools/vr_control.py --controller impedance disarm
```

只停网关会造成输入超时，并不等同于已核验所有电机失能。主机或 CAN 中断时软件也不能保证承重继续；实机验证需独立停机方式和机械支撑。

## 录制、演示与回放

虚拟入口可记录输入包、接收时间、每步 dt、关节状态与配置哈希。文件名必须尚不存在：

```bash
bash scripts/run_vr_rviz.bash record_file:="$PWD/validation/vr_rviz/my-session.jsonl"

# 另一个终端：演示或回放输入只发到虚拟节点
source scripts/environment.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
/usr/bin/python3 tools/vr_replay.py --demo --duration 16
/usr/bin/python3 tools/vr_replay.py --replay validation/vr_rviz/my-session.jsonl

# 不启动 ROS 节点、不发网络包，按原 dt 离线重算
/usr/bin/python3 tools/verify_vr_replay.py validation/vr_rviz/my-session.jsonl
```

演示、回放和 Quest 输入分别运行，不能同时向同一端口发送。`vr_replay.py` 拒绝实机保留端口 15153，实机桥接也拒绝 demo / replay 来源。录制不等于授权重放实机运动。

网关可加 `--record <新JSONL文件>` 保存原始事件与规范化输入；实机 launch 可加 `record_file:=<新JSONL文件>`，`commission` 阶段默认在 `validation/vr_commission/` 创建记录。离线回放工具只重算虚拟模型，不重放实机力矩或验证接触动力学。

## 离线测试与排查

以下检查使用软件模型或隔离 mock；不需要连接机械臂：

```bash
source scripts/environment.bash
a6y_load_install
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$PWD/src/a6_y_vr_teleop:${PYTHONPATH:-}"
/usr/bin/python3 -m pytest -q src/a6_y_vr_teleop/test

# 自动创建并结束自己的虚拟 graph：domain 85 / UDP 15152
/usr/bin/python3 tools/verify_vr_rviz.py
```

位置 full 的接口流程验证使用两个终端：

```bash
# 终端一：无效 CAN 路径 + 隔离 domain 86
bash scripts/run_vr_position.bash mode:=mock stage:=follow \
  motion_range:=full use_rviz:=false

# 终端二：先加载环境与 install，再运行验证
source scripts/environment.bash
a6y_load_install
export ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1
/usr/bin/python3 tools/verify_vr_real.py --mock --controller position \
  --exercise-full-range --output validation/vr_position/mock-new.json
```

阻抗 mock 用 `run_vr_real.bash mode:=mock use_rviz:=false`，验证工具改用 `--controller impedance` 并去掉 `--exercise-full-range`。网关 HTTPS/WSS 测试需单独启动回环实例，见[虚拟入口的软件检查](虚拟机械臂/VR_RVIZ_USAGE.md#软件检查命令)。完整工程回归见[离线测试指南](../how-to/run-offline-tests.md)。

| 现象 | 先检查 |
| --- | --- |
| Quest 网页可打开，但不能进入 XR | HTTPS 证书信任、地址中的主机 IP、是否进入沉浸模式 |
| 有网页却没有手柄输入 | 网关 `right events`、节点 `pose_age`、真实 / 虚拟 profile 与 UDP 端口是否配套 |
| ARM 被拒绝 | 是否 readonly / hold 模式，PREPARE 渐增是否完成，输入是否新鲜，侧握把是否释放 |
| launch 拒绝启动 | 设备稳定路径、旧 `/arm6/controller_manager`、串口占用和配置预检缺项 |
| 跟随变慢或短暂停住 | 查看跟踪偏差、`TRACKING_LIMITED` / IK 状态、原始输入时间戳；topic 频率不能替代机械时延 |
| 记录不能离线复核 | 模型 / 限位哈希和依赖是否变化，文件是否完整，路径是否仍能定位模型 |

## 当前边界与进一步阅读

虚拟测试证明协议、映射、IK 与状态机行为；mock 证明软件接口和许可流程，都不替代实际 Quest 追踪、负载、碰撞间距、完整工作空间和运动停止距离的现场验证。历史 J1 单轴实验或支撑下保持记录也不能证明整套 VR 动态跟随已经验收。

full 位置模式不运行 CAD 碰撞检查，也没有环境避障；位置闭环的电流 / 力矩限制由电机固件决定，不能套用阻抗或示教夹爪的 N·m 上限。配置中的频率与速度均为软件目标或上限，不能据此宣称硬实时、端到端延迟或最大负载能力。

- [虚拟模型使用、配置与诊断](虚拟机械臂/VR_RVIZ_USAGE.md)
- [实机各模式及历史调试记录](实机控制/VR_REAL_ROBOT_USAGE.md)
- [虚拟方案设计](虚拟机械臂/VR_RVIZ_PLAN.md) · [实机方案设计](实机控制/VR_REAL_ROBOT_PLAN.md)
- [参考方案分析](参考方案/VR_REFERENCE_ANALYSIS.md) · [Piper 控制路径比较](参考方案/VR_PIPER_CONTROL_COMPARISON.md)
- [VR 源码](../../src/a6_y_vr_teleop/) · [网关源码](../../vr_gateway/) · [工具脚本](../../tools/)
