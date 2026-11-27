# A5 虚拟机械臂 VR 使用说明

当前入口总览、环境准备和三种模式的操作顺序见[VR 遥操作总入口](../README.md)。本页保留带日期的实现与调试记录；历史 `validation/` 文件未随本次源码发布，相应位置保留本地记录路径，不作为公开可下载的验证证据。下列命令均在仓库的 `A6-Y-G/` 目录执行。

本入口使用 Quest 右手柄控制 RViz 中的 A5 运动学模型。ROS domain 默认为 84，命名空间为 `/arm6_vr`；没有 ros2_control、串口或 CAN 输出。真实机械臂仍由原有入口控制。

## 当前实现与验证

已实现浏览器输入网关、六维相对位姿映射、A5 Pinocchio IK、关节与夹爪限速、握把重抓、断流保持、会话/乱序校验、RViz 目标显示、输入录制及回放。

2026-10-08 的软件验证结果：

- 23 项测试通过，覆盖三个平移轴和三个旋转轴、六轴可达目标、限速、夹爪、重抓、过期、乱序、跨 ±π 旋转、错误输入及实时回放的时间基准。
- 模拟浏览器通过 HTTPS/WSS 向真实 Vuer 服务发送事件，列主序解码及四次握把状态变化通过。该测试不是实际 Quest 浏览器验证。
- 独立 domain 85 集成测试运行 60 秒，收到 7500 条虚拟 JointState，实测约 124.998 Hz；消息时间戳间隔 P99 为 8.772 ms、最大 10.221 ms。稀疏诊断中的 IK/状态步进耗时 P99 为 1.184 ms，完整回调最大耗时 5.089 ms；测试中未触发状态机的周期或求解预算保护。
- 七个独立坐标都有正确输出，手指 mimic 与 TCP TF 均存在，断流后保持姿态。7568 个录制状态离线重算，关节差为 0。
- 已目视检查 RViz 模型和显示配置。实际 Quest 输入频率、浏览器追踪有效性及手柄操作方向仍需头显联调，不能由模拟客户端测试代替。

报告位于 核心测试（本地历史文件：`validation/vr_rviz/unit_tests.xml`）、网关验证（本地历史文件：`validation/vr_rviz/gateway_check.json`）、ROS 集成验证（本地历史文件：`validation/vr_rviz/integration_check.json`）、回放验证（本地历史文件：`validation/vr_rviz/replay_check.json`）及 RViz 截图（本地历史文件：`validation/vr_rviz/rviz_home.png`）。

## 启动

从 `A6-Y-G/` 目录执行。以下为已有依赖环境中的更新命令；新克隆先按 [VR 统一入口](../README.md#准备构建与网关)构建主工程并安装网关环境：

```bash
source scripts/environment.bash
colcon build --base-paths src --packages-select a6_y_vr_teleop --symlink-install
```

终端一启动虚拟机械臂和 RViz：

```bash
./scripts/run_vr_rviz.bash
```

终端二启动 Quest 网关：

```bash
./scripts/run_vr_gateway.bash
```

Quest 和主机连接同一局域网，浏览器访问 `https://<主机局域网IP>:8012`，完成本地开发证书信任操作后进入沉浸式 XR。主机地址改变后，需要重新生成包含新 IP 的证书。

网页用于手柄输入，完整机械臂显示在电脑的 RViz 中，不传输电脑画面或摄像头图像。右手柄松开握把时应显示 HOLD，按住后显示 TELEOP。终端中的 `right events` 和 `pose_age` 可用于确认原始数据是否持续到达。首次连接或断连恢复，必须先松开握把，再按下；直接保持按下不会启动跟随。

同一个 UDP 输入端口只接受一个活跃输入会话。测试回放与 Quest 不能同时驱动；停止一个输入源，等待至少 0.15 秒，再接入另一个。

## 操作与显示

| 操作或显示 | 含义 |
| --- | --- |
| 按住右握把 | 从当前虚拟 TCP 建立参考，相对跟随手柄 |
| 松开右握把 | 当帧保持当前六轴和夹爪状态 |
| TELEOP 时扣扳机 | 越扣越闭合；松扳机张开，握把释放时夹爪也冻结 |
| HOLD 时 A 按钮 | 保持当前状态，下一次握把重新建参考；不自动回零 |
| WAITING | 尚未收到可以建立握把状态的有效输入 |
| HOLD | 已有有效输入，等待按住握把 |
| TELEOP | 正在跟随 |
| PAUSED | 输入超时、追踪突跳、求解或周期异常；检查原因后松开再握住 |
| 橙红 / 黄色 / 绿色箭头 | 原始映射目标 / 受限目标 / 实际虚拟 TCP；重合时会互相遮挡 |

默认轴映射为 VR 右/上/后 → A5 的负 Y/正 Z/负 X。平移比例 0.5，手柄移动 10 cm 对应 TCP 目标移动 5 cm；旋转比例为 1。实际面对机械臂的方向可能需要调整配置中的 `vr_to_base`，先分别检查三个平移方向和三个旋转方向。

HOLD 是直接冻结虚拟 q，未计算力矩，也不模拟受力保持。碰撞检测、动力学和真机执行均不属于这个入口。初始 q 已检查关节限位及局部雅可比，未作为实机安全姿态认证。

## 配置与诊断

参数文件为 [a5_vr.yaml](../../../src/a6_y_vr_teleop/config/a5_vr.yaml)，修改后重启虚拟入口。它与原来的三份控制器 YAML 独立。

| 参数 | 默认值 |
| --- | --- |
| `update_rate` | 125 Hz，虚拟节点目标频率 |
| `input_timeout` | 0.15 s，原始右手位姿和按键均需新鲜 |
| `position_scale` | 0.5 |
| `max_velocity` | 六轴 0.5 rad/s，夹爪 0.1 m/s；与运行限制取较小值 |
| `target_linear_speed` | 0.15 m/s |
| `target_angular_speed` | 0.8 rad/s |
| `compute_budget` | 0.004 s，状态步进计算预算；超出后保持并要求重新握把 |
| `max_control_period` | 0.04 s，超长周期不补做大步进 |
| `udp_port` | 本机 15150；网关对应 `--udp-port` |

诊断终端必须进入同一 domain：

```bash
source scripts/environment.bash
a6y_load_install
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
ros2 topic echo /arm6_vr/status
# 另一个同域终端可查看发布频率
ros2 topic hz /arm6_vr/joint_states
```

topic 频率不等于头显原始输入频率；状态中的 `observed_input_events_hz` 是节点收到输入报文的累计平均率，包含重复/被拒绝报文，须结合 accepted、rejected 和 pose_age 判断。`compute_ms` 只计状态步进，`callback_max_ms` 包含本地读取和 ROS 发布等处理，均不代表头显到显示的总延迟。

## 无头显演示与回放

先启动虚拟 RViz，再运行：

```bash
source scripts/environment.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
/usr/bin/python3 tools/vr_replay.py --demo --duration 16
```

演示只发送本机虚拟输入，结束后节点因超时进入 PAUSED。录制时给启动入口传入一个尚不存在的文件路径：

```bash
./scripts/run_vr_rviz.bash record_file:="$PWD/validation/vr_rviz/my_session.jsonl"
```

录制包含配置、模型与限位文件 SHA-256、计算依赖版本、输入包、实际接收时间、每次虚拟步进的 dt、q 和状态原因。离线复核会检查模型与限位文件是否变化。重新播放和离线逐步复核：

```bash
# 向已运行的虚拟节点发送输入，重建会话和主机单调时间
/usr/bin/python3 tools/vr_replay.py --replay validation/vr_rviz/my_session.jsonl

# 在 source ROS 环境后，不启动节点、不使用网络，按录制时间步重算
/usr/bin/python3 tools/verify_vr_replay.py validation/vr_rviz/my_session.jsonl
```

实时回放受 ROS 调度影响，不要求逐位一致；离线复核使用相同输入和 dt，运行时预算保护按记录重现。对于异常网络报文，应先查看录制中的拒绝原因，不能把损坏输入当正常演示。

## 重建网关环境与证书

网关依赖固定在 [requirements.lock](../../../vr_gateway/requirements.lock)，不安装到系统 ROS 的 Python 中。当前主机安装了 `uv`，可在未 source ROS 的普通终端重建：

```bash
/usr/bin/python3 -m venv --without-pip .venv-vr
uv pip sync --python .venv-vr/bin/python vr_gateway/requirements.lock
```

若 `uv` 不在 PATH，先按它的安装位置配置 PATH；也可使用系统 `python3-venv` 创建常规虚拟环境，再执行 `pip install -r vr_gateway/requirements.lock`。

首次证书生成：

```bash
read -r -p "主机局域网 IPv4 地址: " A6Y_VR_IP
bash scripts/create_vr_certificate.bash "$A6Y_VR_IP"
```

脚本不会覆盖已有证书。IP 改变时先把 `validation/vr_rviz/tls/` 下旧证书另存，再生成新证书。私钥不进入源码管理，也不放入网关公开静态目录。浏览器若无法进入 XR，先检查证书信任、HTTPS/WSS 地址和是否已进入沉浸式模式；本地 HTTPS 响应成功不等于头显已开启追踪。

## 软件检查命令

```bash
source scripts/environment.bash
a6y_load_install
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$PWD/src/a6_y_vr_teleop:$PYTHONPATH"
/usr/bin/python3 -m pytest src/a6_y_vr_teleop/test -q

# 自动启动并关闭自己的虚拟 graph，使用 domain 85 / UDP 15152
/usr/bin/python3 tools/verify_vr_rviz.py
```

网关独立测试需要先开回环测试实例：

```bash
./scripts/run_vr_gateway.bash --host 127.0.0.1 --port 8013 --udp-port 15151
# 另一个终端
.venv-vr/bin/python tools/verify_vr_gateway.py
```

按 Ctrl+C 退出对应终端即可停止虚拟节点或网关，不发送电机停机、使能或回零指令。
