# 四臂只读反馈与 RViz 零位检查

此步骤用于将本次电机零位标定后的真实姿态，与提供的 `src/A5/urdf/A5.urdf` 对照。
显示链路为状态查询 → 电机角度换算 → `joint_states` → `robot_state_publisher` → RViz。
不需要构建项目，也不依赖 Pinocchio。原有扫描／零位标定窗口仍用 `run_arm_scan.bash` 打开。

## 启动本地选择窗口

在本机图形桌面的终端运行：

```bash
bash '/home/taoqiu/文档/ChatGPT/A6Y远程遥操作/A6-Y-G/scripts/run_arm_rviz.bash'
```

1. 选择 **运动端左臂、运动端右臂、示教端左臂、示教端右臂** 中的一条。默认不选中任何臂。
2. 保持“实时只读反馈 + RViz”，点击“启动”。窗口显示电机原始角度及换算后的模型位置，RViz 显示当前姿态。
3. 观察实物和模型的基座、J1～J6、夹爪零姿态及关节安装位置。黄色箭头是 URDF 关节正轴，RGB 坐标轴分别是 X/Y/Z。
4. 按需要在已支撑、可手动活动的状态下逐关节小幅摆动，核对对应关节、转动方向和角度比例；本程序不提供运动按钮。
5. 不一致时点击“停止显示并释放串口”，等待停止后回到上一步标定。停止后重新选择机械臂，才可开始下一次查看。

切换到“仅预览 URDF 零姿态（无硬件）”，可以先查看 q=0 的模型。预览明确显示“无实测”，电机读数为空，不代表实机标定通过。
打开窗口、选臂、刷新 USB 状态均不打开串口；只在点击实时启动后连接选定适配器。
选择窗口只检查 Linux 的设备信息，不使用浏览器或网络服务。

## 读数与模型约定

| 角色 | 参数值 | USB 序列号 |
| --- | --- | --- |
| 运动端左臂 | `motion_left` | `304C33733036` |
| 运动端右臂 | `motion_right` | `209B3971534B` |
| 示教端左臂 | `teaching_left` | `304F33733036` |
| 示教端右臂 | `teaching_right` | `209D3982534B` |

- 身份来源是项目旁的 `diagnostics/arm_roles.yaml`。连接前按 USB 序列号再次核验，使用稳定 by-id 路径，不回退到历史 ttyACM 编号。
- 电机位置使用前一步记录的、按 USB 序列号和 ID 绑定的 PMAX。运动左臂夹爪约 3.14 rad，其余三台约 12.566 rad；不会混用。
- 显示换算在本目录的 `joint_mapping.yaml`：`q_urdf = sign × position_scale × q_motor_rad + offset`。J1～J6 默认方向为 `[-1, +1, +1, -1, -1, -1]`，偏移为 0；夹爪比例 0.0239 m/rad。这些沿用原程序，**尚未核对本次四臂安装**。
- 各臂的 `overrides` 可单独存放之后确认的映射差异，配置与 USB 序列号绑定。显示窗口不写入映射、URDF 或电机零位。
- 四臂暂时共用现有 A5 URDF，均以自己的 `base_link` 为基准。此页面不表现四臂在桌面中的相对安装位置，也不自动镜像左右臂。若实物结构有差异，应核实对应 URDF，而非靠电机零位掩盖。
- J1～J6 的模型坐标内部为 rad，表格显示角度；夹爪模型坐标内部为 m，表格显示单指位移 mm，不是实测开口宽度。另一手指由原 URDF 的 mimic 规则联动。
- 不减去启动时位置、不自动消除零偏、不滤波、不回绕、不截断超限角度。URDF 范围越界只标记，保留实际反馈；越界判断容许一个位置量化步长，避免电机零附近的量化值误报。
- 悬停表格可看 URDF 关节原点、轴、限制、采样时间、原始计数和 PMAX。

## 只读边界与失联处理

通信复用现有 `StatusOnlyControl`，发送层只允许 CAN `0x7FF / <ID> 00 CC 00 00 00 00 00`，ID 限 1～7。
打开／关闭串口时只有适配器自身的 SLCAN 初始化／关闭操作；不发送电机使能、失能、运动、写零或参数写入命令。
显示入口只启动反馈节点、`robot_state_publisher` 和 RViz，不加载控制器。

默认目标为每秒 20 轮，单电机等待最多 40 ms；实际速率取决于串口响应，不保证同步采样。
只有完整一轮的七个位置都新鲜（最长 400 ms）时才发布整臂 `joint_states`。
缺失时清空该次缺失读数、暂停模型姿态更新，RViz 红字显示 `FEEDBACK LOST / POSE FROZEN`，
旧姿态可能仍留在 RViz 中，它不是当前真机姿态。通信恢复后自动继续显示。
电机异常或已使能状态会提示，位置仍然保留；程序不会尝试清错或改变使能。

与前一步扫描／标定共用全局任务锁和串口锁。实时显示期间不能进行零位标定。
关闭任一显示会话的 RViz 或点击停止，会停止查询并关闭该会话的显示进程；关闭选择窗口也会等待清理。
进程只清理自己启动的子进程，不批量结束其他 ROS 程序。

ROS 默认使用本机 domain 194，可通过 `A6Y_REVIEW_DOMAIN_ID` 修改。
每次启动使用独立的 `/a6y_check/<角色>/s<随机值>` 命名空间，`joint_states`、`robot_description`、`tf`、`tf_static`、`markers` 均隔离。
模型直接来自当前源码，网格路径转换成当前工作区的绝对 file URI；不会引用旧 install 中的模型。
启动时会检查模型节点是否接入关节反馈；DDS 通信异常时报告错误。

每次结束记录到 `diagnostics/arm_rviz_reports/<角色>_<唯一目录>/session.json`，包含身份、URDF 哈希、显示映射、最后一轮读数、发送总数和最后 70 条查询。
同目录保存 ROS 参数快照、RViz 配置和子进程日志。这是检查记录，不是连续轨迹或标定通过证明。

## 命令行选择与检查

从 `A6-Y-G` 目录运行：

```bash
# 终端数字菜单选择运动／示教、左／右臂，然后打开 RViz
bash scripts/run_arm_rviz.bash --cli

# 指定机械臂
bash scripts/run_arm_rviz.bash --cli --arm teaching_left

# 不连接硬件，只看模型零姿态
bash scripts/run_arm_rviz.bash --cli --arm motion_left --preview

# 无图形桌面时只发布真实反馈与 TF，Ctrl+C 停止
bash scripts/run_arm_rviz.bash --cli --arm motion_right --no-rviz

# 只核对四臂配置，不连接串口、不启动 ROS
bash scripts/run_arm_rviz.bash --dry-run
```

`--duration 10` 可用于 CLI 在 10 秒后停止。图形窗口的 `--arm` 只预选目标，不自动连接。
`--dry-run` 是配置检查；`--preview` 是明确的模型演示，两者都不访问电机。
本机需要 ROS 2 Humble 的 `rclpy`、`sensor_msgs`、`visualization_msgs`、`robot_state_publisher`、`rviz2`，以及现有的 PyQt5、python-can、pyserial、PyYAML 和 fuser。

## 无硬件验证

```bash
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -s -m unittest discover \
  -s tools/arm_visualization -p 'test_*.py' -v

# 需要本机 ROS DDS 通信；只发布合成姿态，不连接串口、不打开 RViz
source scripts/environment.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=195 ROS_LOG_DIR=/tmp/a6y-rviz-verification-logs
/usr/bin/python3 tools/arm_visualization/verify_ros_display.py
```

测试覆盖四臂映射与 PMAX、方向/比例换算、禁止自动消零与限幅、身份不符、非法配置、缺失/过期反馈、CAN 发送白名单、PTY 实际 SLCAN 通信、任务互斥、停止/断连释放资源，以及离屏选择窗口。
ROS 验证实际订阅 `joint_states`、机器人描述和九个 TF，检查夹爪 mimic 和失联不补零。
自动测试不能判断真实安装与零点是否正确；这需要在本机桌面打开 RViz 后与实物对照。

ROS 模型发布机制参考 [robot_state_publisher 官方说明](https://github.com/ros/robot_state_publisher/tree/humble)。
