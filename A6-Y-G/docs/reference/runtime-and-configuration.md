# 运行与配置参考

本文供查询依赖、入口、角色配置及当前示教参数。入门步骤见[离线教程](../tutorials/first-offline-run.md)，实体操作见[动力示教指南](../how-to/run-power-teaching.md)。路径以仓库根目录为基准。

## 环境要求

| 项目 | 要求 |
| --- | --- |
| 系统 | Ubuntu 22.04；当前开发基线为 x86_64 |
| ROS | ROS 2 Humble，`/opt/ros/humble/setup.bash` |
| Python | 系统 `/usr/bin/python3`，Ubuntu 22.04 基线为 3.10 |
| 构建 | C++ 编译器、CMake、colcon、rosdep；RViz 面板另需 Qt5 Widgets |
| 控制与显示 | ros2_control、ros2_controllers、RViz2、xacro、robot_state_publisher；MoveIt 入口另需 MoveIt 2 |
| Pinocchio | C++ 库和系统 Python 绑定均需 ≥ 4.0；控制器使用原生 mimic 支持 |
| Python 模块 | PyQt5、PySerial、python-can、PyYAML、NumPy、SciPy；曲线分析另需 Matplotlib、Pandas |
| DDS | 重力补偿／示教默认 `rmw_cyclonedds_cpp`，通信限制本机 |
| 真机接口 | 每臂独立 CANable2／SLCAN，总线 1 Mbps；串口配置 115200 baud |
| 图形界面 | Qt 窗口和 RViz 需要图形桌面；配置检查与离线测试可用无头环境 |

依赖声明位于各包的 `package.xml`；C++ 的 Pinocchio 最低版本由[控制器 CMake](../../src/a6_y_controllers/CMakeLists.txt)校验。软件源提供的版本取决于机器配置；`ros-humble-pinocchio` 这一包名本身不保证满足 ≥ 4.0。Python 导入成功也不能替代 C++ 构建检查。

环境入口 [`environment.bash`](../../scripts/environment.bash)会清理旧 overlay、Conda、用户 Python 路径并加载系统 ROS。VR 网关使用独立环境，不能把其依赖混入 ROS 系统 Python。

### 已安装 ROS 2 Humble 后的依赖准备

下面命令使用已配置的 Ubuntu／ROS 软件源，不负责安装 ROS 本身。先查看 Pinocchio 候选版本：

```bash
sudo apt-get update
apt-cache policy ros-humble-pinocchio
```

候选版本不存在或低于 4.0 时，**停在这里**，先由环境维护者准备满足要求的 C++ 库及匹配 Python 绑定。不要用 pip 安装一个同名模块替代 C++ 依赖，也不要降低本项目的 CMake 版本要求。候选版本满足要求时，公共依赖安装入口为：

```bash
sudo apt-get install -y \
  build-essential cmake pkg-config git psmisc qtbase5-dev \
  python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-can python3-serial python3-yaml python3-numpy python3-scipy \
  python3-pyqt5 python3-matplotlib python3-pandas python3-venv fonts-noto-cjk \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-moveit \
  ros-humble-rviz2 ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rmw-cyclonedds-cpp ros-humble-pinocchio

# 仅尚未初始化 rosdep 的机器执行一次：sudo rosdep init
rosdep update
source A6-Y-G/scripts/environment.bash
rosdep install --from-paths A6-Y-G/src \
  A6-Y-G/tools/arm_moveit/ros2_stack/src A6-Y-G/tools/arm_moveit/rviz_panel \
  --ignore-src --rosdistro humble --skip-keys ament_python -y
```

本项目现有 Python 包声明中的 `ament_python` 在原环境的 rosdep 数据库中无法解析，因此跳过这个键，由 colcon 提供相应构建支持；其余依赖交给 rosdep。此处只解析两套源码的依赖，不把同名驱动放进同一次构建。

安装后用系统 Python 与独立 CMake 配置分别核验版本：

```bash
source A6-Y-G/scripts/environment.bash
/usr/bin/python3 - <<'PYTHON'
import pinocchio
assert tuple(int(x) for x in pinocchio.__version__.split('.')[:2]) >= (4, 0)
print('Python Pinocchio:', pinocchio.__version__)
PYTHON
A6Y_PIN_CHECK=$(mktemp -d /tmp/a6y-pinocchio-check.XXXXXX)
cat > "$A6Y_PIN_CHECK/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(a6y_pinocchio_check LANGUAGES CXX)
find_package(pinocchio 4.0 REQUIRED)
message(STATUS "C++ Pinocchio: ${pinocchio_VERSION}")
CMAKE
cmake -S "$A6Y_PIN_CHECK" -B "$A6Y_PIN_CHECK/build"
```

任一检查失败时先处理依赖，停止后续启动步骤。两侧版本应匹配；最终还需通过教程中的项目 `--check` 构建与配置检查。

## 启动入口

以下命令均从仓库根目录执行。打开窗口本身不会自动开始真机会话。

| 功能 | 命令 | 默认／用途 |
| --- | --- | --- |
| 扫描 | `bash A6-Y-G/scripts/run_arm_scan.bash` | 图形诊断；CLI `--dry-run` 只列配置 |
| 只读模型 | `bash A6-Y-G/scripts/run_arm_rviz.bash` | 按角色显示反馈与 RViz |
| 动力示教 | `bash A6-Y-G/scripts/run_arm_teaching.bash` | 默认离线模拟；`--check` 检查配置 |
| 独立重力补偿 | `bash A6-Y-G/scripts/run_arm_gravity.bash` | 六轴补偿、J7 只读；`--check` 检查配置 |
| MoveIt | `bash A6-Y-G/scripts/run_arm_moveit.bash` | 默认模拟；`--check-config` 检查配置 |
| VR 虚拟模型 | `bash A6-Y-G/scripts/run_vr_rviz.bash` | 无实体串口的运动学模型 |

重力补偿／示教启动器自动构建到 `/tmp/a6y-gravity-build-<uid>/`，MoveIt 另有独立构建缓存。两套源码含同名 `a6_y_ros2_control`，不能混入同一次 colcon 构建。示教默认 ROS domain 197，可用 `A6Y_GRAVITY_DOMAIN_ID` 覆盖；测试入口使用自己的固定 domain。

## 角色与身份关联

角色为 `motion_left`、`motion_right`、`teaching_left`、`teaching_right`。身份由 USB 序列号和完整 by-id 路径关联，不能依靠可变化的 `ttyACM` 编号。

| 文件 | 内容 |
| --- | --- |
| [`diagnostics/arm_roles.yaml`](../../../diagnostics/arm_roles.yaml) | 四臂角色、USB 序列号、串口稳定路径 |
| [`joint_mapping.yaml`](../../tools/arm_visualization/joint_mapping.yaml) | 各角色身份，电机到模型的方向、比例和零偏 |
| [`motor_position_limits.yaml`](../../tools/arm_diagnostics/motor_position_limits.yaml) | 按序列号与电机 ID 保存的 PMAX 缓存 |
| [`motor_limits.json`](../../tools/arm_gravity/motor_limits.json) | ID 与 PMAX／VMAX／TMAX 等协议参考参数 |
| [`profiles/`](../../tools/arm_gravity/profiles/) | 按角色／USB 保存的重力补偿参数 |
| [`teaching_gripper_limits.yaml`](../../tools/arm_gravity/teaching_gripper_limits.yaml) | 四臂 J7 实测角度端点、内收余量、身份及 PMAX |

这些是本套设备的运行配置。夹爪原始采样和现场日志保存在本地，没有作为公开数据附件提供；运行 YAML 不能代替原始采样记录。换板流程见[USB2CAN 更换指南](../how-to/replace-usb-can-adapter.md)。

每臂电机指令 ID 为 1～7，反馈 ID 为 17～23。J1～J3 对应 DM4340P，J4～J6 对应 DM4310，J7 对应 DM3507；具体硬件需与配置一致。PMAX／VMAX／TMAX 是协议编码范围，不等于机械行程、允许运动速度或物体夹持力上限。

## 动力示教参数

| 项目 | 当前值／行为 | 实现位置 |
| --- | --- | --- |
| 目标发布 | 100 Hz | `teaching_port_config.py` |
| 原生循环 | 250 Hz | `teaching_port_config.py` |
| CAN 速率 | 1 Mbps | `teaching_port_config.py` |
| 六轴软启动 | 1.2 s，绝对角度跟随 | `teaching_port_config.py` |
| 示教 J1～J6 | MIT 重力补偿 | 各臂 profile 与控制器 |
| 示教 J7 | MIT Kp=0、Kd=0，软件力矩通道 | `teaching_port_config.py` |
| 运动 J1～J7 | POS_VEL | `teaching_port_session.py` |
| 夹爪目标 | 各臂内收范围内的 0～100% 映射 | `teaching_gripper.py` |
| 端点内收 | 每端 0.029288703 motor rad，按现有比例约为单指 0.7 mm | `teaching_gripper_limits.yaml` |
| 夹爪限速 | 0.100 m/s，模型单指位移 | `teaching_gripper.py` |
| 空载阻尼 | 0.008 N·m/(motor rad/s)，速度死区 0.04 motor rad/s | `gripper_haptics.py` |
| 空载阻尼上限 | 0.015 N·m | `gripper_haptics.py` |
| 示教 J7 总力矩上限 | 2.0 N·m，接触反馈与阻尼合计 | `gripper_haptics.py` |
| 接触反馈 | 增益 0.5，虚拟刚度 18 N·m/m，变化率 0.8 N·m/s | `gripper_haptics.py` |
| 受阻接触条件 | 正向力矩 ≥0.12 N·m、双目标闭合误差 ≥0.25 mm、速度 ≤1.5 mm/s，持续 120 ms | `gripper_haptics.py` |
| 持续负载条件 | 正向力矩 ≥0.25 N·m，目标没有明显落后实际位置，持续 60 ms | `gripper_haptics.py` |
| 释放位移 | 映射单指回退 0.3 mm | `gripper_haptics.py` |
| J7 力反馈时效 | 80 ms；反馈异常锁存，底层力矩／门控超时清零 | `gripper_haptics_ros.py`、C++ 控制器 |

接触条件还要求没有明确张开。速度、位移的 mm/m 均为模型坐标，不是独立测量的实际指间距。当前端点定义普通跟随目标；启动、反馈与退出保持允许协议编码范围内的实测位置，不再以超出实测端点直接拒绝启动。

示教 J7 上限由生成配置注入并保留一个 MIT 编码步长余量；独立重力 profile 的 J7 项仍为零。不能通过把独立 profile 的 J7 力矩项改为 2 来配置示教反馈。

## 单位与记录

J1～J6 位置为 rad，J7 `gripper_finger_joint1` 为单指位移 m。当前 J7 比例为 0.0239 m/rad，正方向闭合。`effort_j7` 为移动关节广义力，电机力矩需按方向与比例换算。

会话保存于 `diagnostics/arm_teaching_reports/`，启动输出位于 `diagnostics/arm_teaching_logs/`。会话中的 `session.json`、实际生成 YAML 和源码哈希用于判断本轮加载了什么；日志字段与复核步骤见[日志指南](../how-to/inspect-teaching-logs.md)。软件配置频率不等于电机内部伺服频率或固定端到端延迟。
