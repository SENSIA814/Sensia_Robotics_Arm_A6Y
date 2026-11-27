# 第一次运行：配置检查与离线界面

本教程面向第一次接触本项目的机器人开发和操作人员。完成后，你会检查四臂配置，并打开默认离线动力示教界面；整个过程不需要连接机械臂。

## 1. 准备工作目录

开始前需要 Ubuntu 22.04、已安装的 ROS 2 Humble，以及项目依赖。ROS 位于 `/opt/ros/humble`；Pinocchio 需要 **≥ 4.0 的 C++ 库及匹配的系统 Python 绑定**。ROS 安装完成后的依赖安装命令见[依赖准备入口](../reference/runtime-and-configuration.md#已安装-ros-2-humble-后的依赖准备)。本教程从依赖已准备好的开发环境开始。

在新终端获取代码：

```bash
git clone https://github.com/SENSIA814/Sensia_Robotics_Arm_A6Y.git
cd Sensia_Robotics_Arm_A6Y
```

后续命令均从这个仓库根目录执行。保留 `A6-Y-G/` 与 `diagnostics/` 的相邻关系；使用纯英文克隆路径可避开部分 Humble 构建工具的路径问题。

## 2. 核对运行环境

```bash
test -f /opt/ros/humble/setup.bash
source A6-Y-G/scripts/environment.bash
/usr/bin/python3 - <<'PY'
import sys
import pinocchio
import yaml, serial, can, numpy, scipy
from PyQt5 import QtWidgets
version = tuple(int(x) for x in pinocchio.__version__.split('.')[:2])
assert version >= (4, 0), '本项目需要 Pinocchio >= 4.0'
print('Python:', sys.version.split()[0])
print('Pinocchio:', pinocchio.__version__)
print('Python 依赖检查通过')
PY
ros2 pkg prefix controller_manager
ros2 pkg prefix position_controllers
ros2 pkg prefix joint_trajectory_controller
ros2 pkg prefix rmw_cyclonedds_cpp
```

你应看到依赖检查通过及各 ROS 包的安装路径。环境脚本会清理旧 ROS overlay、Conda 和用户 Python 路径，因此在这个专用终端继续操作。

若缺少模块或 ROS 包，先按[依赖参考](../reference/runtime-and-configuration.md#环境要求)补齐再继续。若 Pinocchio 低于 4.0，当前环境不满足[控制器的 CMake 要求](../../src/a6_y_controllers/CMakeLists.txt)；不能只通过安装一个 Python 包替代 C++ 依赖，也不能假定软件源的 `ros-humble-pinocchio` 一定提供所需版本。下面的构建检查还会验证 C++ 侧依赖。

## 3. 检查四臂角色

```bash
bash A6-Y-G/scripts/run_arm_scan.bash --cli --arms all --dry-run
```

输出应列出四个角色及其 USB 序列号，最后显示 `DRY RUN：没有打开串口`。这些是仓库保存的本套设备配置；此步骤没有查询电脑上实际插着哪些设备。

## 4. 检查动力示教配置

```bash
bash A6-Y-G/scripts/run_arm_teaching.bash --check
```

首次执行会构建所需 ROS 包，后续按源码内容复用或更新缓存。成功时末尾显示：

```text
动力示教四臂模型、身份、默认配对、夹爪端点及速度参数检查通过；没有连接硬件。
```

这表示模型、角色关联和运行参数可加载；不表示实体机械臂已经完成标定。

## 5. 打开离线界面

在图形桌面终端运行：

```bash
bash A6-Y-G/scripts/run_arm_teaching.bash
```

确认窗口中模式是 **“离线模拟”**，两组分别为示教左→运动左、示教右→运动右。你应看到配对、四臂状态、夹爪开度和日志区域。此时尚未开始会话，关闭窗口即可结束本教程。

你已经完成第一次配置检查和界面启动。需要操作实体设备时，继续阅读[运行动力示教](../how-to/run-power-teaching.md)；需要验证软件链路时，阅读[运行离线测试](../how-to/run-offline-tests.md)。
