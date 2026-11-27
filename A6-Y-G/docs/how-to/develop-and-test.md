# 构建底层控制器并运行离线回归

适用于修改模型、驱动、阻抗或单关节实验的开发者。先准备[运行依赖](../reference/runtime-and-configuration.md)，从仓库根目录开始。单关节工具和阻抗验证脚本使用 `A6-Y-G/build`、`A6-Y-G/install`，与桌面程序的 `/tmp` 缓存分开。

## 1. 构建主工程

```bash
cd A6-Y-G
source scripts/environment.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths src \
  --packages-up-to a6_y_controllers \
  --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
a6y_load_install
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
```

后续本页命令以 `A6-Y-G/` 为当前目录。`--base-paths src` 排除 MoveIt 独立栈的同名驱动；`--packages-up-to` 构建控制器及其本地依赖。开发 VR 节点还需按 [VR 文档](../VR遥操作/实机控制/VR_REAL_ROBOT_USAGE.md)构建对应包。用 `ros2 pkg prefix a6_y_controllers` 核对加载来源。

环境脚本会清理旧 overlay、Conda 和用户 Python 路径。构建失败时检查本次输出，不用其他安装目录的旧二进制代替验证。

## 2. 检查协议和两种阻抗

```bash
ctest --test-dir build/a6_y_ros2_control --output-on-failure
ctest --test-dir build/a6_y_controllers --output-on-failure
/usr/bin/python3 tools/verify_impedance_dry_run.py
/usr/bin/python3 tools/verify_joint_impedance_dry_run.py
```

按顺序执行。CTest 既有 C++ 也有 Python／PTY 检查。两个阻抗脚本固定 `dry_run=true`、`auto_enable=false` 和不存在的实体串口，核对插件、参数继承、目标捕获和七轴状态，输出在 `validation/`；它们不是手推动态实验。

需要模型回位验证时运行：

```bash
/usr/bin/python3 tools/verify_joint_impedance_return.py
```

它调用 `build/a6_y_controllers/test_a5_joint_impedance_controller`，用模拟接口及假设摩擦／惯量，输出到 `validation/joint_spring_strengthening/`。必须先完成本地测试构建，仅有桌面 `/tmp` 缓存不够。模型释放收敛不证明实机内环、摩擦或碰撞条件。

## 3. 选择单关节实验

`mock` 不打开实体 CAN，`real` 是会使能并运动的实验，不能当作扫描自检。所有脚本默认 mock，示例仍显式指定：

```bash
bash scripts/run_j1_position_test.bash --mode mock --degrees 10 --move-seconds 12
```

| 实验 | 入口 | 目标与接管范围 |
| --- | --- | --- |
| J1 位置往返 | `run_j1_position_test.bash` | 相对起点运动后返回；默认仅 J1 可控，其余只读 |
| J2 位置往返 | `run_j2_position_test.bash` | `--start-degrees`／`--end-degrees`；接管六轴保持其他轴 |
| J3 位置往返 | `run_j3_position_test.bash` | `--start-degrees`／`--end-degrees`；接管六轴保持其他轴 |
| J1～J4 高速轨迹 | `run_j1_highspeed_test.bash` 至 `run_j4_highspeed_test.bash` | 必须 `--anchor-json`；接管六轴，测试指定轴，夹爪只读 |

高速实验要求与机构、零位相符的锚点，不提供通用锚点或自动复用现场文件。先看相应 `tools/test_j*_highspeed.py --help`，核对角度、速度及超时；`--return-home-only` 回到实验锚点，不等于六轴数值零。J4 另有 `--hold-seconds`。

真机实验须停止占用串口的其他会话，使用核对过的 by-id 路径传 `--can-channel`，并确认该脚本接管范围。脚本检查不代替机械路径确认；查看报告的 `passed`、`disarm_confirmed` 和控制器退出结果，中断不代表返回或失能完成。

报告位于 `validation/j*_position/`、`validation/j*_highspeed/`。分析工具包括 `analyze_j1_tuning.py` 至 `analyze_j3_tuning.py`、`analyze_j1_highspeed.py` 至 `analyze_j4_highspeed.py`；各自输入格式以参数说明和源码为准。历史调参值不作为其他设备默认值。

## 4. 区分读取、RAM 写入与 Flash 保存

| 工具 | 功能与边界 |
| --- | --- |
| `tools/arm_diagnostics/scanner.py` | 按角色查询状态，不使能、不写参数；推荐 `scripts/run_arm_scan.bash --cli` |
| `tools/read_motor_position_params.py` | 单电机读取 ID、量程和伺服寄存器；发送端白名单仅允许读取／状态 |
| `src/a6_y_ros2_control/tools/read_motor_params.py` | 按配置关节／ID 读取寄存器，`--dry-run` 预览 |
| `src/a6_y_ros2_control/tools/write_motor_params_temp.py` | `--set` 修改 RAM，真正写入需 `--yes` |
| `src/a6_y_ros2_control/tools/save_motor_params_flash.py` | 独立 Flash 保存，需 `--yes`；不是临时调参 |
| `src/a6_y_ros2_control/tools/damiao_set_id.py` | 默认离线预览；`--diagnose`／`--verify` 只读，`--write` 处理失能、改 ID 并保存 Flash |
| `src/a6_y_ros2_control/tools/motor_status.py` | 订阅已有 ROS 动态状态，不独占串口 |

```bash
/usr/bin/python3 src/a6_y_ros2_control/tools/read_motor_params.py --help
/usr/bin/python3 src/a6_y_ros2_control/tools/write_motor_params_temp.py --help
/usr/bin/python3 src/a6_y_ros2_control/tools/save_motor_params_flash.py --help
/usr/bin/python3 src/a6_y_ros2_control/tools/damiao_set_id.py --help
```

先读取当前值、核对单位和范围，再计划写入与回读。PMAX／VMAX 是编码量程，不能当作运动目标限位。USB 迁移见[适配器更换](replace-usb-can-adapter.md)；原始寄存器和实验日志保留本地，不作为公开样例锚点。

## 5. 回归上层入口

回到仓库根目录，动力示教按[离线测试指南](run-offline-tests.md)运行，MoveIt 用[独立栈测试](../../tools/arm_moveit/README.md#开发离线回归与来源)。固定阻抗停止撤去支撑，桌面示教正常退出先回零，MoveIt 退出取决于接管与规划状态；不能共用一套退出假设。

记录版本、命令、输出和适用模式，只有本次实际执行通过才计入结论。dry_run、PTY、模型动力学分别覆盖接口、协议和假设工况，不能合并为全部真机验证。
