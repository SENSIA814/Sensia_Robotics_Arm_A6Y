# 软件验证与实物验收边界

验证对象为三个包：`a5_description`、`a6_y_ros2_control`、`a6_y_controllers`。
所有下列检查均不访问真实电机；离线通过不代表实物标定或重力补偿效果已验收。

## 执行方法

先按 [README](../../README.md) 完成构建，再在项目根目录运行：

```bash
source scripts/environment.bash
a6y_load_install
export ROS_LOG_DIR="$PWD/validation/ros_logs"
export ROS_DOMAIN_ID=193
export ROS_LOCALHOST_ONLY=1
mkdir -p "$ROS_LOG_DIR"
ctest --test-dir build/a6_y_ros2_control --output-on-failure
ctest --test-dir build/a6_y_controllers --output-on-failure
/usr/bin/python3 tools/validate_bundle.py
/usr/bin/python3 tools/derive_a5_tcp.py
/usr/bin/python3 tools/test_s0.py
/usr/bin/python3 tools/verify_s0.py
/usr/bin/python3 tools/verify_dry_run.py
```

## 检查范围

| 检查 | 内容 | 报告 |
| --- | --- | --- |
| 驱动 CTest | 协议、有效位置接收、伪终端反馈采样、电机 ID 写入/保存模拟与伪终端核对 | 构建目录的 CTest 输出 |
| 控制器 CTest | 力输出合成、零力命令、七轴重力、连续摩擦补偿与生命周期 | 构建目录的 CTest 输出 |
| `validate_bundle.py` | 网格、模型、配置、32 姿态重力与势能梯度 | `validation/model_check.json` |
| `derive_a5_tcp.py` | 两指 STL 包围盒中心、三种开度的 TCP 一致性 | `validation/tcp_geometry.json` |
| `test_s0.py` | 六项几何与标定记录回归 | 终端 unittest 输出 |
| `verify_s0.py` | 独立 FK、模型/驱动限位、实测记录一致性 | `validation/s0_check.json` |
| `verify_dry_run.py` | ROS 启停、七轴消息、28 命令接口、TCP TF、重力输出与禁用状态 | `validation/dry_run_check.json` |

完整启动检查默认使用 domain 193、localhost 通信和不可用设备路径，显式指定 `dry_run=true`、`auto_enable=false`。可通过 `A6Y_TEST_ROS_DOMAIN_ID` 修改测试 domain。日志输出到 `validation/ros_logs/` 和 `validation/dry_run_launch.log`。

连续摩擦补偿的单元测试覆盖静止噪声死区、持续拖动、输出幅值及渐增速度、减速/停动/换向、非法输入和长控制周期。控制器集成测试覆盖关节数组映射、重力与摩擦叠加后的总力矩限幅、禁用输出、异常状态清零、重新激活时的补偿重置，以及非法参数和模式冲突。dry-run 检查实际加载的摩擦参数，并验证零速度下没有附加摩擦力矩。这些检查不代表已经测量真实摩擦或验证实机手感。

## 实物记录

`a5_calibration.yaml` 中的安装方向、电机映射、夹爪间距和 TCP 测量仍需现场填写。只修改 `verified` 标志不能替代实际观测。

```bash
/usr/bin/python3 tools/verify_s0.py --require-field-calibration
```

未完成或不一致的实测记录使严格检查退出 2；普通模式在软件一致性通过时退出 0，并列出待测项。具体方法见 [模型与标定](S0_CALIBRATION.md)。

历史报告只描述生成当时的配置；改动模型、参数或驱动后，应重新执行检查。构建目录和安装目录为可再生文件，不属于交付源码。

## 2026-10-07 整理后验证

三个包的 colcon 构建通过；驱动 CTest 3/3、控制器 CTest 3/3、标定 unittest 6/6 全部通过。模型、STL TCP、S0 软件一致性和完整 ROS dry-run 通过，launch 正常退出。Python 语法、XML、文档链接与重命名引用检查通过。

验证在允许本机 DDS 通信的环境中完成，未访问真实串口。构建出现依赖库 FindBoost 的 python310 头文件检查警告，未影响成功结果。汇总为 `validation/cleanup_check.json`。实物标定仍未完成。

整理阶段验证后曾清理构建、安装与构建日志目录。重新构建时，colcon/ament 会自动生成环境脚本。

## 2026-10-07 连续摩擦补偿验证

已重新构建并安装 `a6_y_controllers`。控制器 CTest 4/4 组、21/21 用例通过，其中新增 6 个摩擦算法用例和 3 个控制器集成用例。`validate_bundle.py` 的配置与 32 姿态模型检查通过。

完整 dry-run 通过，确认实际加载连续补偿开关及七轴参数、零速度无附加摩擦力矩、七轴硬件使能均为 0，launch 正常退出。沙箱内首次启动受到 DDS 套接字权限限制，重试在允许本机通信的环境完成，仍固定 `dry_run=true`、`auto_enable=false`、无效串口路径；未访问真实电机。报告为 `validation/model_check.json` 和 `validation/dry_run_check.json`。

J1/J3 的 0.15 N·m 是试调起点，尚无实机摩擦辨识或手感验收。J2 补偿为零。原配置 J2/J3 的 `torque_limits=20` 与运动限位文件中的 `max_effort=10` 不一致，本次未改动这组限值，以上结果不表示 S0 限位一致性已通过。
