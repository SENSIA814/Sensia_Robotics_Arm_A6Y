# 运行离线测试

本指南用于修改代码或准备发布后验证软件行为。需要[运行依赖](../reference/runtime-and-configuration.md#环境要求)、可写的构建／日志目录，以及允许 ROS 本机通信和 PTY 虚拟串口的环境。以下命令从仓库根目录执行，不打开实体串口；各组按顺序运行，避免共享缓存和锁互相影响。

## 1. 快速检查配置

```bash
bash A6-Y-G/scripts/run_arm_scan.bash --cli --arms all --dry-run
bash A6-Y-G/scripts/run_arm_rviz.bash --cli --arm motion_left --dry-run
bash A6-Y-G/scripts/run_arm_teaching.bash --check
```

这组检查验证配置可读、模型与身份关联；示教检查首次可能构建 ROS 包。退出码非零时先修复报告中的问题，再进行集成测试。

## 2. 验证诊断与只读模型工具

```bash
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -s -m unittest discover \
  -s A6-Y-G/tools/arm_diagnostics -p 'test_*.py' -v
/usr/bin/python3 -s -m unittest discover \
  -s A6-Y-G/tools/arm_visualization -p 'test_*.py' -v
```

诊断测试会使用回环服务与虚拟串口。沙箱禁止这些接口导致的失败，需要与代码断言失败分开记录。

## 3. 验证完整动力示教链路

```bash
bash A6-Y-G/scripts/test_arm_teaching.bash
```

脚本顺序执行 Python 单元测试、相关控制器与驱动的 CTest 检查、四臂 PTY 跟随／退出、夹爪轻阻尼与接触反馈、退出失败／反馈故障／模式失败／取消回零及离线 UI 检查。脚本遇失败停止；不要仅凭最后一个已通过的用例宣称整组完成。

构建缓存位于 `/tmp/a6y-gravity-build-<uid>/`，ROS 测试日志位于 `diagnostics/arm_teaching_test_logs/`。模拟会话及验证产物位于 `diagnostics/arm_teaching_test_reports/`；具体目录由脚本输出。此入口使用 ROS domain 196，运行期间不要同时启动默认同 domain 的独立 MoveIt 会话。

需要验证独立重力补偿时另运行：

```bash
bash A6-Y-G/scripts/test_arm_gravity.bash
```

## 4. 记录结论

本次发布记录为 **201 项 Python 单元测试、12 项 CTest 检查和 7 组离线集成验证**；12 个 CTest 条目中包含 Python 检查，并非全部是 C++ 测试。201 项对应诊断与重力／示教测试，不包含上面单列的只读模型测试。

记录源码版本、执行命令、通过／失败范围和产物路径。测试数量可能随版本变化，应引用本次输出；不要把历史测试数量写成当前固定保证。

PTY 运行真实 ROS 节点与 C++ 协议驱动，但电机由程序模拟。它能检查指令、模式切换、接触保持和退出流程，不能证明实体跟踪延迟、摩擦、反馈手感或夹持力。真机试用结果应单独记录。
