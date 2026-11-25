# 实验程序、设备配置与验证记录

本目录与 `A6-Y-G/` 保持相邻。`arm_roles.yaml` 是四臂程序实际读取的 USB 身份配置，不能把整个 `diagnostics/` 当作临时日志目录删除。

## 仓库保留内容

- 四臂角色绑定、适配器记录、电机寄存器与量程快照。
- 当前实验分析和调参脚本，以及它们旁边的参数、结果摘要和图表。
- 界面截图、功能验证 JSON 和移植来源记录。
- 主程序源码位于 `../A6-Y-G/`；带日期的目录是当时实验记录，不能视为每台设备都适用的默认配置。

| 入口 / 目录 | 用途 |
| --- | --- |
| `inspect_arms_readonly.py` | 独立只读电机检查 |
| `run_interface_c_readonly.bash` | 早期本机只读入口，含原工作区路径，使用前需调整；日常推荐主项目的 `run_arm_scan.bash` |
| `teaching_tracking_analysis_20261125/` | 示教跟踪、速度与时序分析及 HTML 报告生成 |
| `teaching_250hz_analysis_20261125/` | 250 Hz 阶段的跟踪分析 |
| `teaching_following_analysis_20261125_*/` | 指定实验会话的跟随误差分析 |
| `teaching_step_logging_20261125/scan_parameters.py` | 当时阶跃日志实验使用的电机参数读取 |
| `teaching_motor_limits_revision_20261125/read_limits.py` | 读取电机量程与寄存器记录 |
| `teaching_tuning_20261125/`、`*_trial_20261125/` | 实验调参脚本、基准和结果；其中 `apply_gains.py` 属于真机参数操作 |
| `gravity_home_revision_20261125/analyze_previous_home.py` | 历史回零日志分析 |

## 未纳入 Git 的本地数据

根目录 `.gitignore` 排除了 `*_reports/`、`*_logs/`、`before/` 修改前备份、Python 缓存和 `*.log`。这些文件仍保留在开发电脑，程序运行后也会继续生成。原始高频 CSV、逐帧数据和大量 ROS 会话不随源码仓库上传。

历史 `analyze.py` 常按固定会话 ID 读取 `arm_teaching_reports/`，部分会话元数据还记录原机器的绝对路径。因此克隆仓库后可以阅读代码和已保存结果，但重新计算这些历史结果需要恢复原始数据并核对路径。结果中的旧路径和时间戳作为实验来源保留，不代表仓库克隆位置。

新采集会话优先使用通用分析入口（从 `A6-Y-G/` 执行）：

```bash
/usr/bin/python3 tools/arm_gravity/analyze_teaching_step.py /绝对路径/teaching_会话目录
```

该入口离线分析已有日志，不连接 ROS 或电机。历史调参脚本则可能访问实体串口、修改电机 RAM 参数；请先阅读其参数和适用设备，不能作为安装或自检命令批量执行。
