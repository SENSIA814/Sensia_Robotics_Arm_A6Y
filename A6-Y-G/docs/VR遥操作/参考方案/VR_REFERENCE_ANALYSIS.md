# Meta to Piper 源码调研

核对日期：2026-10-08。用途：为 A6-Y-G 的 VR 控制 RViz 虚拟 A5 机械臂方案提供源码依据。参考目录为 `/home/alce/Code/Meta_to_Piper`，本附件只记录本地源码事实与移植判断；源码存在不等于已在当前设备完成验证。

推荐抽取 Quest 浏览器输入、握把相对控制和位姿滤波的设计；A5 模型、IK 模型加载、ROS 2 输出及虚拟状态应在 A6-Y-G 中独立实现。参考工程的正常输出是 Piper SDK/CAN 关节指令，其 dry-run 是 MuJoCo 可视化，并非现成的 RViz 遥操作包。[运行时构建](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:121)、[发送实现](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/control/sender.py:29)

## 两套参考实现的区别

| 项目 | piper-quest3-teleop | Meta_to_Arm_staging |
|---|---|---|
| 当前源码结构 | 左右臂各自状态、mapper、IK、发送进程，主循环串行更新两臂 | 单臂运行时；左手残留底盘接口，底盘 ROS 桥未启用 |
| IK | MuJoCo 模型与 mink；配置 DAQP | Pinocchio FK/Jacobian、CasADi 组装 QP、ProxQP 求解 |
| 机器人模型 | Piper MJCF，另用独立 Piper DH FK 做基准和骨架 | 由 Piper MJCF 手工转写的 Pinocchio 模型，另保留 Piper DH FK |
| 设置频率 | 主循环与发送各 100 Hz；录制反馈读回 20 Hz | 主循环 200 Hz；发送参数未定义时默认 100 Hz |
| 虚拟显示 | dry-run 打开两个 MuJoCo viewer；Vuer 显示骨架 | `viewer=None`，保留 Vuer 骨架 |
| 录制 | 主循环 ZMQ 发布，独立进程写 CSV | 本次所查 app 无对应发布链路 |

依据：[双臂构建](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:45)、[mink 初始化](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/init_mink.py:9)、[频率配置](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/config.py:14)、[staging 单臂构建](/home/alce/Code/Meta_to_Piper/Meta_to_Arm_staging/teleop/app.py:31)、[staging 频率配置](/home/alce/Code/Meta_to_Piper/Meta_to_Arm_staging/teleop/config.py:14)。

主目录有使用说明、依赖清单、录制和滤波等较完整的配套；staging 是一条已写出代码的 Pinocchio 迁移路线，不能据此认定验证成熟度更高。尤其 staging 模型文件头的“FK 误差约 1e-15”是作者注释，本附件未复现该结论。[模型说明](/home/alce/Code/Meta_to_Piper/Meta_to_Arm_staging/teleop/runtime/build_piper_model.py:1)

中文 README 与当前源码有几处偏差：README 称左手基本不用，但主目录当前创建左右两臂；README 称退出会回零，当前清理流程是停 sender、关 viewer 和输入资源；A 按钮也只在 HOLD 分支触发回零，不能按文档概括成“任何时候按 A 都回零”。[README 左手与退出描述](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/README.zh-CN.md:166)、[双臂构建](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:125)、[清理流程](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/context.py:102)、[HOLD 分支](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:238)

## VR 输入和传输

输入路线是 Quest 浏览器访问 Vuer HTTPS 服务，通过 WSS 事件取得控制器数据；本地应用没有使用原生 Meta Quest SDK。服务注册 `CONTROLLER_MOVE`，前端启用 `MotionControllers(stream=True, left=True, right=True)`；事件回调把左右手各 16 个矩阵元素和 14 个状态量写入 Python 多进程共享内存。主循环轮询共享内存，而不是每个控制周期都等待一帧新 VR 数据。[Vuer 服务与事件注册](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:58)、[事件处理](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:144)、[前端控制器](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:270)

关键输入约定：

- 位姿矩阵从 16 元素按 **列主序** `reshape(4,4,order="F")` 还原；后续 pose7 为 `[x,y,z,qx,qy,qz,qw]`。[矩阵读取](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:342)、[pose7 转换](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/VuerTeleop.py:56)
- 按键数组中 0 为 trigger 布尔值、1 为 squeeze、4 为 A、5 为 B、6 为 triggerValue、7 为 squeezeValue、10/11 为摇杆；其余是触摸板、摇杆按下、A/B 数值等字段。[状态解码](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:166)
- 本地锁定 Vuer 版本为 0.0.70；服务有自动添加 `?ws=wss://host:port` 的重定向中间件，注释将其解释为该版 HTTPS 端口处理兼容补丁。移植时可保留为可选兼容处理，不能把注释当作所有 Vuer 版本的行为。[依赖锁定](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/requirements.txt:25)、[中间件](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:18)
- 配套说明使用同一局域网与 `https://<电脑IP>:8012`。源码证书路径相对 `teleop` 解析。新项目应配置自己的地址和新证书，不复制参考项目的私钥或把其旧 IP 固化为默认部署值。[连接说明](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/README.zh-CN.md:95)、[证书路径](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:45)

主启动器主动清除 `PYTHONPATH`、禁用 user site，再激活 `piper-teleop` conda；staging 使用 `piper_pinocchio`。因此不能把整个脚本放进已 source ROS 2 的终端后就假定能直接导入 rclpy。独立 Vuer 网关和 ROS 2 节点之间定义稳定数据协议，更适合当前虚拟 A5 接入。[主启动器](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/run_teleop.sh:6)、[staging 启动器](/home/alce/Code/Meta_to_Piper/Meta_to_Arm_staging/run_teleop.sh:6)

## 按键状态机和坐标映射

握把使用布尔 squeeze 通道，阈值 0.5，控制器模块计算按下/松开边沿。进入 TELEOP 时记录当前手柄位姿和机器人基准目标；从 HOLD 重新握住时再次捕获基准，因此可反复“握住移动、松开换手位、再握住”。扳机模拟量经死区和 EMA 转为 0..1000 夹爪目标。主目录初始状态为 RETURNING，以关节速度 0.5 rad/s 回到全零，再等待握把。[按键处理](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/init_right_controller.py:146)、[夹爪处理](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/init_right_controller.py:203)、[回零及 clutch](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:204)、[初始状态](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/context.py:39)

实际坐标链不是单一 `P` 矩阵：

1. Vuer 输入先做基变换 `T_z = C T_vr C^-1`，`C` 的旋转部分把原坐标 `(x,y,z)` 映射为 `(-z,-x,y)`。[转换实现](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/Preprocessor.py:22)、[矩阵定义](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/constants_vuer.py:11)
2. mapper 的平移为 `p_target = p_robot_ref + s_p (p_z - p_z_ref)`；此处没有再次使用 `P`。[平移实现](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/mapping/vr_mapper.py:154)
3. 姿态先算 `dR = R_z_ref^T R_z`，再算 `dR_robot = P dR P^T`，最后右乘基准姿态 `R_target = R_robot_ref dR_robot`。P 映射 `(-z,y,x)`，其轴约定与上一步 C 不同。[姿态实现](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/mapping/vr_mapper.py:189)、[P 定义](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/mapping/vr_mapper.py:24)
4. 当前 mapper 实例平移缩放为 1.0、旋转缩放为 0.9；旋转缩放采用轴角，只缩放角度。[实例配置](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/init_fk_mapper.py:21)、[轴角缩放](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/mapping/vr_mapper.py:196)

A5 移植应明确“手柄局部旋转增量”或“工作台固定轴旋转增量”的选择，并为平移、旋转使用可解释的统一坐标约定；Piper 的 `P`、DH 零位和末端朝向不应直接套到 A5。握住瞬间不跳变、正向三轴平移与三轴旋转是必要验收案例。参考滤波器的滞回死区、位置自适应低通和姿态 SLERP 可以借鉴，但参数需要按新采样时间重新验证。[位姿滤波](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/control/pose_filter.py:75)

## IK 模型和输出接口

主目录用 mink 的末端 FrameTask 与 PostureTask，设置关节位置限制和速度限制；配置指向 Piper `piper.xml` 的 `attachment_site`。每步根据上一关节目标更新 configuration，求速度、积分后把结果直接写回 MuJoCo qpos 并执行 `mj_forward`。这条干运行路径显示运动学姿态，不是由执行器/接触动力学积分得到的运动。[任务与限位](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/init_mink.py:14)、[IK 与姿态更新](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/control/ik_stepper.py:23)

staging 的 QP 变量是关节位移 Δq，目标项包括位置、姿态、回归参考姿态和阻尼；约束合并关节位置范围与 `max_vel*dt` 单步限制。Pinocchio Jacobian 使用 `LOCAL_WORLD_ALIGNED`，旋转误差为 `R_current*log3(R_current^T*R_target)`。该数学结构可供 A5 IK 参考；现有模型构造器硬编码六个 Piper 关节的安装位姿、轴、限位和末端偏移，必须替换为 A5 自身 URDF/TCP。求解代码还需补充解算状态、残差、有限值与结果限位检查。[QP 实现](/home/alce/Code/Meta_to_Piper/Meta_to_Arm_staging/teleop/control/pin_ik.py:92)、[硬编码模型](/home/alce/Code/Meta_to_Piper/Meta_to_Arm_staging/teleop/runtime/build_piper_model.py:14)

两套参考程序的主要机械臂输出都是共享内存中的六个弧度关节目标，经 sender 转为 Piper 整数角度后调用 `JointCtrl`，夹爪调用 `GripperCtrl`。Piper SDK 类和这些单位换算不适用于 A5 的达妙驱动，也不是 ROS 2 `JointState` 输出。[发送进程](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/piper_send_process.py:73)、[整数换算](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/control/sender.py:29)、[SDK 调用](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/piper/driver.py:70)

名为 `init_ros2_bridge.py` 的模块实际用于 Vision60 底盘，发布 `/mcu/command/manual_twist` 并调用 `/ensure_mode`；它不是机械臂 ROS 桥，不能用作 RViz 关节状态接口。[ROS 桥源码](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/init_ros2_bridge.py:13)

## 频率和录制的边界

100 Hz/200 Hz 是主循环限速器的设置，不代表 VR 事件新鲜频率。事件处理函数签名里的 `fps=60` 未用于限速；图像/骨架发送循环实际 `sleep(0.03)`，还要加渲染与编码耗时。主 README 也明确保留“循环频率不匹配目标”的待办，因此不能引用这些设置证明持续实测频率。[事件函数](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:144)、[显示循环](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:333)、[README 待办](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/README.md:23)

主目录录制使用 ZMQ PUB 非阻塞发送 JSON，独立订阅器生成 `hand.csv`、`arm_command.csv`、`arm_actual.csv`；记录输入、目标、实际读回以及对应时间，设计值得借鉴。记录中的 `t` 是主循环组帧时间，并不是原始 Quest 事件采样时间；100 Hz 录制中可能重复 20 Hz 读回和更低频输入。没有新帧序号时，不能从 CSV 行数推断 VR 新鲜采样率。[发布与记录字段](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/recorder_pub.py:21)、[CSV](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/record_data.py:115)、[读回频率](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/config.py:51)

仓库的 `scripts/replay_demo.py` 使用 Isaac Gym、H1 Inspire 机器人和另一套演示数据，不是上述 Piper CSV 的现成回放器。A5 阶段应自行定义可重复的输入记录/离线回放入口，不把这个文件计作已完成能力。[回放模型](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/scripts/replay_demo.py:58)

## 必须补齐的问题

| 问题 | 源码依据与对 A5 的影响 |
|---|---|
| 原始输入失联识别 | 事件回调仅覆盖共享矩阵/按键，未写入原始事件时间或序号；断连处理退出显示协程，也未清除旧 squeeze。网关必须在真实事件到达时更新 session、seq、monotonic 时间，转发心跳不能给旧样本换新时间。[事件](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:144)、[断连](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:335) |
| 松开边沿精确冻结 | TELEOP 分支先计算 target，再在松开时修改 mode 和 hold_target；本帧局部 target 未替换，后面仍可能执行一次 IK。新状态机应先处理 release/stale，再决定是否运行 IK。[分支顺序](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:260) |
| HOLD 的含义 | 参考 HOLD 仍对保持位姿执行 IK，存在 posture 项影响；若要求虚拟关节完全冻结，应冻结 q，而不只是保存末端 target。[HOLD](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:238)、[IK 分支](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:277) |
| 夹爪单位 | `grip_out` 已为 0..1000，app 写共享内存前再次乘 1000；sender 又把值截到 0..1000。按此源码路径，中间模拟量可能快速饱和。A5 应从归一化 trigger 直接映射自身夹爪坐标，不移植这段单位链。[写入](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/app.py:304)、[截断](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/control/sender.py:43) |
| 自动回零 | 参考初始状态主动 RETURNING，且 q=0 是 Piper 的特定选择。A5 虚拟场景应从配置的可解初始姿态进入 HOLD，复位只作为明确的虚拟动作。[初始状态](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/runtime/context.py:39) |
| 并发样本一致性 | 矩阵与按键存于不同共享数组，写入也分开；消费者可遇到位姿、按键来自相邻事件。网关宜将两者连同元数据作为一个原子快照转发。[共享内存](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:79)、[覆盖顺序](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/TeleVision.py:155) |

## 复用清单和来源标注

| 类别 | 移植选择 |
|---|---|
| Vuer 输入 | 借鉴 HTTPS/WSS 建连、CONTROLLER_MOVE 解码、列主序和浏览器兼容处理；重构成不依赖 Piper/相机/骨架的网关 |
| clutch 和滤波 | 借鉴边沿检测、相对基准捕获、四元数连续性、位姿滤波；补断流/重连/原子帧和正确的 release 优先级 |
| IK | 参考 staging 的 Pinocchio FK/Jacobian 与限位思想；用 A5 URDF 和真实 TCP，新实现求解/异常检查，不复制 Piper 硬编码模型 |
| 输出 | 新建 ROS 2 虚拟 JointState 与状态诊断，接 robot_state_publisher/RViz；不引入 Piper SDK、CAN sender 或 ros2_control 硬件启动 |
| 录制 | 借鉴输入/目标/状态分离，增加原始事件时序和序号；离线回放作为新功能验收 |
| 依赖 | 从网关最小依赖开始；原 requirements 含 torch、训练/视觉、Piper SDK 等无关包，且 recorder 使用的 pyzmq 未在清单列出，不能整份当作最小可复现环境。[依赖清单](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/requirements.txt:1)、[zmq 导入](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/recorder_pub.py:13) |

主目录根 LICENSE 标注 Apache 2.0，并列出 ACT/DETR/dex-retargeting/Vuer 等来源；Piper 模型目录另附 MIT LICENSE。staging 顶层当前未见独立 LICENSE/requirements，不能把它的来源记录省略。落地时只保留实际使用代码的来源、版权头与相应许可证，Vuer 等第三方依赖按实际安装版本记录许可；参考仓库的根 LICENSE 不能替代各依赖自己的许可。[主 LICENSE](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/LICENSE:1)、[模型 LICENSE](/home/alce/Code/Meta_to_Piper/piper-quest3-teleop/teleop/piper/agilex_piper/LICENSE:1)
