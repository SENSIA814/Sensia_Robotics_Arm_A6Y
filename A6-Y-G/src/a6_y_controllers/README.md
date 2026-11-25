# A5 七轴重力控制器

使用完整 A5 模型及 Pinocchio mimic 支持，控制六轴机械臂和一台夹爪电机。
夹爪反馈参与重力计算，默认夹爪命令为零力。

源码入口为 `src/A6_y_gravity_comp_controller.cpp`，插件声明为
`A6_y_controllers_plugin.xml`。配置位于 `config/`，启动入口为
`launch/a5_gravity_comp.launch.py`。

构建、使用、参数和验证步骤统一维护在项目根目录 README.md；安装后请查阅源码工作区的 README。

另有两个独立模式：`a5_cartesian_impedance.launch.py` 保持末端位姿，
`a5_joint_impedance.launch.py` 保持六轴关节角度。三个控制器共用命令接口，不能同时激活。
关节模式参数位于 `config/a5_joint_impedance_controllers.yaml`，其六项增益对应 joint1～joint6。
