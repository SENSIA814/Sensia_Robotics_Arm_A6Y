# a6_y_ros2_control config

The active A6_Y motor map is consumed by the robot xacro and lives in:

```text
src/A6_Y_Moveitconfig/config/a6_y_damiao_motors.yaml
```

Keep motor IDs, direction signs, offsets, soft position limits, and conservative velocity limits there so the generated `ros2_control` URDF has a single source of truth.

Full-arm hardware is split into two ros2_control systems: right arm on `/dev/ttyACM0` and left arm on `/dev/ttyACM1`. Because the buses are independent, both arms intentionally use `slave_id=0x01..0x07` and `master_id=0x11..0x17`.

Single-motor ros2_control validation uses local minimal config files:

```text
src/a6_y_ros2_control/config/a6_y_single_motor.urdf.xacro
src/a6_y_ros2_control/config/single_motor_ros2_controllers.yaml
```

Those files are for bench validation of one motor, currently defaulting to `joint1` with `DM6428P`, `slave_id=0x01`, and `master_id=0x11`.
