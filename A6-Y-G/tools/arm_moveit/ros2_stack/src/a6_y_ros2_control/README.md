# a6_y_ros2_control

`a6_y_ros2_control` is the ros2_control hardware layer for the A6_Y Damiao motor arm.

Current scope:

- Controls the six arm joints `joint1..joint6`.
- Keeps the gripper joints in the robot model but does not register them in ros2_control yet.
- Uses a CANable2/SLCAN backend.
- Supports `dry_run`, real read-only feedback, `POS_VEL` trajectory control, and MIT command interfaces for later gravity compensation.
- Publishes extra diagnostics through `/dynamic_joint_states`: `motor_error`, `fault`, `has_feedback`, `feedback_age`, and `enabled`.

Default motor mapping is stored in:

```text
src/A6_Y_Moveitconfig/config/a6_y_damiao_motors.yaml
```

Initial motor order:

```text
joint1 -> DM4340P
joint2 -> DM4340P
joint3 -> DM4340P
joint4 -> DM4310
joint5 -> DM4340
joint6 -> DM4340
```

Validate the config:

```bash
ros2 run a6_y_ros2_control validate_motor_config.py
```

Dry-run ros2_control:

```bash
ros2 launch a6_y_ros2_control a6_y_ros2_control.launch.py dry_run:=true
```

Real read-only bringup:

```bash
ros2 launch a6_y_ros2_control a6_y_real_readonly.launch.py \
  can_channel:=/dev/ttyACM0
```

Real MoveIt bringup:

```bash
ros2 launch a6_y_ros2_control a6_y_real_moveit_bringup.launch.py \
  can_channel:=/dev/ttyACM0
```

Before enabling real motion, confirm every motor has fresh feedback, no faults, correct `sign`, and calibrated `offset`.
