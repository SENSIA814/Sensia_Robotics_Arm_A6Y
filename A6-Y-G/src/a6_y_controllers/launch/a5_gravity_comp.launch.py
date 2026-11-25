"""Gravity compensation using the complete A5 arm and gripper model."""

from pathlib import Path
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _shutdown_if_failed(event, _context):
    if event.returncode != 0:
        return [EmitEvent(event=Shutdown(reason="A5 arm/gripper controller spawner failed"))]
    return []


def _launch_setup(context):
    share = Path(get_package_share_directory("a6_y_controllers"))
    model_file = Path(get_package_share_directory("a5_description")) / "urdf/A5.urdf"
    mappings = {
        key: LaunchConfiguration(key).perform(context)
        for key in ("dry_run", "auto_enable", "can_channel", "can_bitrate", "tty_baudrate")
    }
    for key in ("dry_run", "auto_enable"):
        if mappings[key].lower() not in ("true", "false"):
            raise ValueError(f"{key} must be true or false")
        mappings[key] = mappings[key].lower()
    motor_config = LaunchConfiguration("motor_config_file").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    mappings.update(model_file=str(model_file), motor_config_file=motor_config)
    description = xacro.process_file(
        str(share / "config/a5.urdf.xacro"), mappings=mappings
    ).toxml()

    # Pin Pinocchio to the robot_description model even for custom tuning YAML.
    with open(controllers_file, encoding="utf-8") as stream:
        controller_config = yaml.safe_load(stream)
    controller_name = LaunchConfiguration("controller_name").perform(context)
    if controller_name not in ("a6_y_gravity_comp_controller", "a6_y_cartesian_impedance_controller",
                               "a6_y_joint_impedance_controller"):
        raise ValueError("Unsupported arm controller")
    params = controller_config[f"/arm6/{controller_name}"]["ros__parameters"]
    gravity_file = LaunchConfiguration("gravity_parameters_file").perform(context)
    if gravity_file and controller_name in ("a6_y_cartesian_impedance_controller", "a6_y_joint_impedance_controller"):
        with open(gravity_file, encoding="utf-8") as stream:
            gravity_params = yaml.safe_load(stream)["/arm6/a6_y_gravity_comp_controller"]["ros__parameters"]
        # Reuse the user's commissioned gravity values, never Cartesian gains from another robot.
        for key in ("torque_scales", "effort_signs", "damping_gains", "torque_limits"):
            params[key] = gravity_params[key]
    expected_joints = [f"joint{i}" for i in range(1, 7)] + ["gripper_finger_joint1"]
    if params["joints"] != expected_joints:
        raise ValueError("Arm controller must command the six arm joints and gripper_finger_joint1 in order")
    if controller_name == "a6_y_gravity_comp_controller" and params.get("hardware_enable_interface") != "motor_enable":
        raise ValueError("A5 arm/gripper gravity compensation requires motor_enable handshake")
    params["urdf_path"] = str(model_file)

    # Humble spawner loads controller parameters from a file. Use one scoped
    # snapshot for manager and spawner, and remove it at launch shutdown.
    with tempfile.NamedTemporaryFile(
        mode="w", prefix="arm6_gravity_", suffix=".yaml", delete=False
    ) as stream:
        yaml.safe_dump(controller_config, stream)
        pinned_file = Path(stream.name)

    def cleanup(_event, _context):
        pinned_file.unlink(missing_ok=True)
        return []

    robot_description = {"robot_description": ParameterValue(description, value_type=str)}
    manager = Node(
        package="controller_manager", executable="ros2_control_node", namespace="arm6",
        parameters=[robot_description, str(pinned_file)], output="screen",
        on_exit=EmitEvent(event=Shutdown(reason="A5 arm/gripper controller manager exited")),
    )
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher", namespace="arm6",
        parameters=[robot_description], output="screen",
    )
    broadcaster = Node(
        package="controller_manager", executable="spawner", namespace="arm6", output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/arm6/controller_manager"],
    )
    gravity = Node(
        package="controller_manager", executable="spawner", namespace="arm6", output="screen",
        arguments=[
            controller_name, "--controller-manager", "/arm6/controller_manager",
            "--param-file", str(pinned_file),
        ],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", namespace="arm6",
        arguments=["-d", str(share / "config/a5_readonly.rviz")],
        output="screen", condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=_shutdown_if_failed)),
        RegisterEventHandler(OnProcessExit(target_action=gravity, on_exit=_shutdown_if_failed)),
        state_publisher, manager, broadcaster, gravity, rviz,
    ]


def generate_launch_description():
    config = Path(get_package_share_directory("a6_y_controllers")) / "config"
    return LaunchDescription([
        DeclareLaunchArgument(
            "dry_run", default_value="true", description="No CAN connection when true"
        ),
        DeclareLaunchArgument(
            "auto_enable", default_value="false",
            description="Allow hardware enable after controller ready",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false",
            description="Display the complete A5 arm and gripper model using /arm6 real or dry-run feedback",
        ),
        DeclareLaunchArgument("can_channel", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("can_bitrate", default_value="1000000"),
        DeclareLaunchArgument("tty_baudrate", default_value="115200"),
        DeclareLaunchArgument(
            "motor_config_file", default_value=str(config / "a5_motors.yaml")
        ),
        DeclareLaunchArgument("controller_name", default_value="a6_y_gravity_comp_controller"),
        DeclareLaunchArgument("gravity_parameters_file", default_value=""),
        DeclareLaunchArgument(
            "controllers_file", default_value=str(config / "a5_gravity_controllers.yaml")
        ),
        OpaqueFunction(function=_launch_setup),
    ])
