from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _load_default_motor_config():
    config_path = Path(get_package_share_directory("A6_Y_Moveitconfig")) / "config" / "a6_y_damiao_motors.yaml"
    defaults = {
        "dry_run": "true",
        "can_channel": "/dev/ttyACM0",
        "can_bitrate": "1000000",
        "tty_baudrate": "115200",
        "control_mode": "pos_vel",
        "auto_enable": "false",
        "set_control_mode_on_configure": "false",
        "require_feedback_on_configure": "true",
        "allow_partial_feedback": "false",
        "disable_on_deactivate": "true",
        "command_timeout": "0.2",
        "max_start_delta": "0.25",
        "receive_timeout": "0.001",
        "feedback_timeout": "0.5",
        "status_request_period": "0.05",
        "initial_feedback_attempts": "60",
        "mit_kp": "0.0",
        "mit_kd": "0.5",
    }
    try:
        with config_path.open("r", encoding="utf-8") as stream:
            params = yaml.safe_load(stream)["a6_y_damiao_system"]["ros__parameters"]
        can = params.get("can", {})
        configured_defaults = params.get("defaults", {})
        defaults.update(
            {
                "can_channel": _launch_value(can.get("channel", defaults["can_channel"])),
                "can_bitrate": _launch_value(can.get("can_bitrate", defaults["can_bitrate"])),
                "tty_baudrate": _launch_value(can.get("tty_baudrate", defaults["tty_baudrate"])),
            }
        )
        for key in tuple(defaults.keys()):
            if key in configured_defaults:
                defaults[key] = _launch_value(configured_defaults[key])
    except Exception:
        pass
    return str(config_path), defaults


def generate_launch_description():
    hardware_name = LaunchConfiguration("hardware_name")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    motor_config_file = LaunchConfiguration("motor_config_file")
    dry_run = LaunchConfiguration("dry_run")
    can_channel = LaunchConfiguration("can_channel")
    can_bitrate = LaunchConfiguration("can_bitrate")
    tty_baudrate = LaunchConfiguration("tty_baudrate")
    control_mode = LaunchConfiguration("control_mode")
    auto_enable = LaunchConfiguration("auto_enable")
    set_control_mode_on_configure = LaunchConfiguration("set_control_mode_on_configure")
    require_feedback_on_configure = LaunchConfiguration("require_feedback_on_configure")
    allow_partial_feedback = LaunchConfiguration("allow_partial_feedback")
    disable_on_deactivate = LaunchConfiguration("disable_on_deactivate")
    command_timeout = LaunchConfiguration("command_timeout")
    max_start_delta = LaunchConfiguration("max_start_delta")
    receive_timeout = LaunchConfiguration("receive_timeout")
    feedback_timeout = LaunchConfiguration("feedback_timeout")
    status_request_period = LaunchConfiguration("status_request_period")
    initial_feedback_attempts = LaunchConfiguration("initial_feedback_attempts")
    mit_kp = LaunchConfiguration("mit_kp")
    mit_kd = LaunchConfiguration("mit_kd")
    start_joint_state_broadcaster = LaunchConfiguration("start_joint_state_broadcaster")
    start_trajectory_controller = LaunchConfiguration("start_trajectory_controller")

    moveit_config_share = FindPackageShare("A6_Y_Moveitconfig")
    xacro_file = PathJoinSubstitution([moveit_config_share, "config", "y1_6.urdf.xacro"])
    initial_positions_file = PathJoinSubstitution([moveit_config_share, "config", "initial_positions.yaml"])
    default_motor_config_file, motor_defaults = _load_default_motor_config()
    controllers_file = PathJoinSubstitution([moveit_config_share, "config", "ros2_controllers.yaml"])

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    xacro_file,
                    " initial_positions_file:=",
                    initial_positions_file,
                    " motor_config_file:=",
                    motor_config_file,
                    " hardware_name:=",
                    hardware_name,
                    " hardware_plugin:=",
                    hardware_plugin,
                    " dry_run:=",
                    dry_run,
                    " can_channel:=",
                    can_channel,
                    " can_bitrate:=",
                    can_bitrate,
                    " tty_baudrate:=",
                    tty_baudrate,
                    " control_mode:=",
                    control_mode,
                    " auto_enable:=",
                    auto_enable,
                    " set_control_mode_on_configure:=",
                    set_control_mode_on_configure,
                    " require_feedback_on_configure:=",
                    require_feedback_on_configure,
                    " allow_partial_feedback:=",
                    allow_partial_feedback,
                    " disable_on_deactivate:=",
                    disable_on_deactivate,
                    " command_timeout:=",
                    command_timeout,
                    " max_start_delta:=",
                    max_start_delta,
                    " receive_timeout:=",
                    receive_timeout,
                    " feedback_timeout:=",
                    feedback_timeout,
                    " status_request_period:=",
                    status_request_period,
                    " initial_feedback_attempts:=",
                    initial_feedback_attempts,
                    " mit_kp:=",
                    mit_kp,
                    " mit_kd:=",
                    mit_kd,
                ]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_name", default_value="A6YArmSystem"),
            DeclareLaunchArgument("hardware_plugin", default_value="a6_y_ros2_control/A6YDamiaoSystem"),
            DeclareLaunchArgument("motor_config_file", default_value=default_motor_config_file),
            DeclareLaunchArgument("dry_run", default_value=motor_defaults["dry_run"]),
            DeclareLaunchArgument("can_channel", default_value=motor_defaults["can_channel"]),
            DeclareLaunchArgument("can_bitrate", default_value=motor_defaults["can_bitrate"]),
            DeclareLaunchArgument("tty_baudrate", default_value=motor_defaults["tty_baudrate"]),
            DeclareLaunchArgument("control_mode", default_value=motor_defaults["control_mode"]),
            DeclareLaunchArgument("auto_enable", default_value=motor_defaults["auto_enable"]),
            DeclareLaunchArgument("set_control_mode_on_configure", default_value=motor_defaults["set_control_mode_on_configure"]),
            DeclareLaunchArgument("require_feedback_on_configure", default_value=motor_defaults["require_feedback_on_configure"]),
            DeclareLaunchArgument("allow_partial_feedback", default_value=motor_defaults["allow_partial_feedback"]),
            DeclareLaunchArgument("disable_on_deactivate", default_value=motor_defaults["disable_on_deactivate"]),
            DeclareLaunchArgument("command_timeout", default_value=motor_defaults["command_timeout"]),
            DeclareLaunchArgument("max_start_delta", default_value=motor_defaults["max_start_delta"]),
            DeclareLaunchArgument("receive_timeout", default_value=motor_defaults["receive_timeout"]),
            DeclareLaunchArgument("feedback_timeout", default_value=motor_defaults["feedback_timeout"]),
            DeclareLaunchArgument("status_request_period", default_value=motor_defaults["status_request_period"]),
            DeclareLaunchArgument("initial_feedback_attempts", default_value=motor_defaults["initial_feedback_attempts"]),
            DeclareLaunchArgument("mit_kp", default_value=motor_defaults["mit_kp"]),
            DeclareLaunchArgument("mit_kd", default_value=motor_defaults["mit_kd"]),
            DeclareLaunchArgument("start_joint_state_broadcaster", default_value="true"),
            DeclareLaunchArgument("start_trajectory_controller", default_value="true"),
            Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen", parameters=[robot_description]),
            Node(package="controller_manager", executable="ros2_control_node", output="screen", parameters=[robot_description, controllers_file]),
            Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"], output="screen", condition=IfCondition(start_joint_state_broadcaster)),
            Node(package="controller_manager", executable="spawner", arguments=["arm_controller", "--controller-manager", "/controller_manager"], output="screen", condition=IfCondition(start_trajectory_controller)),
        ]
    )
