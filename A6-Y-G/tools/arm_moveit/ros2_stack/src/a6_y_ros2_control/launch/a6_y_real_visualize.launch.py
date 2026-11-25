from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _load_visualize_defaults():
    defaults = {
        "can_channel": "/dev/ttyACM0",
        "can_bitrate": "1000000",
        "tty_baudrate": "115200",
        "receive_timeout": "0.001",
        "feedback_timeout": "0.5",
        "status_request_period": "0.05",
        "allow_partial_feedback": "true",
    }
    config_path = Path(get_package_share_directory("A6_Y_Moveitconfig")) / "config" / "a6_y_damiao_motors.yaml"
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
        for key in (
            "receive_timeout",
            "feedback_timeout",
            "status_request_period",
        ):
            if key in configured_defaults:
                defaults[key] = _launch_value(configured_defaults[key])
    except Exception:
        pass
    return defaults


def generate_launch_description():
    defaults = _load_visualize_defaults()

    motor_config_file = LaunchConfiguration("motor_config_file")
    dry_run = LaunchConfiguration("dry_run")
    can_channel = LaunchConfiguration("can_channel")
    can_bitrate = LaunchConfiguration("can_bitrate")
    tty_baudrate = LaunchConfiguration("tty_baudrate")
    receive_timeout = LaunchConfiguration("receive_timeout")
    feedback_timeout = LaunchConfiguration("feedback_timeout")
    status_request_period = LaunchConfiguration("status_request_period")
    allow_partial_feedback = LaunchConfiguration("allow_partial_feedback")
    rviz_config = LaunchConfiguration("rviz_config")

    readonly_launch = PathJoinSubstitution(
        [FindPackageShare("a6_y_ros2_control"), "launch", "a6_y_real_readonly.launch.py"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("a6_y_ros2_control"), "config", "a6_y_real_readonly.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_channel", default_value=defaults["can_channel"]),
            DeclareLaunchArgument(
                "motor_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("A6_Y_Moveitconfig"), "config", "a6_y_damiao_motors.yaml"]
                ),
            ),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("can_bitrate", default_value=defaults["can_bitrate"]),
            DeclareLaunchArgument("tty_baudrate", default_value=defaults["tty_baudrate"]),
            DeclareLaunchArgument("receive_timeout", default_value=defaults["receive_timeout"]),
            DeclareLaunchArgument("feedback_timeout", default_value=defaults["feedback_timeout"]),
            DeclareLaunchArgument("status_request_period", default_value=defaults["status_request_period"]),
            DeclareLaunchArgument("allow_partial_feedback", default_value=defaults["allow_partial_feedback"]),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(readonly_launch),
                launch_arguments={
                    "dry_run": dry_run,
                    "can_channel": can_channel,
                    "motor_config_file": motor_config_file,
                    "can_bitrate": can_bitrate,
                    "tty_baudrate": tty_baudrate,
                    "receive_timeout": receive_timeout,
                    "feedback_timeout": feedback_timeout,
                    "status_request_period": status_request_period,
                    "allow_partial_feedback": allow_partial_feedback,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="a6_y_real_readonly_rviz",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
