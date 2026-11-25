from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motor_config_file = LaunchConfiguration("motor_config_file")
    dry_run = LaunchConfiguration("dry_run")
    can_channel = LaunchConfiguration("can_channel")
    can_bitrate = LaunchConfiguration("can_bitrate")
    tty_baudrate = LaunchConfiguration("tty_baudrate")
    receive_timeout = LaunchConfiguration("receive_timeout")
    feedback_timeout = LaunchConfiguration("feedback_timeout")
    status_request_period = LaunchConfiguration("status_request_period")
    allow_partial_feedback = LaunchConfiguration("allow_partial_feedback")
    base_launch = PathJoinSubstitution([FindPackageShare("a6_y_ros2_control"), "launch", "a6_y_ros2_control.launch.py"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("can_channel", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument(
                "motor_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("A6_Y_Moveitconfig"), "config", "a6_y_damiao_motors.yaml"]
                ),
            ),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("can_bitrate", default_value="1000000"),
            DeclareLaunchArgument("tty_baudrate", default_value="115200"),
            DeclareLaunchArgument("receive_timeout", default_value="0.001"),
            DeclareLaunchArgument("feedback_timeout", default_value="0.5"),
            DeclareLaunchArgument("status_request_period", default_value="0.05"),
            DeclareLaunchArgument("allow_partial_feedback", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "dry_run": dry_run,
                    "auto_enable": "false",
                    "set_control_mode_on_configure": "false",
                    "require_feedback_on_configure": "true",
                    "allow_partial_feedback": allow_partial_feedback,
                    "start_trajectory_controller": "false",
                    "motor_config_file": motor_config_file,
                    "can_channel": can_channel,
                    "can_bitrate": can_bitrate,
                    "tty_baudrate": tty_baudrate,
                    "receive_timeout": receive_timeout,
                    "feedback_timeout": feedback_timeout,
                    "status_request_period": status_request_period,
                }.items(),
            ),
        ]
    )
