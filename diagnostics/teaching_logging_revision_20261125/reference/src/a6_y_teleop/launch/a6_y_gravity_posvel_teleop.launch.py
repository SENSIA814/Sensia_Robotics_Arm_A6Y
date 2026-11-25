from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    leader_can_channel = LaunchConfiguration("leader_can_channel")
    follower_can_channel = LaunchConfiguration("follower_can_channel")
    leader_motor_config_file = LaunchConfiguration("leader_motor_config_file")
    follower_motor_config_file = LaunchConfiguration("follower_motor_config_file")
    can_bitrate = LaunchConfiguration("can_bitrate")
    tty_baudrate = LaunchConfiguration("tty_baudrate")
    dry_run = LaunchConfiguration("dry_run")
    leader_command_timeout = LaunchConfiguration("leader_command_timeout")
    follower_command_timeout = LaunchConfiguration("follower_command_timeout")
    max_start_delta = LaunchConfiguration("max_start_delta")
    receive_timeout = LaunchConfiguration("receive_timeout")
    feedback_timeout = LaunchConfiguration("feedback_timeout")
    status_request_period = LaunchConfiguration("status_request_period")
    initial_feedback_attempts = LaunchConfiguration("initial_feedback_attempts")
    leader_mit_kp = LaunchConfiguration("leader_mit_kp")
    leader_mit_kd = LaunchConfiguration("leader_mit_kd")
    follower_controllers_file = LaunchConfiguration("follower_controllers_file")
    follower_controller_name = LaunchConfiguration("follower_controller_name")
    follower_controller_type = LaunchConfiguration("follower_controller_type")
    config_file = LaunchConfiguration("config_file")
    start_teleop = LaunchConfiguration("start_teleop")
    teleop_start_delay = LaunchConfiguration("teleop_start_delay")

    teleop_share = FindPackageShare("a6_y_teleop")
    moveit_config_share = FindPackageShare("A6_Y_Moveitconfig")
    ros2_control_share = FindPackageShare("a6_y_ros2_control")

    bringup_launch = PathJoinSubstitution(
        [ros2_control_share, "launch", "a6_y_dual_arm_teleop_bringup.launch.py"]
    )
    teleop_launch = PathJoinSubstitution(
        [teleop_share, "launch", "a6_y_leader_follower_teleop.launch.py"]
    )
    default_config = PathJoinSubstitution([teleop_share, "config", "teleop.yaml"])
    default_leader_motor_config = PathJoinSubstitution(
        [moveit_config_share, "config", "a6_y_leader_damiao_motors.yaml"]
    )
    default_follower_motor_config = PathJoinSubstitution(
        [moveit_config_share, "config", "a6_y_follower_damiao_motors.yaml"]
    )
    default_follower_controllers_file = PathJoinSubstitution(
        [ros2_control_share, "config", "follower_direct_position_controllers.yaml"]
    )

    dual_arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            "leader_can_channel": leader_can_channel,
            "follower_can_channel": follower_can_channel,
            "leader_motor_config_file": leader_motor_config_file,
            "follower_motor_config_file": follower_motor_config_file,
            "can_bitrate": can_bitrate,
            "tty_baudrate": tty_baudrate,
            "dry_run": dry_run,
            "leader_command_timeout": leader_command_timeout,
            "follower_command_timeout": follower_command_timeout,
            "max_start_delta": max_start_delta,
            "receive_timeout": receive_timeout,
            "feedback_timeout": feedback_timeout,
            "status_request_period": status_request_period,
            "initial_feedback_attempts": initial_feedback_attempts,
            "leader_mit_kp": leader_mit_kp,
            "leader_mit_kd": leader_mit_kd,
            "follower_controllers_file": follower_controllers_file,
            "follower_controller_name": follower_controller_name,
            "follower_controller_type": follower_controller_type,
        }.items(),
    )

    teleop_mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch),
        launch_arguments={"config_file": config_file}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("leader_can_channel", default_value="/dev/ttyACM1"),
            DeclareLaunchArgument("follower_can_channel", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("leader_motor_config_file", default_value=default_leader_motor_config),
            DeclareLaunchArgument("follower_motor_config_file", default_value=default_follower_motor_config),
            DeclareLaunchArgument("can_bitrate", default_value="1000000"),
            DeclareLaunchArgument("tty_baudrate", default_value="115200"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("leader_command_timeout", default_value="2.0"),
            DeclareLaunchArgument("follower_command_timeout", default_value="0.2"),
            DeclareLaunchArgument("max_start_delta", default_value="0.1"),
            DeclareLaunchArgument("receive_timeout", default_value="0.001"),
            DeclareLaunchArgument("feedback_timeout", default_value="0.0"),
            DeclareLaunchArgument("status_request_period", default_value="0.05"),
            DeclareLaunchArgument("initial_feedback_attempts", default_value="60"),
            DeclareLaunchArgument("leader_mit_kp", default_value="0.0"),
            DeclareLaunchArgument("leader_mit_kd", default_value="0.3"),
            DeclareLaunchArgument("follower_controllers_file", default_value=default_follower_controllers_file),
            DeclareLaunchArgument("follower_controller_name", default_value="direct_position_controller"),
            DeclareLaunchArgument(
                "follower_controller_type",
                default_value="position_controllers/JointGroupPositionController",
            ),
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("start_teleop", default_value="true"),
            DeclareLaunchArgument("teleop_start_delay", default_value="5.0"),
            dual_arm_bringup,
            TimerAction(
                period=teleop_start_delay,
                actions=[teleop_mapper],
                condition=IfCondition(start_teleop),
            ),
        ]
    )
