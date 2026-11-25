from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def robot_description(
    hardware_name,
    motor_config_file,
    initial_positions_file,
    xacro_file,
    *,
    dry_run,
    can_channel,
    can_bitrate,
    tty_baudrate,
    control_mode,
    auto_enable,
    command_timeout,
    max_start_delta,
    receive_timeout,
    feedback_timeout,
    status_request_period,
    initial_feedback_attempts,
    mit_kp,
    mit_kd,
):
    return {
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
                    " hardware_plugin:=a6_y_ros2_control/A6YDamiaoSystem",
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
                    " set_control_mode_on_configure:=false",
                    " require_feedback_on_configure:=true",
                    " allow_partial_feedback:=false",
                    " disable_on_deactivate:=true",
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

    moveit_config_share = FindPackageShare("A6_Y_Moveitconfig")
    ros2_control_share = FindPackageShare("a6_y_ros2_control")
    xacro_file = PathJoinSubstitution([moveit_config_share, "config", "y1_6.urdf.xacro"])
    initial_positions_file = PathJoinSubstitution([moveit_config_share, "config", "initial_positions.yaml"])
    default_leader_motor_config = PathJoinSubstitution(
        [moveit_config_share, "config", "a6_y_leader_damiao_motors.yaml"]
    )
    default_follower_motor_config = PathJoinSubstitution(
        [moveit_config_share, "config", "a6_y_follower_damiao_motors.yaml"]
    )
    leader_controllers_file = PathJoinSubstitution(
        [ros2_control_share, "config", "leader_gravity_comp_controllers.yaml"]
    )
    default_follower_controllers_file = PathJoinSubstitution(
        [ros2_control_share, "config", "follower_direct_position_controllers.yaml"]
    )

    leader_description = robot_description(
        "A6YLeaderSystem",
        leader_motor_config_file,
        initial_positions_file,
        xacro_file,
        dry_run=dry_run,
        can_channel=leader_can_channel,
        can_bitrate=can_bitrate,
        tty_baudrate=tty_baudrate,
        control_mode="mit",
        auto_enable="true",
        command_timeout=leader_command_timeout,
        max_start_delta=max_start_delta,
        receive_timeout=receive_timeout,
        feedback_timeout=feedback_timeout,
        status_request_period=status_request_period,
        initial_feedback_attempts=initial_feedback_attempts,
        mit_kp=leader_mit_kp,
        mit_kd=leader_mit_kd,
    )
    follower_description = robot_description(
        "A6YFollowerSystem",
        follower_motor_config_file,
        initial_positions_file,
        xacro_file,
        dry_run=dry_run,
        can_channel=follower_can_channel,
        can_bitrate=can_bitrate,
        tty_baudrate=tty_baudrate,
        control_mode="pos_vel",
        auto_enable="true",
        command_timeout=follower_command_timeout,
        max_start_delta=max_start_delta,
        receive_timeout=receive_timeout,
        feedback_timeout=feedback_timeout,
        status_request_period=status_request_period,
        initial_feedback_attempts=initial_feedback_attempts,
        mit_kp="0.0",
        mit_kd="0.3",
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
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace="leader",
                output="screen",
                parameters=[leader_description, leader_controllers_file],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/leader/controller_manager",
                    "--param-file",
                    leader_controllers_file,
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "a6_y_gravity_comp_controller",
                    "--controller-manager",
                    "/leader/controller_manager",
                    "--param-file",
                    leader_controllers_file,
                    "--controller-type",
                    "a6_y_controllers/A6YGravityCompController",
                ],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace="follower",
                output="screen",
                parameters=[follower_description, follower_controllers_file],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/follower/controller_manager",
                    "--param-file",
                    follower_controllers_file,
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    follower_controller_name,
                    "--controller-manager",
                    "/follower/controller_manager",
                    "--param-file",
                    follower_controllers_file,
                    "--controller-type",
                    follower_controller_type,
                ],
            ),
        ]
    )
