from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    hardware_name = LaunchConfiguration("hardware_name")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
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
    mit_kp = LaunchConfiguration("mit_kp")
    mit_kd = LaunchConfiguration("mit_kd")
    joint_name = LaunchConfiguration("joint_name")
    motor_type = LaunchConfiguration("motor_type")
    slave_id = LaunchConfiguration("slave_id")
    master_id = LaunchConfiguration("master_id")
    sign = LaunchConfiguration("sign")
    offset = LaunchConfiguration("offset")
    initial_position = LaunchConfiguration("initial_position")
    min_position = LaunchConfiguration("min_position")
    max_position = LaunchConfiguration("max_position")
    max_velocity = LaunchConfiguration("max_velocity")
    controllers_file = LaunchConfiguration("controllers_file")
    start_joint_state_broadcaster = LaunchConfiguration("start_joint_state_broadcaster")
    start_trajectory_controller = LaunchConfiguration("start_trajectory_controller")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("a6_y_ros2_control"), "config", "a6_y_single_motor.urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    xacro_file,
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
                    " mit_kp:=",
                    mit_kp,
                    " mit_kd:=",
                    mit_kd,
                    " joint_name:=",
                    joint_name,
                    " motor_type:=",
                    motor_type,
                    " slave_id:=",
                    slave_id,
                    " master_id:=",
                    master_id,
                    " sign:=",
                    sign,
                    " offset:=",
                    offset,
                    " initial_position:=",
                    initial_position,
                    " min_position:=",
                    min_position,
                    " max_position:=",
                    max_position,
                    " max_velocity:=",
                    max_velocity,
                ]
            ),
            value_type=str,
        )
    }

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[robot_description, controllers_file],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
            condition=IfCondition(start_joint_state_broadcaster),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["single_motor_controller", "--controller-manager", "/controller_manager"],
            output="screen",
            condition=IfCondition(start_trajectory_controller),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_name", default_value="A6YSingleMotorSystem"),
            DeclareLaunchArgument("hardware_plugin", default_value="a6_y_ros2_control/A6YDamiaoSystem"),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("can_channel", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("can_bitrate", default_value="1000000"),
            DeclareLaunchArgument("tty_baudrate", default_value="115200"),
            DeclareLaunchArgument("control_mode", default_value="pos_vel"),
            DeclareLaunchArgument("auto_enable", default_value="false"),
            DeclareLaunchArgument("set_control_mode_on_configure", default_value="false"),
            DeclareLaunchArgument("require_feedback_on_configure", default_value="true"),
            DeclareLaunchArgument("allow_partial_feedback", default_value="false"),
            DeclareLaunchArgument("disable_on_deactivate", default_value="true"),
            DeclareLaunchArgument("command_timeout", default_value="0.2"),
            DeclareLaunchArgument("max_start_delta", default_value="0.25"),
            DeclareLaunchArgument("receive_timeout", default_value="0.001"),
            DeclareLaunchArgument("feedback_timeout", default_value="0.5"),
            DeclareLaunchArgument("status_request_period", default_value="0.05"),
            DeclareLaunchArgument("mit_kp", default_value="0.0"),
            DeclareLaunchArgument("mit_kd", default_value="0.5"),
            DeclareLaunchArgument("joint_name", default_value="joint1"),
            DeclareLaunchArgument("motor_type", default_value="DM4340P"),
            DeclareLaunchArgument("slave_id", default_value="0x01"),
            DeclareLaunchArgument("master_id", default_value="0x11"),
            DeclareLaunchArgument("sign", default_value="1.0"),
            DeclareLaunchArgument("offset", default_value="0.0"),
            DeclareLaunchArgument("initial_position", default_value="0.0"),
            DeclareLaunchArgument("min_position", default_value="-0.25"),
            DeclareLaunchArgument("max_position", default_value="0.25"),
            DeclareLaunchArgument("max_velocity", default_value="0.1"),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("a6_y_ros2_control"), "config", "single_motor_ros2_controllers.yaml"]
                ),
            ),
            DeclareLaunchArgument("start_joint_state_broadcaster", default_value="true"),
            DeclareLaunchArgument("start_trajectory_controller", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
