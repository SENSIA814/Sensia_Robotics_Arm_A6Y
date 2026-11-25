from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_channel = LaunchConfiguration("can_channel")
    dry_run = LaunchConfiguration("dry_run")
    auto_enable = LaunchConfiguration("auto_enable")
    command_timeout = LaunchConfiguration("command_timeout")
    max_start_delta = LaunchConfiguration("max_start_delta")
    feedback_timeout = LaunchConfiguration("feedback_timeout")
    home_state = LaunchConfiguration("home_state")
    hold_duration = LaunchConfiguration("hold_duration")
    move_duration = LaunchConfiguration("move_duration")
    home_start_delay = LaunchConfiguration("home_start_delay")
    start_rviz = LaunchConfiguration("start_rviz")
    start_end_effector_server = LaunchConfiguration("start_end_effector_server")

    ros2_control_launch = PathJoinSubstitution(
        [FindPackageShare("a6_y_ros2_control"), "launch", "a6_y_ros2_control.launch.py"]
    )
    move_group_launch = PathJoinSubstitution([FindPackageShare("A6_Y_Moveitconfig"), "launch", "move_group.launch.py"])
    moveit_rviz_launch = PathJoinSubstitution(
        [FindPackageShare("A6_Y_Moveitconfig"), "launch", "moveit_rviz.launch.py"]
    )
    end_effector_launch = PathJoinSubstitution(
        [FindPackageShare("a6_y_moveit_control"), "launch", "end_effector_control.launch.py"]
    )
    srdf_file = PathJoinSubstitution([FindPackageShare("A6_Y_Moveitconfig"), "config", "y1_6.srdf"])

    home_node = Node(
        package="a6_y_ros2_control",
        executable="move_to_group_state.py",
        name="a6_y_home_before_moveit",
        output="screen",
        arguments=[
            "--srdf",
            srdf_file,
            "--group",
            "arm",
            "--state",
            home_state,
            "--hold-duration",
            hold_duration,
            "--move-duration",
            move_duration,
        ],
    )

    def start_moveit_after_home(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=[
                        "A6_Y home trajectory failed with return code ",
                        TextSubstitution(text=str(event.returncode)),
                        "; MoveIt will not be started.",
                    ]
                ),
                Shutdown(reason="A6_Y home trajectory failed"),
            ]
        return [
            LogInfo(msg="A6_Y reached home; starting move_group/RViz/MoveIt interface."),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(move_group_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_rviz_launch), condition=IfCondition(start_rviz)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(end_effector_launch), condition=IfCondition(start_end_effector_server)
            ),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_channel", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("auto_enable", default_value="true"),
            DeclareLaunchArgument("command_timeout", default_value="0.0"),
            DeclareLaunchArgument("max_start_delta", default_value="0.1"),
            DeclareLaunchArgument("feedback_timeout", default_value="0.0"),
            DeclareLaunchArgument("home_state", default_value="home"),
            DeclareLaunchArgument("hold_duration", default_value="2.0"),
            DeclareLaunchArgument("move_duration", default_value="20.0"),
            DeclareLaunchArgument("home_start_delay", default_value="4.0"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_end_effector_server", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ros2_control_launch),
                launch_arguments={
                    "dry_run": dry_run,
                    "auto_enable": auto_enable,
                    "can_channel": can_channel,
                    "control_mode": "pos_vel",
                    "command_timeout": command_timeout,
                    "max_start_delta": max_start_delta,
                    "feedback_timeout": feedback_timeout,
                    "allow_partial_feedback": "false",
                    "start_joint_state_broadcaster": "true",
                    "start_trajectory_controller": "true",
                }.items(),
            ),
            TimerAction(period=home_start_delay, actions=[home_node]),
            RegisterEventHandler(OnProcessExit(target_action=home_node, on_exit=start_moveit_after_home)),
        ]
    )
