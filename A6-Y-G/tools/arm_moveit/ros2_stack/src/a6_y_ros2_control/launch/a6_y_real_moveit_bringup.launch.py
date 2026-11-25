from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_channel = LaunchConfiguration("can_channel")
    auto_enable = LaunchConfiguration("auto_enable")
    command_timeout = LaunchConfiguration("command_timeout")
    max_start_delta = LaunchConfiguration("max_start_delta")
    feedback_timeout = LaunchConfiguration("feedback_timeout")
    start_rviz = LaunchConfiguration("start_rviz")
    start_end_effector_server = LaunchConfiguration("start_end_effector_server")

    ros2_control_launch = PathJoinSubstitution([FindPackageShare("a6_y_ros2_control"), "launch", "a6_y_ros2_control.launch.py"])
    move_group_launch = PathJoinSubstitution([FindPackageShare("A6_Y_Moveitconfig"), "launch", "move_group.launch.py"])
    moveit_rviz_launch = PathJoinSubstitution([FindPackageShare("A6_Y_Moveitconfig"), "launch", "moveit_rviz.launch.py"])
    end_effector_launch = PathJoinSubstitution([FindPackageShare("a6_y_moveit_control"), "launch", "end_effector_control.launch.py"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_channel", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("auto_enable", default_value="true"),
            DeclareLaunchArgument("command_timeout", default_value="0.0"),
            DeclareLaunchArgument("max_start_delta", default_value="0.1"),
            DeclareLaunchArgument("feedback_timeout", default_value="0.0"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("start_end_effector_server", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ros2_control_launch),
                launch_arguments={
                    "dry_run": "false",
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
            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(PythonLaunchDescriptionSource(move_group_launch)),
                    IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_rviz_launch), condition=IfCondition(start_rviz)),
                    IncludeLaunchDescription(PythonLaunchDescriptionSource(end_effector_launch), condition=IfCondition(start_end_effector_server)),
                ],
            ),
        ]
    )
