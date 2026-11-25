from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_channel = LaunchConfiguration("can_channel")
    motor_config_file = LaunchConfiguration("motor_config_file")
    dry_run = LaunchConfiguration("dry_run")
    allow_partial_feedback = LaunchConfiguration("allow_partial_feedback")

    ros2_control_share = FindPackageShare("a6_y_ros2_control")
    moveit_config_share = FindPackageShare("A6_Y_Moveitconfig")
    default_motor_config = PathJoinSubstitution(
        [moveit_config_share, "config", "a6_y_follower_damiao_motors.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_channel", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("motor_config_file", default_value=default_motor_config),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("allow_partial_feedback", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([ros2_control_share, "launch", "a6_y_real_visualize.launch.py"])
                ),
                launch_arguments={
                    "can_channel": can_channel,
                    "motor_config_file": motor_config_file,
                    "dry_run": dry_run,
                    "allow_partial_feedback": allow_partial_feedback,
                }.items(),
            )
        ]
    )
