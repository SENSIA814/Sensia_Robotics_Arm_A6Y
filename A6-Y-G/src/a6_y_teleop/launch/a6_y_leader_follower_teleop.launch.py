from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    default_config = PathJoinSubstitution([FindPackageShare("a6_y_teleop"), "config", "teleop.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            Node(
                package="a6_y_teleop",
                executable="leader_follower_teleop",
                name="leader_follower_teleop",
                output="screen",
                parameters=[{"config_file": config_file}],
            ),
        ]
    )
