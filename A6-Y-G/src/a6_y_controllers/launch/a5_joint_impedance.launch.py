"""Hold the measured six-joint configuration using the existing A6-Y-G communication stack."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("a6_y_controllers"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "controllers_file", default_value=str(share / "config/a5_joint_impedance_controllers.yaml")),
        DeclareLaunchArgument(
            "gravity_parameters_file", default_value=str(share / "config/a5_gravity_controllers.yaml"),
            description="Reuse the commissioned gravity scales, signs, joint damping and total torque caps"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/a5_gravity_comp.launch.py")),
            launch_arguments={
                "controller_name": "a6_y_joint_impedance_controller",
                "controllers_file": LaunchConfiguration("controllers_file"),
                "gravity_parameters_file": LaunchConfiguration("gravity_parameters_file"),
            }.items(),
        ),
    ])
