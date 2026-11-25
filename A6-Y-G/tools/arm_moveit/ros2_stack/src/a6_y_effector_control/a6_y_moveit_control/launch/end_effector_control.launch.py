from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("y1_6", package_name="A6_Y_Moveitconfig").to_moveit_configs()
    return LaunchDescription([
        Node(
            package="a6_y_moveit_control",
            executable="end_effector_control_server",
            name="end_effector_control_server",
            output="screen",
            parameters=[moveit_config.to_dict()],
        )
    ])
