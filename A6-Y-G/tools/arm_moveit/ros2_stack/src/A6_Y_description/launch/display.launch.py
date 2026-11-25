import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('y1_6')
    model_path = os.path.join(package_share, 'urdf', 'y1.6.urdf')

    with open(model_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Start joint_state_publisher_gui.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(use_gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])