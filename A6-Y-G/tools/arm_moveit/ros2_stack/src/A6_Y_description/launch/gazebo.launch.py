import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('y1_6')
    gazebo_share = get_package_share_directory('gazebo_ros')
    model_path = os.path.join(package_share, 'urdf', 'y1.6.urdf')

    with open(model_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
            )
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'base_link',
                'base_footprint',
            ],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-entity', 'y1_6',
                '-file', model_path,
            ],
        ),
    ])