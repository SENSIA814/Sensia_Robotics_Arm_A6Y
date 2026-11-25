"""Pure URDF visualization. Intentionally contains no controller_manager."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share=Path(get_package_share_directory('a6_y_vr_teleop'))
    config=LaunchConfiguration('config').perform(context)
    cfg=yaml.safe_load(Path(config).read_text())
    if os.environ.get('ROS_DOMAIN_ID') != str(cfg['domain_id']) or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Start with scripts/run_vr_rviz.bash or set the configured isolated domain and ROS_LOCALHOST_ONLY=1')
    urdf=Path(get_package_share_directory('a5_description'))/'urdf/A5.urdf'
    limits=Path(get_package_share_directory('a6_y_controllers'))/'config/a5_motion_limits.yaml'
    return [
        Node(package='a6_y_vr_teleop',executable='virtual_arm',namespace='arm6_vr',output='screen',
            parameters=[dict(config_path=config,urdf_path=str(urdf),limits_path=str(limits),
                record_file=LaunchConfiguration('record_file').perform(context))]),
        Node(package='robot_state_publisher',executable='robot_state_publisher',namespace='arm6_vr',
            parameters=[dict(robot_description=urdf.read_text(),frame_prefix='vr_a5/',
                             publish_frequency=float(cfg['update_rate']))]),
        Node(package='rviz2',executable='rviz2',name='rviz_vr',namespace='arm6_vr',
            arguments=['-d',str(share/'config/a5_vr.rviz')],condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    share=Path(get_package_share_directory('a6_y_vr_teleop'))
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(share/'config/a5_vr.yaml')),
        DeclareLaunchArgument('use_rviz',default_value='true'),
        DeclareLaunchArgument('record_file',default_value=''),
        OpaqueFunction(function=setup),
    ])
