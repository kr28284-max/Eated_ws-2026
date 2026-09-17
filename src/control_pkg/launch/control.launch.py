"""Launch the EATED top-level control state machine."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the control launch description."""
    default_config = os.path.join(
        get_package_share_directory('control_pkg'),
        'config',
        'control.yaml',
    )
    config_argument = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Control Node parameter YAML file',
    )
    control_node = Node(
        package='control_pkg',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )
    return LaunchDescription([config_argument, control_node])
