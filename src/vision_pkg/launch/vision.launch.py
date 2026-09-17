"""Launch on-demand vision and the configurable eye-in-hand mount TF."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the vision launch description."""
    config_path = (
        get_package_share_directory('vision_pkg') + '/config/vision.yaml'
    )

    model_path = LaunchConfiguration('model_path')
    publish_mount_tf = LaunchConfiguration('publish_camera_mount_tf')

    declared_arguments = [
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Absolute or user-relative path to the YOLO model',
        ),
        DeclareLaunchArgument(
            'publish_camera_mount_tf',
            default_value='true',
            description='Publish end_effector_link to d435i_link static TF',
        ),
        DeclareLaunchArgument('camera_mount_x', default_value='-0.065'),
        DeclareLaunchArgument('camera_mount_y', default_value='0.0195'),
        DeclareLaunchArgument('camera_mount_z', default_value='0.052'),
        DeclareLaunchArgument('camera_mount_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'camera_mount_parent_frame',
            default_value='end_effector_link',
        ),
        DeclareLaunchArgument(
            'camera_mount_child_frame',
            default_value='d435i_link',
        ),
    ]

    vision_node = Node(
        package='vision_pkg',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[config_path, {'model_path': model_path}],
    )

    camera_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d435i_mount_tf_publisher',
        output='screen',
        condition=IfCondition(publish_mount_tf),
        arguments=[
            '--x',
            LaunchConfiguration('camera_mount_x'),
            '--y',
            LaunchConfiguration('camera_mount_y'),
            '--z',
            LaunchConfiguration('camera_mount_z'),
            '--roll',
            LaunchConfiguration('camera_mount_roll'),
            '--pitch',
            LaunchConfiguration('camera_mount_pitch'),
            '--yaw',
            LaunchConfiguration('camera_mount_yaw'),
            '--frame-id',
            LaunchConfiguration('camera_mount_parent_frame'),
            '--child-frame-id',
            LaunchConfiguration('camera_mount_child_frame'),
        ],
    )

    return LaunchDescription(
        declared_arguments + [camera_mount_tf, vision_node]
    )
