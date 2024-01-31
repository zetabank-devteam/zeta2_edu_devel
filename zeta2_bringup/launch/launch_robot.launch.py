from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base2laser',
            arguments=['0', '0', '0.135', '0', '0', '0', '1', 'base_link', 'base_scan'],
            remappings=[('/tf', 'tf')]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base2imu',
            arguments=['0', '0', '0.01', '0', '0', '0', '1', 'base_link', 'imu_link'],
            remappings=[('/tf', 'tf')]
        )
    ])