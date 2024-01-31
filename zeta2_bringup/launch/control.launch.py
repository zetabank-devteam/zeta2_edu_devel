from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='zeta2_bringup',
            executable='control.py',
            name='control',
            output='screen',
        ),
    ])