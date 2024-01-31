from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    zeta2_bringup_dir = get_package_share_directory('zeta2_bringup')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='zeta2_bringup',
            executable='odometry.py',
            name='odometry',
            remappings=[
                        # ('odom', 'zeta/odom'),
                        # ('tf', 'zeta/tf')
                        ],
            output='screen',
        ),
    ])