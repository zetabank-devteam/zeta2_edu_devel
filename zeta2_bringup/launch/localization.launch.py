from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    zeta2_bringup_dir = get_package_share_directory('zeta2_bringup')
    config_file_path = os.path.join(zeta2_bringup_dir, 'config/zeta_efk.yaml')

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
        # launch_ros.actions.Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[config_file_path],
        #     remappings=[('odometry/filtered', 'zeta/odom/filtered'),
        #                 ('/tf', 'tf')],
        # ),
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