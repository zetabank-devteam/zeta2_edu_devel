from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    zeta2_bringup_pkg = launch_ros.substitutions.FindPackageShare(package='zeta2_bringup').find('zeta2_bringup')
    state_publisher_launch_file_path = os.path.join(zeta2_bringup_pkg, 'launch/zeta2_state_publisher.launch.py')
    lidar_pkg = launch_ros.substitutions.FindPackageShare(package='ldlidar').find('ldlidar')
    lidar_launch_file_path = os.path.join(lidar_pkg, 'launch/ldlidar.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_zeta_if',
            default_value='true',
            description='Launch the zeta_if_node'
        ),
        DeclareLaunchArgument(
            'use_zeta_mc',
            default_value='true',
            description='Launch the zeta_mc_node'
        ),
        DeclareLaunchArgument(
            'use_control',
            default_value='true',
            description='Launch the control.py script'
        ),
        DeclareLaunchArgument(
            'use_odometry',
            default_value='true',
            description='Launch the odometry.py script'
        ),
        Node(
            package='zeta2_bringup',
            executable='zeta_if_node',
            name='zeta_if',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_zeta_if')),
            parameters=[
                {'port_name': '/dev/ttyUSB0'},
                {'baudrate': 115200}
            ]
        ),
        Node(
            package='zeta2_bringup',
            executable='zeta_mc_node',
            name='zeta_mc',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_zeta_mc')),
            parameters=[
                {'port_name': '/dev/ttyUSB1'},
                {'baudrate': 115200}
            ]
        ),
        Node(
            package='zeta2_bringup',
            executable='control.py',
            name='control',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_control'))
        ),
        Node(
            package='zeta2_bringup',
            executable='odometry.py',
            name='odometryNode',
            remappings=[
                        # ('odom', 'zeta/odom'),
                        # ('tf', 'zeta/tf')
                        ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_odometry'))
        ),
        # Include the state publisher launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(state_publisher_launch_file_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_file_path),

            launch_arguments={
                'serial_port': LaunchConfiguration('lidar_serial_port'),
                'lidar_frame': LaunchConfiguration('lidar_frame')
            }.items()
        ),

        # ldlidar 관련 파라미터 선언
        DeclareLaunchArgument(
            'lidar_serial_port',
            default_value='/dev/ttyS0',
            description='Serial port for the LIDAR'
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='base_scan',
            description='Frame ID for the LIDAR'
        ),
        
    ])