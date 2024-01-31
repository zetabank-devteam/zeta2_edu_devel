from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
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
        )
    ])