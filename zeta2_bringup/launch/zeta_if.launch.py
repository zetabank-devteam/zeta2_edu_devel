import os
from launch import LaunchDescription
from launch_ros.actions import Node

def find_first_usb_device():
    # List all files in /dev
    devices = os.listdir('/dev')
    
    # Filter for ttyUSB* devices
    usb_devices = sorted([dev for dev in devices if dev.startswith('ttyUSB')])
    
    if not usb_devices:
        raise RuntimeError("No /dev/ttyUSB* devices found")
    
    # Return the first ttyUSB device
    return f"/dev/{usb_devices[0]}"

def generate_launch_description():
    first_usb_device = find_first_usb_device()

    return LaunchDescription([
        Node(
            package='zeta2_bringup',
            executable='zeta_if_node',
            name='zeta_if',
            output='screen',
            parameters=[
                {'port_name': first_usb_device},
                {'baudrate': 115200}
            ]
        ),
    ])