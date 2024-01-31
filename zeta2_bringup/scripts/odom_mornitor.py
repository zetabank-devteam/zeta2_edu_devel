#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class SimpleOdomSubscriber(Node):

    def __init__(self):
        super().__init__('simple_odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")
        self.get_logger().info(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

def main(args=None):
    rclpy.init(args=args)
    simple_odom_subscriber = SimpleOdomSubscriber()
    rclpy.spin(simple_odom_subscriber)
    simple_odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
