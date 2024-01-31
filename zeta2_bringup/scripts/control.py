#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.pub = self.create_publisher(UInt8MultiArray, "motor_driver_serial_input", 1)
        self.sub_raw = self.create_subscription(UInt8MultiArray, "motor_driver_serial_output", self.sub_raw_output, 1)
        self.sub_vel = self.create_subscription(Twist, "cmd_vel", self.sub_cmd_vel, 1)
        self.motor_debug_pub = self.create_publisher(Float32MultiArray, "motor_driver_serial_output_debug", 10)

    def sub_raw_output(self, data):
        if len(data.data) != 12:
            return
        msg = data.data  # Use the array directly
        
        if msg[1] != 3:  # mod 3
            return
        
        '''
        [0   1   2   3   4   5   6   7   8   9  10  11]
        [   mod dir  ---pps  ---pps  -----v  dir ----w
        dir : 0(stop), 1(left), 2(right), 0(forward), 3(backward)
        '''
        '''
            * max speed: 0xf0 250
            * min speed: 0x50 80
            * ID: 0x01
            * 전진: 0x00
            * 후진: 0x03
            * 왼쪽: 0x01 (motor01)    => 0x02
            * 오른쪽: 0x02 (motor02) => 0x01
        '''

        '''  로봇 회전각 정의
                         전방기준
             0~+180(CCW)   |   0~-180 (CW)
                          로봇
        '''


        ''' zeta2
                 3 +lv
       +av 1(ccw)   2(cw) -av
                 1 -lv
        '''


        d = msg[2]  # dir

        motor1_pps = self.bytes_to_int(msg[3:5]) / 10.0  # wheel right
        motor2_pps = self.bytes_to_int(msg[5:7]) / 10.0  # wheel left

        v = self.bytes_to_int(msg[7:9]) / 1000.0
        # d2 = msg[9]
        w = self.bytes_to_int(msg[10:]) / 1000.0
        '''

          ___cw___ccw___up_______down
        d |   2    1    2         0
        d2|   0    0    0 or 1   0 or 1
        '''
        if d == 2:  # d2는 의미 없는 필드가됨. cw: -angular velocity
            w = w * -1.0
        if d == 0:  # backward: -linear velocity
            v = v * -1.0

        self.v = v
        self.w = w
        self.check = True
        self.m1_pps = motor1_pps  # wheel right
        self.m2_pps = motor2_pps  # wheel left
        # self.get_logger().info(f"D: {d} D2: {d2} motor1_pps: {motor1_pps} motor2_pps: {motor2_pps} linear: {v} angular: {w}")
        debug_array = [self.v, self.w]
        debug_data = Float32MultiArray(data=debug_array)
        self.motor_debug_pub.publish(debug_data)  # Publish imu data to update odometry


    def int_to_bytes(self, value, length):
        result = []
        for i in range(0, length):
            result.append(value >> (i * 8) & 0xff)
        result.reverse()
        return result


    def bytes_to_int(self, bytes):
        result = 0
        for b in bytes:
            result = result * 256 + int(b)
        return result
    

    def sub_cmd_vel(self, cmd_vel_msg):
        data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        v = cmd_vel_msg.linear.x  # m/s 
        w = cmd_vel_msg.angular.z   # rad/s

        # self.get_logger().info(f"{time.time()} => linear: {v}, angular: {w}")

        L = 0.174
        # R = 0.035

        Vl = v - w * L / 2.0
        Vr = v + w * L / 2.0

        ID = 0x01
        data[0] = ID
        D = 0x00
        if Vl > 0:
            D = D | 0x02
        if Vr > 0:
            D = D | 0x01

        # ID = 0x01
        # data[0] = ID
        # D = 0x00

        # if Vl > 0 and Vr > 0:
        #     D = 0x03  # Both Vl and Vr are greater than 0
        # elif Vl > 0:
        #     D = 0x01  # Only Vl is greater than 0
        # elif Vr > 0:
        #     D = 0x02  # Only Vr is greater than 0
        # No need for an explicit else condition, as D is already set to 0x00 by default

        # self.get_logger().info(f"D :{D}")

        data[1] = D
        max_vel = 0.30  # 0.30m/second limit velocity

        Vlt = abs(max(-max_vel * 1000.0, min(max_vel * 1000.0, Vl * 1000.0)))  # abs limit max
        Vrt = abs(max(-max_vel * 1000.0, min(max_vel * 1000.0, Vr * 1000.0)))

        # Vlt = abs(Vl * 1000.0)
        # Vrt = abs(Vr * 1000.0)
                
        if Vl > 1:
            Vrt = Vrt - (Vl - 1) * 1000
        if Vl < -1:
            Vrt = Vrt + (Vl + 1) * 1000
        if Vr > 1:
            Vlt = Vlt - (Vr - 1) * 1000
        if Vr < -1:
            Vlt = Vlt + (Vr + 1) * 1000

        # self.get_logger().info(f"Vl :{Vl}, Vr : {Vr}, Vlt: {int(Vlt)}, Vrt: {int(Vrt)}")

        vls = self.int_to_bytes(int(Vlt), 2)  # uint16 to bytes
        vrs = self.int_to_bytes(int(Vrt), 2)
        data[2] = vls[0]  # high
        data[3] = vls[1]  # low
        data[4] = vrs[0]
        data[5] = vrs[1]
        message = UInt8MultiArray()
        message.data = data

        # self.get_logger().info(f"input :{message}")
        
        # self.get_logger().info("vl hi: {}, vl lo: {}, vr hi: {}, vr lo: {}, v: {}, w: {}, Vl: {}, Vr: {}".format(data[2], ...)
        # data[3], data[4], data[5], v, w, Vl * 1000, Vr * 1000)) rospy.loginfo("v: {}, w: {}, Vl: {},
        # Vr: {}".format(v, w, Vl * 1000, Vr * 1000))

        self.pub.publish(message)


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 communication
    ctrl = ControlNode()  # Create an instance of your Control node
    rclpy.spin(ctrl)
    # Shutdown and cleanup
    ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()