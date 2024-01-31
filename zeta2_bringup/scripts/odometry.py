#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import os
import math
from numpy import interp

'''
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Float32MultiArray

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion
'''
import rclpy
from rclpy.node import Node

# tf_transformations is a separate package in ROS 2 for transformations
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from tf2_ros import TransformBroadcaster
from std_msgs.msg import UInt8MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion, TransformStamped

class OdomPose(object):
    x = 0.0
    y = 0.0
    theta = 0.0
    timestamp = 0
    pre_timestamp = 0


class OdomVel(object):
    x = 0.0
    y = 0.0
    w = 0.0


class Joint(object):
    joint_name = ['wheel_left_joint', 'wheel_right_joint']
    joint_pos = [0.0, 0.0]
    joint_vel = [0.0, 0.0]


class ComplementaryFilter():
    def __init__(self, logger=None):
        self.logger = logger
        self.theta = 0.
        self.pre_theta = 0.
        self.wheel_ang = 0.
        self.filter_coef = 2.5
        self.gyro_bias = 0.
        self.count_for_gyro_bias = 110

    def loginfo(self, message):
        self.logger.info(message)

    def gyro_calibration(self, gyro):
        self.count_for_gyro_bias -= 1

        if self.count_for_gyro_bias > 100:
            return "Prepare for gyro_calibration"

        self.gyro_bias += gyro
        if self.count_for_gyro_bias == 1:
            self.gyro_bias /= 100
            self.loginfo('Complete: Gyro calibration')
            return "gyro_calibration OK"

        return "During gyro_calibration"

    def calc_filter(self, gyro, d_time):

        if self.count_for_gyro_bias != 1:
            tmp = self.gyro_calibration(gyro)
            return 0

        gyro -= self.gyro_bias

        self.pre_theta = self.theta
        temp = -1 / self.filter_coef * (-self.wheel_ang + self.pre_theta) + gyro
        self.theta = self.pre_theta + temp * d_time

        # self.logger.info(f"{self.theta * 180 / 3.141592653589793}, {self.wheel_ang * 180 / 3.141592653589793}, {gyro}, {d_time}")
        return self.theta


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.sub_imu = self.create_subscription(Imu, 'imu', self.sub_imu_output, 10)
        self.sub_raw = self.create_subscription(UInt8MultiArray, 'motor_driver_serial_output', self.sub_raw_output, 10)

        self.odom_pose = OdomPose()
        self.odom_vel = OdomVel()
        self.joint = Joint()
        self.v = 0.
        self.w = 0.
        self.z = 0.
        self.calc_yaw = ComplementaryFilter(self.get_logger())

        self.odom_pose.pre_timestamp = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.odom_debug_pub = self.create_publisher(Float32MultiArray, 'odom_debug', 10)
        self.motor_debug_pub = self.create_publisher(Float32MultiArray, 'motor_driver_serial_output_debug', 10)
        self.joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.odom_broadcaster = TransformBroadcaster(self)

        self.check = False

        self.vr = 0.
        self.vl = 0.

        self.m1_pps = 0.
        self.m2_pps = 0.
        self.only_one_imu = True
        self.imu_prev_theta = 0.
        self.yaw = 0.
    

    def bytes_to_int(self, bytes):
        result = 0
        for b in bytes:
            result = result * 256 + int(b)
        return result


    def sub_raw_output(self, data):
        if len(data.data) != 12:
            return
        
        msg =  data.data

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
        d2 = msg[9]
        w = self.bytes_to_int(msg[10:]) / 1000.0
        '''

          ___cw___ccw___up_______down
        d |   2    1    3         0
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

        # log_message = "D: %s D2: %s motor1_pps: %s motor2_pps: %s linear: %s angular: %s"
        # self.logging.info(log_message, d, d2, motor1_pps, motor2_pps, v, w)

        debug_array = [self.v, self.w]
        debug_data = Float32MultiArray(data=debug_array)
        self.motor_debug_pub.publish(debug_data)  # Publish imu data to update odometry

    def sub_imu_output(self, msg):
        orientation_in_quaternion = msg.orientation
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation_in_quaternion.x, orientation_in_quaternion.y, orientation_in_quaternion.z,
             orientation_in_quaternion.w])

        '''  로봇 회전각 정의
                         전방기준
             0~+180(CCW)   |   0~-180 (CW)
                          로봇
        '''
        yaw = yaw * -1.0  # 반대로 들어와 부호 바꿈

        orientation = [0, 0, 0, 0]
        orientation[0] = msg.orientation.w
        orientation[3] = msg.orientation.z
        # imu_theta는 yaw와 비교하기 위해서
        imu_theta = math.atan2(orientation[1] * orientation[2] + orientation[0] * orientation[3],
                               0.5 - orientation[2] * orientation[2] - orientation[3] * orientation[3])

        # 시간의 변화량
        self.odom_pose.timestamp = self.get_clock().now()
        # dt = (self.odom_pose.timestamp - self.odom_pose.pre_timestamp).to_sec()
        dt_duration = self.odom_pose.timestamp - self.odom_pose.pre_timestamp
        dt = dt_duration.nanoseconds / 1e9  # Convert nanoseconds to seconds
        
        # 이동한 거리
        delta_translation = dt * self.v

        #  imu 사용시
        if self.only_one_imu:
            self.only_one_imu = False
            self.imu_prev_theta = yaw
        # 이동한 각
        delta_angle = yaw - self.imu_prev_theta

        # imu 미사용시
        # 1. delta_angle = dt * msg.angular_velocity.z
        # 2. delta_angle = dt * self.z

        # filter version1
        if math.fabs(delta_angle) > 0.0001:
            self.odom_pose.theta += delta_angle
            # rospy.loginfo('theta: {} yaw(degree): {} test(degree): {}'.format(self.odom_pose.theta, math.degrees(yaw),math.degrees(imu_theta)))

        debug_array = [math.degrees(self.odom_pose.theta), math.degrees(yaw), math.degrees(imu_theta)]
        debug_data = Float32MultiArray(data=debug_array)
        self.odom_debug_pub.publish(debug_data)  # Publish imu data to update odometry

        # filter version2
        # self.calc_yaw.wheel_ang += dt * self.w
        # self.odom_pose.theta = self.calc_yaw.calc_filter(yaw, dt)

        # 각도의 이동량
        # wheel_ang 사용시 주석 or filter version1 사용시
        # self.odom_pose.theta += delta_angle
        # heading 방향으로 delta_translation 만큼 이동했을때
        # 위치를 x,y 좌표로 나타내면 아래와 같이 계산할 수 있다.
        self.odom_pose.x += delta_translation * math.cos(self.odom_pose.theta + (delta_angle / 2.0))
        self.odom_pose.y += delta_translation * math.sin(self.odom_pose.theta + (delta_angle / 2.0))
        self.odom_vel.x = self.v
        self.odom_vel.y = 0.
        self.odom_vel.w = self.w

        # publish odometry
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.header.stamp = self.get_clock().now().to_msg()
        quaternion = quaternion_from_euler(0.0, 0.0, self.odom_pose.theta)
        odom.pose.pose = Pose(
            position=Point(x=self.odom_pose.x, y=self.odom_pose.y, z=0.0),
            orientation=Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        )        
        odom.twist.twist = Twist(
            linear=Vector3(x=self.odom_vel.x, y=self.odom_vel.y, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.odom_vel.w)
        )        
        self.odom_pub.publish(odom)

        # Create a TransformStamped message
        transform_stamped = TransformStamped()

        # Fill in the header
        transform_stamped.header.stamp = odom.header.stamp  # Use the same timestamp as in the odom message
        transform_stamped.header.frame_id = odom.header.frame_id  # "odom"
        transform_stamped.child_frame_id = odom.child_frame_id  # "base_link"

        # Fill in the transform
        transform_stamped.transform.translation.x = self.odom_pose.x
        transform_stamped.transform.translation.y = self.odom_pose.y
        transform_stamped.transform.translation.z = 0.0  # Assuming z is 0 as per your application
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]

        # Send the transform
        self.odom_broadcaster.sendTransform(transform_stamped)


        # imu 사용시
        self.imu_prev_theta = yaw
        self.odom_pose.pre_timestamp = self.odom_pose.timestamp

        # publishing joint_state
        wheel_radius = 0.035
        wheel_base = 0.174
        wheel_ang_vel_left = (self.v - (wheel_base / 2.0) * self.w) / wheel_radius
        wheel_ang_vel_right = (self.v + (wheel_base / 2.0) * self.w) / wheel_radius
        # self.vl, self.vr must compare
        tmp_left = interp(self.m2_pps, [0, 630], [-3.14, 3.14])
        tmp_right = interp(self.m1_pps, [0, 630], [-3.14, 3.14])
        self.joint.joint_pos = [tmp_left, tmp_right]  # wheel left, wheel right
        self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]

        joint_states = JointState()
        joint_states.header.frame_id = "base_link"
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = self.joint.joint_name
        joint_states.position = self.joint.joint_pos
        joint_states.velocity = self.joint.joint_vel
        joint_states.effort = []

        self.joint_states_pub.publish(joint_states)

def main(args=None):
    rclpy.init(args=args)# Initialize ROS 2 communication
    odo = OdometryNode() # Create an instance of your Control node
    # In ROS 2, spin is called on an instance of your node
    rclpy.spin(odo)
    # Shutdown and cleanup
    odo.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()