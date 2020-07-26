#!/usr/bin/env python

# -*- coding=utf-8 -*-

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

L = 0.335
d = 0.3050

class VESCSimDriver:
    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    def __init__(self):
        self.sub_cmd_vel = rospy.Subscriber('/vesc_sim/cmd_vel', Twist, self.sub_cmd_vel_callback)

    def sub_cmd_vel_callback(self, data):
        v = data.linear.x * 40.0 / 3.0

        if data.angular.z > 0.602:
            data.angular.z = 0.602
        if data.angular.z < -0.602:
            data.angular.z = -0.602
        
        if math.fabs(data.angular.z) > 1e-3:
            radius = L / math.tan(data.angular.z)
            left_wheel_angle = math.atan(L / (radius - d / 2.0))
            right_wheel_angle = math.atan(L / (radius + d / 2.0))
            
            left_rear_wheel_velocity = v / radius * (radius - d / 2.0)
            right_rear_wheel_velocity = v / radius * (radius + d / 2.0)
            
            left_front_wheel_velocity = left_rear_wheel_velocity / math.cos(left_wheel_angle)
            right_front_wheel_velocity = right_rear_wheel_velocity / math.cos(right_wheel_angle)
        else:
            left_wheel_angle = right_wheel_angle = 0
            left_front_wheel_velocity = right_front_wheel_velocity = v
            left_rear_wheel_velocity = right_rear_wheel_velocity = v

        self.pub_vel_left_rear_wheel.publish(left_rear_wheel_velocity)
        self.pub_vel_right_rear_wheel.publish(right_rear_wheel_velocity)
        self.pub_vel_left_front_wheel.publish(left_front_wheel_velocity)
        self.pub_vel_right_front_wheel.publish(right_front_wheel_velocity)
        self.pub_pos_left_steering_hinge.publish(left_wheel_angle)
        self.pub_pos_right_steering_hinge.publish(right_wheel_angle)

if __name__ == '__main__':
    rospy.init_node('vesc_sim_driver')
    node = VESCSimDriver()
    rospy.spin()