#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros

class OdometryNode:
    pub_twist = rospy.Publisher('/racecar/wheel_velocity', TwistWithCovarianceStamped, queue_size=1)
    wheel_dia = 0.14605

    def __init__(self):
        self.last_recieved_left_axle = 0
        self.last_recieved_right_axle = 0
        self.last_recieved_stamp = None


        rospy.Timer(rospy.Duration(0.02), self.timer_callback) # 50 Hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        rospy.Subscriber('/racecar/joint_states', JointState, self.sub_wheel_update)

    def sub_wheel_update(self, msg):
        try:
            leftArrayIndex = msg.name.index('left_rear_axle')
            rightArrayIndex = msg.name.index('right_rear_axle')
        except ValueError as e:
            pass
        else:
            self.last_recieved_left_axle = msg.velocity[leftArrayIndex]
            self.last_recieved_right_axle = msg.velocity[rightArrayIndex]
            self.last_recieved_stamp = rospy.Time.now()

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return
        
        cmd = TwistWithCovarianceStamped()

        l_vel = self.last_recieved_left_axle * self.wheel_dia / 2.0
        r_vel = self.last_recieved_right_axle * self.wheel_dia / 2.0
        vel = (l_vel + r_vel) / 2.0
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = 'base_footprint'
        cmd.twist.twist.linear.x = vel
        self.pub_twist.publish(cmd)
        

        
        


if __name__ == '__main__':
    rospy.init_node("wheel_odometry_node")
    node = OdometryNode()
    rospy.spin()