#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros

class TransformNode:

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


    def __init__(self):
        rospy.Subscriber('/teb/cmd_vel', Twist, self.transform_speed)

    def transform_speed(self, msg):
        msg.linear.x *= rospy.get_param("mul_weight", 3)
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("teb_cmd_vel")
    node = TransformNode()
    rospy.spin()
