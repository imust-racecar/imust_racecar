#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import JointState
import math
        
WHEEL_DIA = 0.14605
        
class WheelEncoder:
    pub_twistWithCovariance = rospy.Publisher('/vesc_sim/wheel_velocity', TwistWithCovarianceStamped, queue_size=1)

    cmd = TwistWithCovarianceStamped()

    def __init__(self):
        
        rospy.Subscriber('/racecar/joint_states', JointState, self.joint_states_callback)
        
        self.cmd.header.frame_id = 'base_footprint'
        

    def joint_states_callback(self, msg):
        left_index = msg.name.index('left_rear_axle')
        right_index = msg.name.index('right_rear_axle')
        l_vel = msg.velocity[left_index] * WHEEL_DIA / 2.0
        r_vel = msg.velocity[right_index] * WHEEL_DIA / 2.0
        vel = (l_vel + r_vel) / 2.0

        self.cmd.header.stamp = self.last_recieved_stamp = rospy.Time.now()
        self.cmd.twist.twist.linear.x = vel
        self.pub_twistWithCovariance.publish(self.cmd)
        


if __name__ == '__main__':
    rospy.init_node("vesc_sim")
    node = WheelEncoder()
    rospy.spin()