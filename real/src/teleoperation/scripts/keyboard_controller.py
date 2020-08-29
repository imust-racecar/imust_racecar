#!/usr/bin/env python
from geometry_msgs.msg import Twist

import rospy
import sys, select, termios, tty

banner = """
Reading from the keyboard
---------------------------
Moving around:
        w
   a    s    d
anything else : stop
CTRL-C to quit
"""

keyBindings1 = {
  'w':(1,0),
  'd':(1,3.14),
  'a':(1,-3.14),
  's':(-1,0),
}

keyBindings2 = {
    'i': (0.2, 0),
    'k': (-0.2, 0),
    'j': (0, 0.1),
    'l': (0, -0.1)
}

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

speed = 0.4
turn = 0.6

def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  pub = rospy.Publisher('/car/cmd_vel', Twist, queue_size=5)
  rospy.init_node('keyboard_controller')

  x = 0
  th = 0

  while True:
        print(banner)
        key = getKey()
        if key in keyBindings1.keys():
            x = keyBindings1[key][0]
            th = keyBindings1[key][1]
        elif key in keyBindings2.keys():
            speed += keyBindings2[key][0]
            turn += keyBindings2[key][1]
        else:
            x = 0
            th = 0
            if (key == '\x03'):
                break
        msg = Twist();

        msg.linear.x = x * speed
        msg.angular.z = th * turn

        pub.publish(msg)
        print('speed: {}, steering_angle: {}'.format(msg.linear.x, msg.angular.z))
