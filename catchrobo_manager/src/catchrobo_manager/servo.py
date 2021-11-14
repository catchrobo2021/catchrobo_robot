#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Bool

class Servo:
    def __init__(self, name):
        self._name = name
        self._pub = rospy.Publisher(name, Float64, queue_size=10)
        
    def move(self, val, wait=True):
        send_msg = Float64(val)
        self._pub.publish(send_msg)
        if wait:
            rospy.sleep(0.5)
            pass


class Laser:
    def __init__(self, name):
        self._name = name
        self._pub = rospy.Publisher(name, Bool, queue_size=1)
   
    def output(self, laser_on):
        send_msg = Bool(laser_on)
        self._pub.publish(send_msg)
