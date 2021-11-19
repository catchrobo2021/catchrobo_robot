#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import JointState


class Servo:
    def __init__(self, name):
        self._name = name
        self._pub = rospy.Publisher("arduino_command", JointState, queue_size=10)
        self._state = JointState()
        self._state.name = [name]
        
    def move(self, val):
        self._state.position = [val]
        self._pub.publish(self._state)
        rospy.sleep(0.5)
    