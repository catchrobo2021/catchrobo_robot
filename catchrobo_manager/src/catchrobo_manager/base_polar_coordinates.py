#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import copy
import math

import rospy
import moveit_commander
import tf

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Bool

class BasePolarCoordinates:
    def __init__(self):
        self._listener = tf.TransformListener()
        self._listener.waitForTransform("/world", "/base/robot_tip", rospy.Time(), rospy.Duration(4.0))
        try:
            (trans,rot) = self._listener.lookupTransform('/world', '/base/robot_tip', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self._base_posi = trans
    
    def getBasePosi(self):
        return self._base_posi

    def get_r_theta(self, pose):
        x = pose.position.x - self._base_posi[0]
        y = pose.position.y - self._base_posi[1]

        r = math.sqrt(x**2+y**2)
        rad = math.atan2(y, x)

        return r, rad

