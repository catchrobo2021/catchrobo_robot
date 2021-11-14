#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from catchrobo_manager.servo import Servo
from std_msgs.msg import Bool

class Guide:
    def __init__(self):
        self._bar = Servo("guide")
        self._pub_enable = rospy.Publisher("guide_enable", Bool, queue_size=1)
        self.BAR_UP = -115
        self.BAR_DOWN = 0

        self.guideOnOff(True)

    def guideOnOff(self, on_off):
        msg = Bool()
        msg.data =on_off
        self._pub_enable.publish(msg)
            
    def barUp(self):
        self._bar.move(self.BAR_UP)
        self.guideOnOff(False)

    def barDown(self):
        self.guideOnOff(True)
        rospy.sleep(0.1)
        self._bar.move(self.BAR_DOWN)
        


