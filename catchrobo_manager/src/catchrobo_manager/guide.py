#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from catchrobo_manager.servo import Servo
from std_msgs.msg import Bool, Int8

class Guide:
    def __init__(self):
        self._bar = Servo("guide")
        self._pub_enable = rospy.Publisher("guide_enable", Bool, queue_size=1)
        self.BAR_UP = -115
        self.BAR_DOWN = 0

        self.BAR_HORIZONTAL = -30

        self.guideOnOff(True)

    def guideOnOff(self, on_off):
        msg = Bool()
        msg.data =on_off
        self._pub_enable.publish(msg)
            
    def barUp(self):
        self.guideOnOff(True)
        rospy.sleep(0.1)
        self._bar.move(self.BAR_UP)
        # self.guideOnOff(False)

    def barDown(self):
        self.guideOnOff(True)
        rospy.sleep(0.1)
        self._bar.move(self.BAR_DOWN)
        rospy.sleep(0.5)
        self.guideOnOff(False)
        
    
    def canGoCommon(self):
        self.guideOnOff(True)
        rospy.sleep(0.1)
        self._bar.move(self.BAR_HORIZONTAL)
        rospy.sleep(1)
        self.barDown()
    
class GuideActionEnum:
    UP = 0
    DOWN = 1
    CAN_GO_COMMON = 2

class GuideServer:
    def __init__(self):
        self._guide = Guide()
        rospy.Subscriber("guide/action", Int8, callback=self.action)
    
    def action(self, msg):
        action = msg.data
        if action == GuideActionEnum.UP:
            self._guide.barUp()
        elif action == GuideActionEnum.DOWN:
            self._guide.barDown()
        elif action == GuideActionEnum.CAN_GO_COMMON:
            self._guide.canGoCommon()

class GuideClient:
    def __init__(self):
        self._pub = rospy.Publisher("guide/action", Int8, queue_size=5)
    
    def barUp(self):
        self._pub.publish(GuideActionEnum.UP)
    
    def barDown(self):
        self._pub.publish(GuideActionEnum.DOWN)
    
    def canGoCommon(self):
        self._pub.publish(GuideActionEnum.CAN_GO_COMMON)