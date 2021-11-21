#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from catchrobo_manager.servo import Servo
from std_msgs.msg import Bool, Int8

class Guide:
    def __init__(self):
        self._bar = Servo("guide")
        self._pub_enable = rospy.Publisher("guide_enable", Bool, queue_size=1)
        self.BAR_UP = -165
        self.BAR_DOWN = 0

        self.BAR_HORIZONTAL = -30
        self.BAR_SAFE = -60

        self.guideOnOff(True)

        self._now_deg = self.BAR_UP
    
    def move(self, deg):
        self._now_deg  = deg

        ####[TODO] 前日。本当はservoクラスに入れる
        for i in range(5):
            self._bar.move(deg)


    def guideOnOff(self, on_off):
        msg = Bool()
        msg.data =on_off
        self._pub_enable.publish(msg)
            
    def barUp(self):
        self.guideOnOff(True)
        rospy.sleep(0.1)
        self.move(self.BAR_UP)
        # self.guideOnOff(False)

    def barDown(self):
        self.guideOnOff(True)
        rospy.sleep(0.1)

        for i in range(self._now_deg, self.BAR_DOWN, 5):
            self.move(i)
            rospy.sleep(0.1)
        self.move(self.BAR_DOWN)
        
    
    def canGoCommon(self):
        self.guideOnOff(True)
        rospy.sleep(1)
        self.move(self.BAR_HORIZONTAL)
        rospy.sleep(1)
        self.barDown()
    
    def barSafe(self):
        self.move(self.BAR_SAFE)

    
class GuideActionEnum:
    UP = 0
    DOWN = 1
    CAN_GO_COMMON = 2
    BAR_SAFE = 3

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
        elif action == GuideActionEnum.BAR_SAFE:
            self._guide.barSafe()


class GuideClient:
    def __init__(self):
        self._pub = rospy.Publisher("guide/action", Int8, queue_size=5)
    
    def barUp(self):
        self._pub.publish(GuideActionEnum.UP)
    
    def barDown(self):
        self._pub.publish(GuideActionEnum.DOWN)
    
    def canGoCommon(self):
        self._pub.publish(GuideActionEnum.CAN_GO_COMMON)
    
    def barSafe(self):
        self._pub.publish(GuideActionEnum.BAR_SAFE)