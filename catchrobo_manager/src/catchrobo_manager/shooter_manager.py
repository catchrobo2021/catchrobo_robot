#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
from std_msgs.msg import Bool


from catchrobo_manager.servo import Servo


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
        self._pub_enable.publish(on_off)
            
    def barUp(self):
        self._bar.move(self.BAR_UP)
        self.guideOnOff(False)

    def barDown(self):
        self._bar.move(self.BAR_DOWN)
        


class ShooterManager:
    def __init__(self):
        self._shooters = [Servo("sorter1"), Servo("sorter2"), Servo("sorter3")]
        self._guide = Guide()
        self.OPEN_DEG = 30
        

    def open(self, shooting_box):
        row = shooting_box["row"]
        shooter_id = int(row//2)
        temp = row - 2*shooter_id
        if temp == 1:
            direction = 1
        else:
            direction = -1

        deg = direction * self.OPEN_DEG
        shooter = self._shooters[shooter_id]
        shooter.move(deg)


    def close(self, shooting_box):
        row = shooting_box["row"]
        shooter_id = int(row//2)
        shooter = self._shooters[shooter_id]
        shooter.move(0)

    def barUp(self):
        self._guide.barUp()
    
    def barDown(self):
        self._guide.barDown()
