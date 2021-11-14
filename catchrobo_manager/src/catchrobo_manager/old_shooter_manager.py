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
        self._pub_enable.publish(msg)
            
    def barUp(self):
        self._bar.move(self.BAR_UP)
        self.guideOnOff(False)

    def barDown(self):
        self.guideOnOff(True)
        rospy.sleep(0.1)
        self._bar.move(self.BAR_DOWN)
        


class ShooterManager:
    def __init__(self, color):
        self._shooters = [Servo("sorter1"), Servo("sorter2"), Servo("sorter3")]
        self._guide = Guide()
        self.OPEN_DEG = 30
        self._color = color

    def getRow2Direction(self, row):
        row2sorter = [2,2,1,1,0,0]
        if self._color == "blue":
            row2sorter = list(reversed(row2sorter))

        sorter_id = row2sorter[row]


        if row%2 == 0:
            sign = -1
        else:
            sign = 1
        if self._color == "blue":
            sign *= -1
        return sorter_id, sign


    def open(self, shooting_box):
        row = shooting_box.name
        sorter_id, sign = self.getRow2Direction(row)
        deg = sign * self.OPEN_DEG
        shooter = self._shooters[sorter_id]
        shooter.move(deg, False)


    def close(self, shooting_box):
        row = shooting_box.name
        sorter_id, sign = self.getRow2Direction(row)
        shooter = self._shooters[sorter_id]
        # deg = - sign * self.OPEN_DEG
        deg = 0
        shooter.move(deg, True)
        # rospy.sleep(0.5)

    def barUp(self):
        self._guide.barUp()
    
    def barDown(self):
        self._guide.barDown()


