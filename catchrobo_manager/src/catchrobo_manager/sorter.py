#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
from rospy.core import is_shutdown
from std_msgs.msg import Int8
from catchrobo_manager.servo import Servo

class SorterClient:
    def __init__(self):
        self._open_pub = rospy.Publisher("sorter/open_row", Int8, queue_size=5)
        self._close_pub = rospy.Publisher("sorter/close_row", Int8, queue_size=1)
    
    def open(self, row):
        self._open_pub.publish(row)

    def close(self, row):
        self._close_pub.publish(row)


class SorterManager:
    def __init__(self, color):
        self._shooters = [Servo("sorter1"), Servo("sorter2"), Servo("sorter3")]
        self.OPEN_DEG = 30
        self.CLOSE_DEG = 0
        self._color = color

        self.CLOSE_WAIT_S = 0.5
        self._is_closing = False
        self._open_miss = False
        self.RELEASE_BISCO_WAIT = 0.5

        # self._lock = threading.Lock()

    def init(self):
        open_row = [0,2,4]
        for i in open_row:
            self.setOpenRow(i)
            self.open()

    def setOpenRow(self, row):
        self._open_row = row

    def getRow2Direction(self, row):
        row2sorter = [2,2,1,1,0,0]
        if self._color == "blue":
            row2sorter = list(reversed(row2sorter))

        sorter_id = row2sorter[row]
        if row%2 == 0:
            sign = 1
        else:
            sign = -1
        if self._color == "blue":
            sign *= -1
        return sorter_id, sign

    def openAcion(self):
        row = self._open_row
        sorter_id, sign = self.getRow2Direction(row)
        deg = sign * self.OPEN_DEG
        shooter = self._shooters[sorter_id]

        for i in range(5):
            shooter.move(deg, 0)

    def open(self, row):
        self.setOpenRow(row)
        if self._is_closing:
            self._open_miss = True
            return
        self.openAcion()

    def closeAction(self, row):
        sorter_id, sign = self.getRow2Direction(row)
        shooter = self._shooters[sorter_id]
        for i in range(5):
            shooter.move(self.CLOSE_DEG, self.CLOSE_WAIT_S*0.2)

    def close(self, row):
        self._is_closing = True
        rospy.sleep(self.RELEASE_BISCO_WAIT)
        self.closeAction(row)
        self._is_closing = False
        if self._open_miss:
            self.openAcion()
            self._open_miss = False
        
    # def spin(self):
    #     rate = rospy.Rate(10)
    #     while not rospy.is_shutdown():
    #         with self._lock:
    #             deg = self._deg
    #             shooter.move(deg, 0)
    #         rate.sleep()

class SorterServer:
    def __init__(self, color):
        self._sorter = SorterManager(color)
        rospy.Subscriber("sorter/open_row", Int8, callback=self.open)
        rospy.Subscriber("sorter/close_row", Int8, callback=self.close)
        
    
    def open(self, msg):
        self._sorter.open(msg.data)
    
    def close(self, msg):
        self._sorter.close(msg.data)
        


