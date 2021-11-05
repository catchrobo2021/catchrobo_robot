#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Int32MultiArray

class BiscoGUI():
    def __init__(self, database):
        self._database = database
        self._pub2gui = rospy.Publisher("ob", Int32MultiArray, queue_size=1)
        rospy.Subscriber("obj", Int32MultiArray, self.guiCallback)

    def sendGUI(self):
        info = Int32MultiArray()
        bisco_num = self._database.getNum()
        info.data = [0] * bisco_num 
        for i in range(bisco_num):
            info.data[i] = self._database.isExist(i)
        self._pub2gui.publish(info)

    def guiCallback(self,msg):
        # rospy.loginfo(msg.data)
        for i, val in enumerate(msg.data):
            temp = bool(val)
            before = self._database.getState(i, "exist")

            if temp != before:
                rospy.loginfo("change bisco {} -> {}".format(i, temp))
            self._database.updateState(i,"exist", temp)

            
