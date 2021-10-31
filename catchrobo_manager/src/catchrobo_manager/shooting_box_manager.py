#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np

import rospy
import rospkg


from std_msgs.msg import Int32MultiArray

class ShootingBoxManager():
    def __init__(self, color):
        self._count_key = "open"
        self._twin = False
        self.readCsv(color)
        self._pub2gui = rospy.Publisher("box_update", Int32MultiArray, queue_size=1)
        # rospy.Subscriber("box_gui", Int32MultiArray, self.guiCallback)
        self._target_order = [0,2,4,
                            1,0,1,0,1,0,1,
                            3,2,3,2,3,2,3,
                            5,4,5,4,5,4,5]

    def sendGUI(self):
        info = Int32MultiArray()
        info.data = list(self._objects["space"])
        self._pub2gui.publish(info)

    def guiCallback(self,msg):
        # rospy.loginfo(msg.data)
        for i, val in enumerate(msg.data):
            # rospy.loginfo("i, val {}{}".format(i,val))
            self._objects.loc[i, "space"] = int(val)

    
    def readCsv(self, color):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_manager")
        config_path = pkg_path + "/config/"
        csv = config_path + color + "_shoot2.csv"
        self._objects = pd.read_csv(csv, index_col=0)

    def canGoCommon(self):
        group = self._objects.groupby("sorter_id").sum()
        in_sorter_bool = group["space"] >=8
        return in_sorter_bool.sum() <=0

    def delete(self, id):
        self._objects.loc[id, "space"] -= 1

        self._target_order.pop(0)

    def calcTargetTwin(self):
        self._target_ids = self._target_order[0:2]
    
    def getTargetTwin(self):
        return [self.getObj(id) for id in self._target_ids], None

    def getObj(self, id):
        if id is None:
            return None
        else:
            return self._objects.loc[id]

if __name__=="__main__":
    rospy.init_node("test_box")
    manager = ShootingBoxManager("blue")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        ret = manager.canGoCommon()
        rospy.loginfo(ret)
        manager.calcTargetTwin()
        ret, _ = manager.getTargetTwin()
        manager.delete(ret[0].name)
        rospy.loginfo(ret[0].name)
        rate.sleep()
