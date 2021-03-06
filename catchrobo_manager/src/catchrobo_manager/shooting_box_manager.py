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
        self._pub2gui = rospy.Publisher(
            "goal_sub", Int32MultiArray, queue_size=1)
        rospy.Subscriber("goal_pub", Int32MultiArray, self.guiCallback)
        '''
        self._target_order = [0, 2, 4,
                              1, 0, 1, 0, 1, 0, 1,
                              3, 2, 3, 2, 3, 2, 3,
                              5, 4, 5, 4, 5, 4, 5]
        '''

        self._pub_highlight = rospy.Publisher("/highlight_goal", Int32MultiArray, queue_size=1)

    def sendGUI(self):
        info = Int32MultiArray()
        info.data = list(self._objects["exist"])
        self._pub2gui.publish(info)

    def guiCallback(self, msg):
        # rospy.loginfo(msg.data)
        for i, val in enumerate(msg.data):
            # rospy.loginfo("i, val {}{}".format(i,val))
            self._objects.loc[i, "exist"] = int(val)

    def readCsv(self, color):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_manager")
        config_path = pkg_path + "/config/"
        csv = config_path + color + "_shoot2.csv"
        self._objects = pd.read_csv(csv, index_col=0)

    def canGoCommon(self):
        group = self._objects.groupby("sorter_id").sum()
        in_sorter_bool = group["exist"] >= 1
        return in_sorter_bool.sum() >= 3

    def shoot(self, id):
        self._objects.loc[id, "exist"] += 1
        # self._target_order.pop(0)
        self.sendGUI()

    def calcTargetTwin(self):
        box1_l = self._objects.loc[0, "exist"]
        box1_r = self._objects.loc[1, "exist"]
        box2_l = self._objects.loc[2, "exist"]
        box2_r = self._objects.loc[3, "exist"]
        box3_l = self._objects.loc[4, "exist"]
        box3_r = self._objects.loc[5, "exist"]
        box1 = box1_l+box1_r
        box2 = box2_l+box2_r
        box3 = box3_l+box3_r

        val = 100000*box1_l+10000*box1_r+1000*box2_l + 100*box2_r+10*box3_l+box3_r
        if val == 0:
            # 000000
            self._target_ids = [0, 2]
        elif val == 100000:
            # 100000
            self._target_ids = [2, 4]
        elif val == 1000:
            # 001000
            self._target_ids = [0, 4]
        elif val == 10:
            # 000010
            self._target_ids = [0, 2]
        elif val == 101000:
            # 101000
            self._target_ids = [4, 1]
        elif val == 100010:
            # 100010
            self._target_ids = [2, 1]
        elif val == 1010:
            # 001010
            self._target_ids = [0, 1]
        else:
            if box1 <= 6:
                if box1_l > box1_r:
                    self._target_ids = [1, 0]
                    if box1_l-box1_r == 2:
                        self._target_ids = [1, 1]
                elif box1_l <= box1_r:
                    self._target_ids = [0, 1]
                    if box1_r-box1_l == 2:
                        self._target_ids = [0, 0]
            elif box1 == 7:
                if box1_l > box1_r:
                    self._target_ids = [1, 3]
                elif box1_l <= box1_r:
                    self._target_ids = [0, 3]
            elif box2 <= 6:
                if box2_l > box2_r:
                    self._target_ids = [3, 2]
                    if box2_l-box2_r == 2:
                        self._target_ids = [3, 3]
                elif box2_l <= box2_r:
                    self._target_ids = [2, 3]
                    if box2_r-box2_l == 2:
                        self._target_ids = [2, 2]
            elif box2 == 7:
                if box2_l > box2_r:
                    self._target_ids = [3, 5]
                elif box2_l <= box2_r:
                    self._target_ids = [2, 5]
            elif box3 <= 6:
                if box3_l > box3_r:
                    self._target_ids = [5, 4]
                    if box3_l-box3_r == 3:
                        self._target_ids = [5, 5]
                elif box3_l <= box3_r:
                    self._target_ids = [4, 5]
                    if box3_r-box3_l == 3:
                        self._target_ids = [4, 4]
            elif box3 == 7:
                if box3_l > box3_r:
                    self._target_ids = [5, 5]
                elif box3_l <= box3_r:
                    self._target_ids = [4, 4]
        


    def getTargetTwin(self):
        self.highlight(self._target_ids)
        return [self.getObj(id) for id in self._target_ids], None

    def getObj(self, id):
        if id is None:
            return None
        else:
            return self._objects.loc[id]

    def highlight(self, target_ids):
        if target_ids[0] is not None:
            highlight_id = Int32MultiArray()
            if target_ids[1] is not None:
                highlight_id.data  = target_ids
            else:
                highlight_id.data  = target_ids[0:1]
            self._pub_highlight.publish(highlight_id)


if __name__ == "__main__":
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
