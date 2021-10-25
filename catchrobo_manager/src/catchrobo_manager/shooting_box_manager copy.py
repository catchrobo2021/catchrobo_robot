#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np

import rospkg

from geometry_msgs.msg import Pose, Point, Quaternion


class ShootingBoxManager():
    def __init__(self, color):
        self._count_key = "open"
        self._twin = False
        self.readCsv(color)
    
    def readCsv(self, color):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_manager")
        config_path = pkg_path + "/config/"
        csv = config_path + color + "_shoot.csv"
        self._objects = pd.read_csv(csv, index_col=0)

    def canGoCommon(self):
        group = self._objects.groupby("sorter_id")
        in_sorter_bool = group["space"] >=8
        
        return in_sorter_bool.sum() <= 0

    def delete(self, id):
        self._objects.loc[id, "space"] -= 1

    def calcTargetTwin(self):
        self.calcTargetId()
        first = self.getTargetId()
        if first is None:
            self._target_ids = None, None
            self._twin = False
            return

        self.updateState(first, False)

        self.calcTargetId()
        second = self.getTargetId()
        self.updateState(first, True)
        self._target_ids = first, second
    
    def getTargetTwin(self):
        return [self.getObj(id) for id in self._target_ids], None




###################



    

    def calcTargetId(self):
        exist = self._objects[self._count_key]
        # print(self._objects[exist]["priority"])
        if np.sum(exist) == 0:
            self._target_id = None
        else:
            self.updatePriority()
            self._target_id = self._objects[exist]["priority"].idxmin()

    # [TODO]
    def updatePriority(self):
        pass

    def getTargetId(self):
        return self._target_id

    def getTagetObj(self):
        return self._objects.iloc[self._target_id]

    def updateState(self, id, value):
        self._objects.loc[id, self._count_key] = value

    def getPosi(self, id):
        posi = Point()
        posi.x = self._objects.loc[id, "x"]
        posi.y = self._objects.loc[id, "y"]
        posi.z = self._objects.loc[id, "z"]
        return posi

    def getName(self, id):
        return "{}{}".format(self._name, id)

    def getTargeName(self):
        return self.getName(self.getTargetId())

    def isExist(self, id):
        return self._objects.loc[id, "exist"]

    def getState(self, id, key):
        return self._objects.loc[id, key]
    
    def getObj(self, id):
        if id is None:
            return None
        else:
            return self._objects.loc[id]
    
    

