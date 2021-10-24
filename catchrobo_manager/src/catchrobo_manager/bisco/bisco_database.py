#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np

import rospkg

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

class BiscoDatabase:
    def __init__(self):
        self._count_key = "exist"

    def readCsv(self, color):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_manager")
        config_path = pkg_path + "/config/"
        bisco_csv = config_path + color + "_bisco.csv"
        self._objects = pd.read_csv(bisco_csv, index_col=0)
    
    def updateState(self, id, key, value):
        self._objects.loc[id, key] = value
    
    def getState(self, id, key):
        return self._objects.loc[id, key]

    def getPosi(self, id):
        posi = Point()
        posi.x = self._objects.loc[id, "x"]
        posi.y = self._objects.loc[id, "y"]
        posi.z = self._objects.loc[id, "z"]
        return posi

    def isExist(self, id):
        return self._objects.loc[id, "exist"]

    def getObj(self, id):
        if id is None:
            return None
        else:
            return self._objects.loc[id]
    
    def delete(self, id):
        self._objects.loc[id, self._count_key] = False
    
    def getNum(self):
        return len(self._objects)
