#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion


class ObjectDatabase(object):
    def __init__(self, name,csv, count_key):
        self._name = name
        self._objects = pd.read_csv(csv, index_col=0)
        self._count_key = count_key
    
    def calcTargetId(self):
        exist = self._objects[self._count_key]
        # print(self._objects[exist]["priority"])
        if np.sum(exist) == 0:
            self._target_id = None
        else:
            self._target_id = self._objects[exist]["priority"].idxmin()

    def getTargetId(self):
        return self._target_id
    
    def getTagetObj(self):
        return self._objects.iloc[self._target_id]
    
    def updateState(self, id, value):
        self._objects.loc[id, self._count_key] = value

    def getPosi(self, id):
        posi = Point()
        posi.x = self._objects.loc[id,"x"]
        posi.y = self._objects.loc[id,"y"]
        posi.z = self._objects.loc[id,"z"]
        return posi
    
    def getName(self, id):
        return "{}{}".format(self._name, id)

    def getTargeName(self):
        return self.getName(self.getTargetId())

    def isExist(self, id):
        return self._objects.loc[id, "exist"]



        



