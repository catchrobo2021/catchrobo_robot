#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import copy

import rospy

from catchrobo_manager.bisco.bisco_database import BiscoDatabase
from catchrobo_manager.bisco.bisco_rviz import BiscoRviz
from catchrobo_manager.bisco.bisco_gui import BiscoGUI

class TargetBiscoCalculator:
    def getMininumPriorityId(self, database):
        exist = database._objects["exist"]
        # print(self._objects[exist]["priority"])
        if np.sum(exist) == 0:
            minimum = None
        else:
            minimum = database._objects[exist]["priority"].idxmin()
        return minimum
            

    def calcTargetTwin(self, database):
        biscos = copy.deepcopy(database)
        first = self.getMininumPriorityId(biscos)
        if first is None:
            target_ids = None, None
            twin = False
            return target_ids, twin

        biscos.delete(first)
        second = self.getMininumPriorityId(biscos)
        twin = self.isNeighbor(biscos, first, second)
        return [first, second], twin
    
    def isNeighbor(self, biscos, first, second):
        ret = False
        if first is None or second is None:
            return False
        areas = [biscos.getState(first, "my_area"), biscos.getState(second, "my_area")]
        if areas[0] == areas[1] == True:
            if abs(first - second) == 6:
                ret = True
        elif areas[0] == areas[1] == False:
            if abs(first - second) == 1:
                ret = True
        return ret
        

class BiscoManager():
    def __init__(self, color):
        self._database = BiscoDatabase()
        self._database.readCsv(color)

        self._calculator = TargetBiscoCalculator()
        self._rviz = BiscoRviz()
        self._gui = BiscoGUI(self._database)

        self._rviz.addBox2Scene(self._database)
        self._gui.sendGUI()

    def pick(self, id):
        self._rviz.attach(id)
        self._database.delete(id)
        self._gui.sendGUI()

    def release(self, id):
        self._rviz.release(id)

    def calcTargetTwin(self):
        self._target_ids, self._twin =  self._calculator.calcTargetTwin(self._database)

    def getTargetTwin(self):
        return [self._database.getObj(id) for id in self._target_ids], self._twin 


