#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import copy


class TargetBiscoCalculator:

    #[TODO]
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
    
    def getMininumPriorityId(self, database):
        exist = database._objects["exist"]
        # print(self._objects[exist]["priority"])
        if np.sum(exist) == 0:
            minimum = None
        else:
            minimum = database._objects[exist]["priority"].idxmin()
        return minimum

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
        
