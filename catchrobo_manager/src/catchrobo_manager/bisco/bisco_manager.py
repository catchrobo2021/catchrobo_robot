#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catchrobo_manager.bisco.bisco_database import BiscoDatabase
from catchrobo_manager.bisco.bisco_rviz import BiscoRviz
from catchrobo_manager.bisco.bisco_gui import BiscoGUI
from catchrobo_manager.bisco.target_bisco_calculator import TargetBiscoCalculator

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


