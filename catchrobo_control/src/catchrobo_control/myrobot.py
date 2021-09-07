#!/usr/bin/env python
# -*- coding: utf-8 -*-


from catchrobo_control.okamoto import Okamoto

class MyRobot(object):
    def __init__(self):
        motors = [Okamoto() for _ in range(5)]
        self._position = [0 for _ in range(5)]

    def write(self, position):
        self._position = position
        pass

    def read(self):
        return self._position
