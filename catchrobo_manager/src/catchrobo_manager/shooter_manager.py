#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catchrobo_manager.servo import Servo



class ShooterManager:
    def __init__(self):
        self._shooters = [Servo("shooter1"), Servo("shooter2"), Servo("shooter3")]
        self._bar = Servo("bar")
        self.OPEN_DEG = 15
        self.BAR_UP = 0
        self.BAR_DOWN = 90

    def open(self, shooting_box):
        row = shooting_box["row"]
        shooter_id = int(row//2)
        temp = row - 2*shooter_id
        if temp == 1:
            direction = 1
        else:
            direction = -1

        deg = direction * self.OPEN_DEG

        shooter = self._shooters[shooter_id]
        shooter.move(deg)


    def close(self, shooting_box):
        row = shooting_box["row"]
        shooter_id = int(row//2)
        shooter = self._shooters[shooter_id]
        shooter.move(0)
        
    def barUp(self):
        self._bar.move(self.BAR_UP)

    def barDown(self):
        self._bar.move(self.BAR_DOWN)