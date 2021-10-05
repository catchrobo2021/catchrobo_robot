#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import JointState


class Shooter:
    def __init__(self, name):
        self._name = name
        self._pub = rospy.Publisher("arduino_command", JointState, queue_size=10)
        self._state = JointState()
        self._state.name = [name]
        
    def move(self, deg):
        self._state.position = [deg]
        self._pub.publish(self._state)
    


class ShooterManager:
    def __init__(self):
        self._shooters = [Shooter("shooter1"), Shooter("shooter2"), Shooter("shooter3")]
        self.OPEN_DEG = 15

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
        