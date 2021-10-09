#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_manager.my_state_machine import MyStateMachine
from sensor_msgs.msg import PointField

class GameStatus():
    SETUP = 0
    MAIN_START = 1
    MAIN = 2
    RESTART = 3
    END = 4

class GameManager():
    def __init__(self):
        self._command = GameStatus.SETUP
        self._my_state_machine = MyStateMachine()
        rospy.Subscriber("game_status", PointField, self.gamepadCallback)
        
    def gamepadCallback(self, msg):
        self._command = msg.datatype
        # if self._command == GameStatus.MAIN:
        #     entry = msg.count
        #     if entry == EntryPoint.BISCO:
        #         state = self._my_state_machine.calcBiscoAction

        #         self._my_state_machine.setStartState()

    def main(self):
        self._command = GameStatus.MAIN_START
        while not rospy.is_shutdown():
            if self._command == GameStatus.MAIN_START:
                self._my_state_machine.mainStart()
                self._command = GameStatus.MAIN

            elif self._command == GameStatus.MAIN:
                self._my_state_machine.main()

            elif self._command == GameStatus.RESTART:
                self._my_state_machine.restart()
            
            elif self._command == GameStatus.END:
                self._my_state_machine.end()

if __name__ == "__main__":
    rospy.init_node("GameManager")
    game_manager = GameManager()
    # game_manager.init()
    game_manager.main()