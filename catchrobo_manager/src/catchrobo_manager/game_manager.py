#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_manager.catchrobo_center import CatchroboCenter, ActionResult
from sensor_msgs.msg import PointField
from std_msgs.msg import Int8

class GameStatus():
    SETUP = 0
    MAIN_START = 1
    MAIN = 2
    RESTART = 3
    END = 4
    MANUAL = 5

class MenuEnum:
    START = 1
    PAUSE = 2
    STOP = 3

class GameManager():
    def __init__(self):
        self._command = GameStatus.SETUP
        self._catchrobo = CatchroboCenter()
        rospy.Subscriber("menu", Int8, self.menuCallback)
    
    def menuCallback(self, msg):
        menu = msg.data
        if menu == MenuEnum.START:
            if self._command == GameStatus.SETUP:
                self._command = GameStatus.MAIN_START
            elif self._command == GameStatus.MANUAL:
                self._command = GameStatus.MAIN
        
        
    def main(self):
        self._command = GameStatus.MAIN_START
        rospy.sleep(1)
        self._catchrobo.init()
        while not rospy.is_shutdown():
            if self._command == GameStatus.MAIN_START:
                self._catchrobo.mainStart()
                self._command = GameStatus.MAIN

            elif self._command == GameStatus.MAIN:
                ret = self._catchrobo.main()
                if ret == ActionResult.GAME_END:
                    self._command = GameStatus.MANUAL

            # elif self._command == GameStatus.RESTART:
            #     self._my_state_machine.restart()
            
            # elif self._command == GameStatus.END:
            #     self._my_state_machine.end()
            
            elif self._command == GameStatus.MANUAL:
                rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node("GameManager")
    game_manager = GameManager()
    # game_manager.init()
    game_manager.main()