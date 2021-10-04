#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_manager.my_state_machine import MyStateMachine

class GameStatus():
    SETUP_TIME = 0
    GAME = 1
    RESTART = 2 
    MANUAL = 3
    FINISH = 4

class GameManager():
    def __init__(self):
        self._command = GameStatus.GAME
        self._my_state_machine = MyStateMachine()

    def main(self):
        while not rospy.is_shutdown():
            if self._command == GameStatus.GAME:
                self._my_state_machine.main()

            elif self._command == GameStatus.RESTART:
                self._my_state_machine.restart()
            
            elif self._command == GameStatus.MANUAL:
                # 途中割り込み
                self._my_state_machine.pause()
            
            elif self._command == GameStatus.FINISH:
                self._my_state_machine.finish()

if __name__ == "__main__":
    rospy.init_node("GameManager")
    game_manager = GameManager()
    # game_manager.init()
    game_manager.main()