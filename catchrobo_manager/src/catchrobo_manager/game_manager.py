#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_manager.catchrobo_center import CatchroboCenter, ActionResult
from sensor_msgs.msg import PointField
from std_msgs.msg import Int8
import threading

class GameStatus():
    SETUP = 0
    MAIN_START = 1
    MAIN = 2
    EMERGENCY_STOP = 3
    END = 4
    MANUAL = 5

class MenuEnum:
    START = 1
    PAUSE = 2
    EMERGENCY_STOP = 3

class GameManager():
    def __init__(self):
        self._command = GameStatus.SETUP
        self._catchrobo = CatchroboCenter()
        self._lock = threading.Lock()
        rospy.Subscriber("menu", Int8, self.menuCallback)
    
    def menuCallback(self, msg):
        menu = msg.data
        with self._lock:
            if menu == MenuEnum.START:
                if self._command == GameStatus.SETUP or self._command == GameStatus.EMERGENCY_STOP:
                    self._command = GameStatus.MAIN_START
                elif self._command == GameStatus.MANUAL:
                    self._command = GameStatus.MAIN
                
            elif menu == MenuEnum.PAUSE:
                if self._command != GameStatus.SETUP:
                    self._command = GameStatus.MANUAL
            elif menu == MenuEnum.EMERGENCY_STOP:
                self._command = GameStatus.EMERGENCY_STOP
        
        
    def main(self):
        # self._command = GameStatus.MAIN_START
        # rospy.sleep(1)
        rate = rospy.Rate(100)
        self._catchrobo.init()
        while not rospy.is_shutdown():
            with self._lock:
                if self._command == GameStatus.MAIN_START:
                    self._catchrobo.mainStart()
                    self._start_time = rospy.Time.now()

                    self._command = GameStatus.MAIN

                elif self._command == GameStatus.MAIN:
                    ret = self._catchrobo.main()
                    if ret == ActionResult.GAME_END:
                        self._command = GameStatus.END
                    elif ret == ActionResult.PERMISSION:
                        self._command = GameStatus.MANUAL
                    

                elif self._command == GameStatus.EMERGENCY_STOP:
                    self._catchrobo.emergencyStop()
                
                elif self._command == GameStatus.END:
                    self._catchrobo.end()
                    self._command = GameStatus.MANUAL
                    
                    dt = rospy.Time.now() - self._start_time
                    rospy.loginfo(dt.to_sec())
                
                elif self._command == GameStatus.MANUAL:
                    pass
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("GameManager")
    game_manager = GameManager()
    # game_manager.init()
    game_manager.main()