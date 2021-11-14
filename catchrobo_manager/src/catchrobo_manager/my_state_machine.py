#!/usr/bin/env python
# -*- coding: utf-8 -*-


from re import T
from catchrobo_manager.catchrobo_center import CatchroboCenter, ActionResult
import rospy

from std_msgs.msg import Bool

class MyStateMachine():
    def __init__(self):
        self._catchrobo = CatchroboCenter()
        self._next_state = self.calcBiscoAction
        self._start_time = rospy.Time.now()

        rospy.Subscriber("on_manual",Bool,self.joyCallback)
        self._is_manual = False
        self._state_after_manual = self._next_state 

    def joyCallback(self, msg):
        self._is_manual = msg.data

    def main(self):
        ret = self._next_state()
        if self._is_manual:
            if ret != self.manual:
                self._state_after_manual = ret
            self._next_state = self.manual
        else:
            self._next_state = ret


    def restart(self):
        pass

    def pause(self):
        pass

    def end(self):
        self._catchrobo.end()

    def mainStart(self):
        pass

    def calcBiscoAction(self):
        result = self._catchrobo.calcBiscoAction()
        if result == ActionResult.DOING:
            next_state =  self.doBiscoAction
        elif result == ActionResult.GAME_END:
            next_state = self.calGameEndTime
        return next_state
    
    def doBiscoAction(self):
        result = self._catchrobo.doBiscoAction()
        if result == ActionResult.DOING:
            next_state = self.doBiscoAction
        elif result == ActionResult.FINISH:
            next_state = self.calcShootAction
        else:
            rospy.logerr(result)
        return next_state
    
    def calcShootAction(self):
        result = self._catchrobo.calcShootAction()
        if result == ActionResult.DOING:
            next_state =  self.doShootAction
        elif result == ActionResult.GAME_END:
            next_state = self.calGameEndTime
        return next_state

    def doShootAction(self):
        result = self._catchrobo.doShootAction()
        if result == ActionResult.DOING:
            next_state =  self.doShootAction
        elif result == ActionResult.FINISH:
            next_state = self.calcBiscoAction
            
        return next_state

    def calGameEndTime(self):
        self._catchrobo.end()
        now = rospy.Time.now()
        done_time = now- self._start_time
        rospy.loginfo(done_time.to_sec())

        self._is_manual = True
        return self.manual
    

    def manual(self):

        if self._is_manual:
            ret =  self.manual
        else:
            ret = self._state_after_manual
        return ret

