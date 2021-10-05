#!/usr/bin/env python
# -*- coding: utf-8 -*-


from catchrobo_manager.catchrobo_center import CatchroboCenter, ActionResult
import rospy

class MyStateMachine():
    def __init__(self):
        
        self._catchrobo = CatchroboCenter()
        self._next_state = self.calcBiscoAction
        self._start_time = rospy.Time.now()
    
    def main(self):
        self._next_state = self._next_state()
        
    def restart(self):
        pass

    def pause(self):
        pass

    def finish(self):
        pass


    def calcBiscoAction(self):
        result = self._catchrobo.calcBiscoAction()
        if result == ActionResult.DOING:
            next_state =  self.doBiscoAction
        elif result == ActionResult.GAME_END:
            next_state = self.manual
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
            next_state = self.manual
        return next_state

    def doShootAction(self):
        result = self._catchrobo.doShootAction()
        if result == ActionResult.DOING:
            next_state =  self.doShootAction
        elif result == ActionResult.FINISH:
            next_state = self.calcBiscoAction
            
        return next_state

    def manual(self):
        now = rospy.Time.now()
        done_time = now- self._start_time
        rospy.loginfo(done_time.to_sec())

        return self.doNothing
    
    def doNothing(self):
        return self.doNothing
