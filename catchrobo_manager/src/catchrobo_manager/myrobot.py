#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Bool

from catchrobo_manager.brain import Brain
from catchrobo_manager.arm import Arm
from catchrobo_manager.my_robot_result import MyRobotResultMaker
from catchrobo_manager.gripper_manager import GripperManager
# from catchrobo_manager.shooter_manager import ShooterManager
from catchrobo_manager.guide import GuideClient
from catchrobo_manager.sorter import SorterClient
from std_msgs.msg import Bool


class MyRobot():
    def __init__(self,color):
        self._brain = Brain()
        self._arm = Arm()
        self._gripper = GripperManager(color)
        self._guide = GuideClient()
        self._sorter = SorterClient()
        
        # self._enable_joints_publisher.publish(True)
        # rospy.sleep(1)
        # self._arm.goHome(color)
        # self._guide.barDown()
        self._color = color
    
    def init(self):
        # self._enable_joints_publisher = rospy.Publisher('arm0_controller/enable_joints', Bool, queue_size=1)
        # rospy.sleep(0.3)
        self._arm.enable(True)
        rospy.sleep(0.3)        
        self._arm.goHome(self._color)
        self._guide.barUp()
        self._gripper.releaseBisco(0)
        self._gripper.releaseBisco(1)
        open_row = [0,2,4]
        for i in open_row:
            self._sorter.open(i)


    def doAction(self):
        action = self._brain.popAction()
        
        action.show_action()
        params = action.getParams()
        result = MyRobotResultMaker.empty()
        if action.isMove():
            target_pose, laser_on = params
            self._gripper.laser(laser_on)
            self._arm.move(target_pose)

        elif action.isAbove():
            z = params[0]
            self._arm.above(z)

        elif action.isGrip():
            target_gripper, bisco, grip_way,wait = params
            self._gripper.graspBisco(target_gripper, grip_way,wait)
            result = MyRobotResultMaker.grip(bisco.name)

        elif action.isShoot():
            target_gripper, bisco, shooting_box = params
            self._gripper.releaseBisco(target_gripper)
            self._sorter.close(shooting_box.name)
            result = MyRobotResultMaker.shoot(bisco.name, shooting_box.name)

        elif action.isFinish():
            result = MyRobotResultMaker.finish()
            
        elif action.isOpenShooter():
            shooting_box = params[0]
            self._sorter.open(shooting_box.name)

        return result
    
    def calcBiscoAction(self, targets, is_twin):
        self._brain.calcBiscoAction(targets, is_twin)

    def calcShootAction(self, targets, is_twin):
        self._brain.calcShootAction(targets, is_twin)

    def end(self):
        self._arm.move(self._end_pose)
        # self._guide.barUp()
        # self._arm.enable(False)

    def mainStart(self):
        self._arm.enable(True)
        self._guide.barDown()
        rospy.sleep(0.3)
    
    def makeEndPose(self, target_bisco):
        self._end_pose = self._brain.makeEndPose(target_bisco)
        

    def emergencyStop(self):
        self._arm.enable(False)
        self._guide.barUp()
