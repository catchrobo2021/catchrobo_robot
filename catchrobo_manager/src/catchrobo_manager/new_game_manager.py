#!/usr/bin/env python
# -*- coding: utf-8 -*-

from os import wait
import pandas as pd
import numpy as np

import rospy
import rospkg
import tf
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest
from moveit_msgs.srv import GetPositionIK


from catchrobo_manager.object_database import BiscoDatabase, ObjectDatabase
from catchrobo_manager.my_moveit_robot import MyMoveitRobot
# from catchrobo_manager.actions import Actions
from catchrobo_manager.action_planner import ActionPlanner, getName

class MainAction(self):
    pass

class ShootingBoxManager():
    pass

class Robot():
    def __init__(self):
        self._mymoveit = MyMoveitRobot()
        self._next_state = self.calcTargetBisco

    def calc(self):
        self._next_state = self._next_state()

    def calcTargetBisco(self):

        return
    


    pass

class Field():
    def __init__(self, color):
        self._biscos = BiscoManager(color)
        # self._shooting_box = ShootingBoxManager(self._color)

class CatchroboCenter():
    def __init__(self):
        self._color = rospy.get_param("/color")
        self._robot = Robot()
        self._biscos = BiscoManager(color)
        # self._shooting_box = ShootingBoxManager(self._color)
    
    def calcBiscoAction(self):
        remain_bisco = self._biscos.calcTargetTwin()
        if remain_bisco is False:
            return False
        self._robot.calcBiscoAction()
    
    def doBiscoAction(self):
        self._robot.doAction()




class GamePlayer(self):
    def __init__(self):
        
        self._catchrobo = CatchroboCenter()
        self._next_state = self.calcBiscoAction
        
    def main(self):
        self._next_state = self._next_state()
        

    def restart(self):
        pass

    def pause(self):
        pass

    def finish(self):
        self._robot.setupPose()
        pass


    def calcBiscoAction(self):
        if self._catchrobo.calcBiscoAction():
            next_state =  self.doBiscoAction
        else:
            next_state = self.manual
        return next_state
    
    def doBiscoAction(self):
        return self.calcShootAction

    
    def calcShootAction(self):
        pass

    def manual(self):
        pass


    def calcBiscoAction(self):
        self._biscos.getTargetTwin()
        self._robot.calcBiscoAction()


    def biscoAction(self, targets,is_twin):
        actions = []

        if is_twin:
            action = self.arriveBisco(GripperNumber.GRIPPER1, targets[0])
            actions.append(action)
            action = self.graspAction(GripperNumber.GRIPPER1, targets[0], False)
            actions.append(action)
            action = self.graspAction(GripperNumber.GRIPPER2, targets[1], True)
            actions.append(action)
        else:            
            action = self.arriveBisco(GripperNumber.GRIPPER1, targets[0])
            actions.append(action)
            action = self.graspAction(GripperNumber.GRIPPER1, targets[0], True)
            actions.append(action)

            if targets[1] is not None:
                action = self.AboveHand(targets)
                actions.append(action)
                action = self.arriveBisco(GripperNumber.GRIPPER2, targets[1])
                actions.append(action)
                action = self.graspAction(GripperNumber.GRIPPER2, targets[1], True)
                actions.append(action)
         
        return actions

    def arriveBisco(self, target_gripper, target):
        target_pose = Pose()
        target_pose.position = getObjectPosi(target)
        target_pose.position.z = self.BISCO_GRIP_Z
        if target["my_area"]:
            if target_gripper == GripperNumber.GRIPPER2:
                add_x = 0.025 + self.ARM2GRIPPER
                ## [TODO] for blue, * minus
            else:
                add_x = -0.025 - self.ARM2GRIPPER
            target_pose.position.x += add_x
            target_pose.orientation = self.MY_GRIP_QUAT

        else:
            if target_gripper == GripperNumber.GRIPPER2:
                add_y = self.ARM2GRIPPER
                ## [TODO] for blue, * minus
            else:
                add_y = -self.ARM2GRIPPER
            target_pose.position.y += add_y
            target_pose.orientation = self.COMMON_GRIP_QUAT
        return ["move", target_pose]

    def arriveBisco(self):



        pass

    def manual(self):
        pass

class GameStatus():
    SETUP_TIME = 0
    GAME = 1
    RESTART = 2 
    MANUAL = 3
    FINISH = 4

class GameCommandTransfer():

    def __init__(self):
        self._command == GameStatus.SETUP_TIME
        self._game_player = GamePlayer()

    def main(self):
        while not rospy.is_shutdown():
            if self._command == GameStatus.GAME:
                self._game_player.main()

            elif self._command == GameStatus.RESTART:
                self._game_player.restart()
            
            elif self._command == GameStatus.MANUAL:
                # 途中割り込み
                self._game_player.pause()
            
            elif self._command == GameStatus.FINISH:
                self._game_player.finish()

if __name__ == "__main__":
    game_manager = GameFacilitator()
    # game_manager.init()
    game_manager.main()







class ActionState(object):
    BISCO = 1
    SHOOT = 3
    FINISH = 4

class GameManager(object):
    def __init__(self):
        rospy.init_node("GameManager")

        self._can_go_common = False
        self._next_action = ActionState.BISCO
        self._mymoveit = MyMoveitRobot()
        
        self._action_planner = ActionPlanner()
        self._target_biscos = [None, None]
        self._color = rospy.get_param("/color")

    def init(self):
        self.readCsvs()
        self.addBox2Scene()
        self._mymoveit.goHome()
        # self._action_planner.init(self._biscos)
        # rospy.sleep(30)
    
    def addBox2Scene(self):
        BISCO_SIZE = self._action_planner.BISCO_SIZE
        for i in range(self._action_planner.BISCO_NUM):
            if not self._biscos.isExist(i):
                continue
            p = PoseStamped()
            p.header.frame_id = "world"
            p.pose.position = self._biscos.getPosi(i)
            p.pose.position.z += BISCO_SIZE[2] / 2 + 0.0005
            p.pose.orientation.w = 1.0
            size = BISCO_SIZE[0], BISCO_SIZE[1], BISCO_SIZE[2] - 0.001
            self._mymoveit.addBox2Scene(getName(i), p, size)

    # [TODO] load from the retry point
    def readCsvs(self):
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        pkg_path = rospack.get_path("catchrobo_manager")

        config_path = pkg_path + "/config/"

        bisco_csv = config_path + self._color + "_bisco.csv"
        self._biscos = BiscoDatabase(bisco_csv, "exist")
        shoot_csv = config_path + self._color + "_shoot.csv"
        self._shoots = ObjectDatabase(shoot_csv, "open")
    
    def calcTarget(self, obj, name_str):
        obj.calcTargetTwin()
        target_ids, is_twin = obj.getTargetTwin()
        rospy.loginfo("target {}: {}".format(name_str,target_ids))
        ##### grip bisco
        targets = [obj.getObj(id) for id in target_ids]
        return target_ids, is_twin, targets 

    def main(self):
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self._next_action == ActionState.BISCO:
                ##### calc next bisco
                target_ids, is_twin, targets = self.calcTarget(self._biscos, "box")
                ##### if no bisco, finish
                if target_ids[0] is None and target_ids[1] is None:
                    self._next_action = ActionState.FINISH
                    continue
                ##### grip bisco
                actions = self._action_planner.biscoAction(targets, is_twin)
                self._mymoveit.setActions(actions)
                self._mymoveit.doActions()
                self._biscos.delete(target_ids)
                self._target_biscos = targets
                self._next_action = ActionState.SHOOT
                    
            elif self._next_action == ActionState.SHOOT:
                target_ids, is_twin, targets = self.calcTarget(self._shoots, "shoot")
                ##### if no bisco, finish
                if target_ids[0] is None and target_ids[1] is None:
                    self._next_action = ActionState.FINISH
                    continue

                actions = self._action_planner.shootAction(targets, self._target_biscos)
                self._mymoveit.setActions(actions)
                self._mymoveit.doActions()
                self._shoots.delete(target_ids)
                self._next_action = ActionState.BISCO
                
            elif self._next_action == ActionState.FINISH:
                break

        end_time = rospy.Time.now().to_sec()
        print("time: ", end_time - start_time)
        # rate.sleep()

    # [TODO]
    def checkGoCommon(self):
        if not self._can_go_common:
            self._can_go_common = (
                self._box[self._box["open"]]["priority"].max() <= -3
            )  # 0,1,2が埋まればcommonに入ってよい
        return self._can_go_common



if __name__ == "__main__":
    game_manager = GameManager()
    game_manager.init()
    game_manager.main()
