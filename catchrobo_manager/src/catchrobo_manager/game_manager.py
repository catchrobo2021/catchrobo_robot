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
