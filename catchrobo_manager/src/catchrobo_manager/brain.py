#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import copy

import rospy
import tf

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from catchrobo_manager.my_robot_action import MyRobotActionMaker
from catchrobo_manager.gripper_manager import GripperID, GripWay



#helper function
def getObjectPosi(obj):
    posi = Point()
    posi.x = obj["x"]
    posi.y = obj["y"]
    posi.z = obj["z"]
    return posi

class Brain():
    def __init__(self):
        
        self.BISCO_SIZE = 0.086, 0.029, 0.136
        self.BISCO_NUM = 27
        self._color = rospy.get_param("/color")

        self._world_frame = "world"


        self.MAX_HIGHT =  0.35 #+ 0.039
        self.ARM2GRIPPER = 0.05

        self.SAFE_Z_NO_GRIP = self.BISCO_SIZE[2] + 0.15
        
        self.BISCO_GRIP_Z = self.BISCO_SIZE[2] + 0.05 - 0.02
        self.BISCO_ABOVE_Z =self.BISCO_GRIP_Z + self.BISCO_SIZE[2]  + 0.03  #self.BISCO_SIZE[2] + 0.06
        self.BISCO_ABOVE_COMMON_Z =self.BISCO_ABOVE_Z#self.BISCO_SIZE[2] + 0.1
        self.SHOOT_ADD_Z =  self.MAX_HIGHT - 0.12 #self.BISCO_SIZE[2] + 0.01 +0.05


        
        common_pick_quat1 = tf.transformations.quaternion_from_euler(np.pi, 0, 0)
        common_pick_quat2 = tf.transformations.quaternion_from_euler(np.pi, 0, np.pi)
        if self._color == "red":
            self.COMMON_RELEASE_FORWARD = Quaternion(*common_pick_quat1)
            self.COMMON_RELEASE_BACKWARD = Quaternion(*common_pick_quat2)

            self.COMMON_RELEASE_FORWARD_EULER = [np.pi, 0, 0]
            self.COMMON_RELEASE_BACKWARD_EULER = [np.pi, 0, np.pi]
        else:
            self.COMMON_RELEASE_FORWARD = Quaternion(*common_pick_quat2)
            self.COMMON_RELEASE_BACKWARD = Quaternion(*common_pick_quat1)

            self.COMMON_RELEASE_FORWARD_EULER = [np.pi, 0, np.pi]
            self.COMMON_RELEASE_BACKWARD_EULER = [np.pi, 0, 0]
        
        
        
        # self.COMMON_GRIP_QUAT

        # common_pick_quat = tf.transformations.quaternion_from_euler(np.pi, 0, np.pi)
        # self.COMMON_RELEASE_BACKWARD = Quaternion(*common_pick_quat)

        my_pick_quat = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi / 2)
        self.MY_GRIP_QUAT = Quaternion(*my_pick_quat)
        self.MY_RELEASE_EULER = [np.pi, 0, -np.pi / 2]

        self._COMMON_PERMISSION_X = 0.25

    def calcBiscoAction(self, targets,is_twin):
        if not targets[0]["my_area"]:
            actions = self.arriveCommonBisco(GripperID.NEAR, targets[0], self.SAFE_Z_NO_GRIP)

            actions += [
                self.DownHand(),
                self.graspAction(GripperID.NEAR, targets[0], False),
                self.graspAction(GripperID.FAR, targets[1], True),
            ]
        
        elif is_twin:           
            actions = [
                self.arriveBisco(GripperID.NEAR, targets[0], self.SAFE_Z_NO_GRIP),
                self.DownHand(),
                self.graspAction(GripperID.NEAR, targets[0], False),
                self.graspAction(GripperID.FAR, targets[1], True),
            ]

        else:     
            actions = [       
                self.arriveBisco(GripperID.NEAR, targets[0], self.SAFE_Z_NO_GRIP),
                self.DownHand(),
                self.graspAction(GripperID.NEAR, targets[0], True),
            ]

            if targets[1] is not None:
                add = [
                    MyRobotActionMaker.above(self.SAFE_Z_NO_GRIP),
                    self.arriveBisco(GripperID.FAR, targets[1], self.SAFE_Z_NO_GRIP),
                    self.DownHand(),
                    self.graspAction(GripperID.FAR, targets[1], True),
                ]
                actions = actions + add
        add = [
            MyRobotActionMaker.above(self.BISCO_ABOVE_Z),
            MyRobotActionMaker.finish(),
        ]
        actions = actions + add
        self._actoins =  actions
    
    def calcShootAction(self, targets, biscos):
        actions = [
            MyRobotActionMaker.openShooter(targets[0]),
            self.arriveShoot(GripperID.NEAR,targets, biscos),
            self.releaseAction(GripperID.NEAR, targets, biscos),]
        if targets[1] is not None and biscos[1] is not None:
            add =[
                    MyRobotActionMaker.openShooter(targets[1]),
                    self.arriveShoot(GripperID.FAR,targets, biscos),
                    self.releaseAction(GripperID.FAR, targets, biscos),
                ]
            actions = actions + add
        actions = actions + [MyRobotActionMaker.finish()]
        self._actoins =  actions

    def popAction(self):
        if len(self._actoins) == 0:
            return None
            
        return self._actoins.pop(0)

    
    def arriveCommonBisco(self, target_gripper, target, height):
        action_arrive_bisco = self.arriveBisco(target_gripper, target, height)
        params = action_arrive_bisco.getParams()
        target_pose = params[0]

        middle_pose = copy.deepcopy(target_pose)

        middle_pose.position.x = self._COMMON_PERMISSION_X
        if self._color == "blue":
            middle_pose.position.x *= -1

        action_middle = MyRobotActionMaker.move(middle_pose, True)

        actions = [action_middle, 
        MyRobotActionMaker.permission(),
        action_arrive_bisco,
        ]
        return actions

    def arriveBisco(self, target_gripper, target, height):
        target_pose = Pose()
        target_pose.position = getObjectPosi(target)
        target_pose.position.z += height
        if target["my_area"]:
            if target_gripper == GripperID.FAR:
                
                add_x = 0.025 + self.ARM2GRIPPER
            else:
                add_x = -0.025 - self.ARM2GRIPPER
            if self._color == "blue":
                    add_x = - add_x
            target_pose.position.x += add_x
            target_pose.orientation = self.MY_GRIP_QUAT

        else:
            if target_gripper == GripperID.FAR:
                add_y = self.ARM2GRIPPER
            else:
                add_y = -self.ARM2GRIPPER
            target_pose.position.y += add_y
            target_pose.orientation = self.COMMON_RELEASE_FORWARD
        
        action = MyRobotActionMaker.move(target_pose, True)
        return action


    def graspAction(self, target_gripper, target, wait):
        if target["my_area"]:
            grip_way = GripWay.SMALL_GRIP
        else:
            grip_way = GripWay.LONG_GRIP

        action = MyRobotActionMaker.grip(target_gripper, target, grip_way, wait)
        return action


    def AboveHand(self, z):
        # if biscos[0]["my_area"] and biscos[1]["my_area"]:
        #     z = self.BISCO_ABOVE_Z
        # else:
        #     z = self.BISCO_ABOVE_COMMON_Z
        action = MyRobotActionMaker.above(self.BISCO_ABOVE_Z)
        return action
        
    def DownHand(self):
        action = MyRobotActionMaker.above(self.BISCO_GRIP_Z)
        return action


    def releaseAction(self, target_gripper, target_shooting_boxes, biscos):
        action = MyRobotActionMaker.shoot( target_gripper, biscos[target_gripper],  target_shooting_boxes[target_gripper])
        return action

    def arriveShoot(self,target_gripper, target_shoots, gripping_biscos):
        
        target_shoot = target_shoots[target_gripper]
        gripping_bisco = gripping_biscos[target_gripper]
        if target_shoot is None:
            return None
        ###### above bisco
        target_pose = Pose()
        target_pose.position = getObjectPosi(target_shoot)

        # target_pose.position.y -= 0.1
        target_pose.position.z += self.SHOOT_ADD_Z
        if gripping_bisco["my_area"]:
            if target_gripper == GripperID.FAR:
                add_x = 0.025 + self.ARM2GRIPPER
            else:
                add_x = -0.025 - self.ARM2GRIPPER
            if self._color == "blue":
                    add_x = - add_x
            target_pose.position.x += add_x
            # target_pose.orientation = self.MY_GRIP_QUAT

            euler = self.MY_RELEASE_EULER

        else:
            if gripping_bisco["forward"]:
                if target_gripper == GripperID.NEAR:
                    add_y = -self.ARM2GRIPPER
                else:
                    add_y = self.ARM2GRIPPER
                # orientation = self.COMMON_RELEASE_FORWARD              
                euler = self.COMMON_RELEASE_FORWARD_EULER
            else:
                if target_gripper == GripperID.NEAR:
                    add_y = self.ARM2GRIPPER
                else:
                    add_y = -self.ARM2GRIPPER
                # orientation = self.COMMON_RELEASE_BACKWARD
                euler = self.COMMON_RELEASE_BACKWARD_EULER

            target_pose.position.y += add_y
            # target_pose.orientation = orientation
        
        ##### orientation change
        euler = list(euler)
        euler[2] += target_shoot["yaw_deg"]/180.0 * np.pi
        quat = tf.transformations.quaternion_from_euler(*euler)
        target_pose.orientation = Quaternion(*quat)
        
        action = MyRobotActionMaker.move(target_pose, False)

        return action

    def makeEndPose(self, target_bisco):
        action = self.arriveBisco(GripperID.NEAR, target_bisco, self.MAX_HIGHT)
        params = action.getParams()
        target_pose = params[0]
        return target_pose