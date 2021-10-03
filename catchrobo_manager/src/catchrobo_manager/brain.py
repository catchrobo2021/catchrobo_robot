#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy
import tf

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped



class GripperNumber(object):
    GRIPPER1 = 0
    GRIPPER2 = 1

#helper function
def getObjectPosi(obj):
    posi = Point()
    posi.x = obj["x"]
    posi.y = obj["y"]
    posi.z = obj["z"]
    return posi

class ActionType():
    MOVE = 0
    ABOVE = 1
    GRIP = 2
    RELEASE = 3
    FINISH = 4

    @classmethod
    def show_action(cls, action):
        if action == ActionType.MOVE:
            temp = "MOVE"
        elif action == ActionType.ABOVE:
            temp = "ABOVE"
        elif action == ActionType.GRIP:
            temp = "GRIP"
        elif action == ActionType.RELEASE:
            temp = "RELEASE"
        elif action == ActionType.FINISH:
            temp = "FINISH"
        rospy.loginfo("ActionType : " + temp)

class Brain():
    def __init__(self):
        

        
        self.BISCO_SIZE = 0.086, 0.029, 0.136
        self.BISCO_NUM = 27
        self._color = rospy.get_param("/color")

        self._world_frame = "world"

        self.ARM2GRIPPER = 0.05
        self.BISCO_ABOVE_Z = 0.136 + 0.06
        self.BISCO_GRIP_Z = 0.136 + 0.04
        self.BISCO_ABOVE_COMMON_Z = 0.136 + 0.1


        self.MYAREA_GRASP_DIST = 0.03
        self.COMMON_GRASP_DIST = 0.01

        common_pick_quat = tf.transformations.quaternion_from_euler(np.pi, 0, 0)
        self.COMMON_GRIP_QUAT = Quaternion(*common_pick_quat)

        self.COMMON_RELEASE_FORWARD = self.COMMON_GRIP_QUAT

        common_pick_quat = tf.transformations.quaternion_from_euler(np.pi, 0, np.pi)
        self.COMMON_RELEASE_BACKWARD = Quaternion(*common_pick_quat)

        my_pick_quat = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi / 2)
        self.MY_GRIP_QUAT = Quaternion(*my_pick_quat)

    def calcBiscoAction(self, targets,is_twin):
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
        actions.append([ActionType.FINISH])
        self._actoins =  actions

    def popAction(self):
        if len(self._actoins) == 0:
            return None
            
        return self._actoins.pop(0)

    

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
        return [ActionType.MOVE, target_pose]


    def graspAction(self, target_gripper, target, wait):
        if target["my_area"]:
            dist = self.MYAREA_GRASP_DIST
        else:
            dist = self.COMMON_GRASP_DIST

        return [ActionType.GRIP, target.name, target_gripper, wait, dist]

    def AboveHand(self, biscos):
        if biscos[0]["my_area"] and biscos[1]["my_area"]:
            z = self.BISCO_ABOVE_Z
        else:
            z = self.BISCO_ABOVE_COMMON_Z
        
        return [ActionType.ABOVE, z]
    
    def calcShootAction(self, targets, biscos):
        actions = []
        action = self.AboveHand(biscos)
        actions.append(action)
        action = self.arriveShoot(GripperNumber.GRIPPER1,targets, biscos)
        actions.append(action)
        action = self.releaseAction(GripperNumber.GRIPPER1, targets, biscos)
        actions.append(action)

        action = self.arriveShoot(GripperNumber.GRIPPER2,targets, biscos)
        actions.append(action)
        action = self.releaseAction(GripperNumber.GRIPPER2, targets, biscos)
        actions.append(action)
        actions.append([ActionType.FINISH])
        self._actoins =  actions

    def releaseAction(self, target_gripper, target_shooting_boxes, biscos):
        return [ActionType.RELEASE, target_gripper, target_shooting_boxes[target_gripper].name, biscos[target_gripper].name]

    def arriveShoot(self,target_gripper, target_shoots, gripping_biscos):
        
        target_shoot = target_shoots[target_gripper]
        gripping_bisco = gripping_biscos[target_gripper]
        if target_shoot is None:
            return None
        ###### above bisco
        target_pose = Pose()
        target_pose.position = getObjectPosi(target_shoot)
        target_pose.position.y -= 0.1
        target_pose.position.z = self.BISCO_GRIP_Z
        if gripping_bisco["my_area"]:
            if target_gripper == GripperNumber.GRIPPER2:
                add_x = 0.025 + self.ARM2GRIPPER
                ## [TODO] for blue, * minus
            else:
                add_x = -0.025 - self.ARM2GRIPPER
            target_pose.position.x += add_x
            target_pose.orientation = self.MY_GRIP_QUAT

        else:
            if gripping_bisco["forward"]:
                if target_gripper == GripperNumber.GRIPPER1:
                    add_y = -self.ARM2GRIPPER
                else:
                    add_y = self.ARM2GRIPPER
                orientation = self.COMMON_RELEASE_FORWARD
            else:
                if target_gripper == GripperNumber.GRIPPER1:
                    add_y = self.ARM2GRIPPER
                else:
                    add_y = -self.ARM2GRIPPER
                orientation = self.COMMON_RELEASE_BACKWARD

            target_pose.position.y += add_y
            target_pose.orientation = orientation

        return [ActionType.MOVE, target_pose]
        # self._mymoveit.setTargetPose(target_pose)

        # if not self._mymoveit.go():
        #     rospy.logerr("cannot make path")
        #     return False

        # self._mymoveit.releaseBisco(target_gripper)
        # return True

        # [TODO]
    def mannualAction(self):
        # rospy.loginfo(self._arm.get_current_pose())
        pass

