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

def getName(id):
    return "bisco{}".format(id)


class Actions(object):
    def __init__(self):
        rospy.init_node("GameManager")

        self._mymoveit = MyMoveitRobot()
        self.BISCO_SIZE = 0.086, 0.029, 0.136
        self.BISCO_NUM = 27
        self._color = rospy.get_param("/color")
        self._can_go_common = False

        self._target_gripper_id = 0
        self._world_frame = "world"

        self.ARM2GRIPPER = 0.05
        self.BISCO_ABOVE_Z = 0.136 + 0.06
        self.BISCO_GRIP_Z = 0.136 + 0.04

        self.MYAREA_GRASP_DIST = 0.03
        self.COMMON_GRASP_DIST = 0.01

        common_pick_quat = tf.transformations.quaternion_from_euler(np.pi, 0, 0)
        self.COMMON_GRIP_QUAT = Quaternion(*common_pick_quat)

        self.COMMON_RELEASE_FORWARD = self.COMMON_GRIP_QUAT

        common_pick_quat = tf.transformations.quaternion_from_euler(np.pi, 0, np.pi)
        self.COMMON_RELEASE_BACKWARD = Quaternion(*common_pick_quat)

        my_pick_quat = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi / 2)
        self.MY_GRIP_QUAT = Quaternion(*my_pick_quat)

    def init(self, biscos):
        self.AddBisco2Scene(biscos)
        # self._mymoveit.goStartup()
        self._mymoveit.goHome()

    def AddBisco2Scene(self, biscos):
        for i in range(self.BISCO_NUM):
            if not biscos.isExist(i):
                continue
            p = PoseStamped()
            p.header.frame_id = self._world_frame
            p.pose.position = biscos.getPosi(i)
            p.pose.position.z += self.BISCO_SIZE[2] / 2 + 0.0005
            p.pose.orientation.w = 1.0
            size = self.BISCO_SIZE[0], self.BISCO_SIZE[1], self.BISCO_SIZE[2] - 0.001
            self._mymoveit.addBox2Scene(getName(i), p, size)

    def biscoAction(self, targets,is_twin):
        ##### move for grip1
        ret = self.arriveBisco(GripperNumber.GRIPPER1, targets[0])
        if is_twin:
            ###### grip both without move
            self._mymoveit.graspBisco(
                GripperNumber.GRIPPER1,
                getName(targets[0].name),
                False,
                self.getGraspDist(targets[0]),
            )
            self._mymoveit.graspBisco(
                GripperNumber.GRIPPER2,
                getName(targets[1].name),
                True,
                self.getGraspDist(targets[1]),
            )
        else:            
            ###### gripper1
            self._mymoveit.graspBisco(
                GripperNumber.GRIPPER1,
                getName(targets[0].name),
                True,
                self.getGraspDist(targets[0]),
            )
            if targets[1] is not None:
                ###### move
                ret = self.arriveBisco(GripperNumber.GRIPPER2, targets[1])
                ###### grip2
                self._mymoveit.graspBisco(
                    GripperNumber.GRIPPER2,
                    getName(targets[1].name),
                    True,
                    self.getGraspDist(targets[1]),
                )
        return ret

    

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

        self._mymoveit.setTargetPose(target_pose)
        # rospy.loginfo(target_pose)
        # rospy.sleep(100)

        if not self._mymoveit.go():
            rospy.logerr("cannot make path")
            return False
        return True

    def getGraspDist(self, target):
        if target["my_area"]:
            dist = self.MYAREA_GRASP_DIST
        else:
            dist = self.COMMON_GRASP_DIST
        return dist

    def AboveHand(self):
        target_pose = self._mymoveit.getTargetPose()
        target_pose.position.z = self.BISCO_ABOVE_Z
        self._mymoveit.setTargetPose(target_pose)
        if not self._mymoveit.go():
            rospy.logerr("cannot make path")
            return False
        return True
    
    def shootAction(self, targets, biscos):
        self.AboveHand()
        self.oneShootAction(GripperNumber.GRIPPER1,targets, biscos)
        ret = self.oneShootAction(GripperNumber.GRIPPER2, targets, biscos)
        return ret

    def oneShootAction(self,target_gripper, target_shoots, gripping_biscos):
        
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

        self._mymoveit.setTargetPose(target_pose)

        if not self._mymoveit.go():
            rospy.logerr("cannot make path")
            return False

        self._mymoveit.releaseBisco(target_gripper)
        return True

        # [TODO]
    def mannualAction(self):
        rospy.loginfo(self._arm.get_current_pose())
        pass


if __name__ == "__main__":
    game_manager = GameManager()
    game_manager.init()
    game_manager.main()
    rospy.spin()
