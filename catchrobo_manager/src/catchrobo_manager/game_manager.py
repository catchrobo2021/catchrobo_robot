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


class ActionState(object):
    BISCO = 1
    SHOOT = 3
    FINISH = 4


class GripperNumber(object):
    FORWARD = 1
    BACKWARD = 0


class GameManager(object):
    def __init__(self):
        rospy.init_node("GameManager")

        self._mymoveit = MyMoveitRobot()
        self.BISCO_SIZE = 0.086, 0.029, 0.136
        self.BISCO_NUM = 27
        self._color = rospy.get_param("/color")
        self._can_go_common = False

        self._target_gripper_id = 0
        self._world_frame = "world"

        self._next_action = ActionState.BISCO

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

        self._gripping_bisco_id = [None, None]

    def init(self):
        self.readCsvs()

        self.AddBisco2Scene()
        self._mymoveit.goStartup()
        self._mymoveit.goHome()

    def AddBisco2Scene(self):
        for i in range(self.BISCO_NUM):
            if not self._biscos.isExist(i):
                continue
            p = PoseStamped()
            p.header.frame_id = self._world_frame
            p.pose.position = self._biscos.getPosi(i)
            p.pose.position.z += self.BISCO_SIZE[2] / 2 + 0.0005
            p.pose.orientation.w = 1.0
            size = self.BISCO_SIZE[0], self.BISCO_SIZE[1], self.BISCO_SIZE[2] - 0.001
            self._mymoveit.addBox2Scene(i, p, size)

    # [TODO] load from the retry point
    def readCsvs(self):
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        pkg_path = rospack.get_path("catchrobo_manager")

        config_path = pkg_path + "/config/"

        bisco_csv = config_path + self._color + "_bisco.csv"
        self._biscos = BiscoDatabase("bisco", bisco_csv, "exist")
        shoot_csv = config_path + self._color + "_shoot.csv"
        self._shoots = ObjectDatabase("shoot", shoot_csv, "open")

    def main(self):
        # self._arm.set_max_velocity_scaling_factor(1)

        # rate = rospy.Rate(10)
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self._next_action == ActionState.BISCO:
                ret = self.biscoAction()
                self._next_action = ActionState.SHOOT
                if ret is None:
                    self._next_action = ActionState.FINISH
            elif self._next_action == ActionState.SHOOT:
                self.AboveHand()
                ret = self.oneShootAction(GripperNumber.BACKWARD)
                if ret is True:
                    ret = self.oneShootAction(GripperNumber.FORWARD)
                if ret is None:
                    self._next_action = ActionState.FINISH
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

    # [TODO]
    def mannualAction(self):
        rospy.loginfo(self._arm.get_current_pose())
        pass

    def getGraspDist(self, target_id):
        if self._biscos.getState(target_id, "my_area"):
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

    def biscoAction(self):
        self._biscos.calcTargetTwin()
        target_ids, is_twin = self._biscos.getTargetTwin()

        if target_ids[0] is None:
            return None

        if is_twin:
            ret = self.normalGo2Bisco(GripperNumber.BACKWARD, target_ids[0])
            ###### grip both
            self._mymoveit.graspBisco(
                GripperNumber.BACKWARD,
                self._biscos.getName(target_ids[0]),
                True,
                self.getGraspDist(target_ids[0]),
            )
            self._gripping_bisco_id[GripperNumber.BACKWARD] = target_ids[0]
            self._mymoveit.graspBisco(
                GripperNumber.FORWARD,
                self._biscos.getName(target_ids[1]),
                True,
                self.getGraspDist(target_ids[1]),
            )
            self._gripping_bisco_id[GripperNumber.FORWARD] = target_ids[1]

            ### update bisco before go. without this, game cannot go next bisco if once fails to go.
            self._biscos.updateState(target_ids[0], False)
            self._biscos.updateState(target_ids[1], False)
        else:
            ###### gripper1
            ret = self.normalGo2Bisco(GripperNumber.BACKWARD, target_ids[0])

            self._mymoveit.graspBisco(
                GripperNumber.BACKWARD,
                self._biscos.getName(target_ids[0]),
                True,
                self.getGraspDist(target_ids[0]),
            )
            self._gripping_bisco_id[GripperNumber.BACKWARD] = target_ids[0]
            self._biscos.updateState(target_ids[0], False)
            ###### gripper2
            if target_ids[1] is not None:
                ret = self.normalGo2Bisco(GripperNumber.FORWARD, target_ids[1])
                self._mymoveit.graspBisco(
                    GripperNumber.FORWARD,
                    self._biscos.getName(target_ids[1]),
                    True,
                    self.getGraspDist(target_ids[1]),
                )
                self._gripping_bisco_id[GripperNumber.FORWARD] = target_ids[1]
                self._biscos.updateState(target_ids[1], False)

        return ret

    def normalGo2Bisco(self, target_gripper, target_id):
        target_pose = Pose()
        target_pose.position = self._biscos.getPosi(target_id)
        target_pose.position.z = self.BISCO_GRIP_Z
        if self._biscos.getState(target_id, "my_area"):
            if target_gripper == GripperNumber.FORWARD:
                add_x = 0.025 + self.ARM2GRIPPER
                ## [TODO] for blue, * minus
            else:
                add_x = -0.025 - self.ARM2GRIPPER
            target_pose.position.x += add_x
            target_pose.orientation = self.MY_GRIP_QUAT

        else:
            if target_gripper == GripperNumber.FORWARD:
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

    def oneShootAction(self, target_gripper):
        self._shoots.calcTargetId()
        target_id = self._shoots.getTargetId()
        print("target shoot: ", target_id)
        if target_id is None:
            return None

        ###### above bisco
        target_pose = Pose()
        target_pose.position = self._shoots.getPosi(target_id)
        target_pose.position.z = self.BISCO_GRIP_Z

        gripping_bisco = self._gripping_bisco_id[target_gripper]
        if self._biscos.getState(gripping_bisco, "my_area"):
            if target_gripper == GripperNumber.FORWARD:
                add_x = 0.025 + self.ARM2GRIPPER
                ## [TODO] for blue, * minus
            else:
                add_x = -0.025 - self.ARM2GRIPPER
            target_pose.position.x += add_x
            target_pose.orientation = self.MY_GRIP_QUAT

        else:
            if self._biscos.getState(gripping_bisco, "forward"):
                if target_gripper == GripperNumber.FORWARD:
                    add_y = -self.ARM2GRIPPER
                else:
                    add_y = self.ARM2GRIPPER
                orientation = self.COMMON_RELEASE_FORWARD
            else:
                if target_gripper == GripperNumber.FORWARD:
                    add_y = self.ARM2GRIPPER
                else:
                    add_y = -self.ARM2GRIPPER
                orientation = self.COMMON_RELEASE_BACKWARD

            target_pose.position.y += add_y
            target_pose.orientation = orientation

        self._mymoveit.setTargetPose(target_pose)

        ### update bisco before go. without this, game cannot go next bisco if once fails to go.
        self._shoots.updateState(target_id, False)

        if not self._mymoveit.go():
            rospy.logerr("cannot make path")
            return False

        self._mymoveit.releaseBisco(target_gripper)
        return True


if __name__ == "__main__":
    game_manager = GameManager()
    game_manager.init()
    game_manager.main()
    rospy.spin()
