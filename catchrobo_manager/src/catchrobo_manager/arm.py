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
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest
from moveit_msgs.srv import GetPositionIK


class Arm(object):
    def __init__(self):
        self._scene = (
            moveit_commander.PlanningSceneInterface()
        )  # sleep a bit to update roscore
        self._robot = moveit_commander.RobotCommander()
        rospy.loginfo(self._robot.get_group_names())
        self._arm = moveit_commander.MoveGroupCommander("arm0")
        self._gripper = [
            moveit_commander.MoveGroupCommander("hand1"),
            moveit_commander.MoveGroupCommander("hand2"),
        ]
        
        rospy.wait_for_service("compute_ik", timeout=10.0)
        rospy.wait_for_service("/get_planning_scene", timeout=10.0)
        self.compute_ik = rospy.ServiceProxy("compute_ik", GetPositionIK)

        self._pose_stamped = PoseStamped()
        self._pose_stamped.header.frame_id = "world"  # Hard coded for now

        # Create a moveit ik request
        self._ik_request = PositionIKRequest()
        self._ik_request.group_name = "arm0"  # Hard coded for now
        # self._ik_request.ik_link_names = hand1_attached_link_names
        self._ik_request.timeout.secs = 0.1
        self._ik_request.avoid_collisions = True
        self._ik_request.attempts = 1000
        self._handling_box = [0, 0]

        
    def addBox2Scene(self, name, p, size):
        rospy.sleep(0.1)
        self._scene.add_box(name, p, size)
        

    def gripperMove(self, target_gripper, dist, wait):
        temp = "gripper {}: {} cm".format(target_gripper, dist)
        rospy.loginfo(temp)
        # self._gripper[target_gripper].set_joint_value_target([dist, dist])
        # self._gripper[target_gripper].go(wait)

    def goStartup(self):
        for i in range(2):
            self.gripperMove(i, 0, False)
        self._arm.set_named_target("startup")
        self._arm.go()

    def goHome(self):
        # for i in range(2):
        #     self.gripperMove(i, 0, False)
        self._arm.set_named_target("home")
        self._arm.go()

    def setTargetPose(self, target_pose):
        self._target_pose = target_pose

    def getTargetPose(self):
        return self._target_pose

    def go(self):
        goal = self._target_pose
        now = rospy.Time.now()
        self._pose_stamped.header.stamp = now
        self._pose_stamped.pose = goal
        self._ik_request.pose_stamped = self._pose_stamped
        self._ik_request.robot_state = self._robot.get_current_state()

        request_value = self.compute_ik(self._ik_request)
        if request_value.error_code.val < 0:
            return False
        joints = request_value.solution.joint_state.position
        rospy.loginfo(joints)

        self._arm.set_joint_value_target(joints[0:5])
        dt = rospy.Time.now().to_sec() - now.to_sec()
        rospy.loginfo("IK time : {}".format(dt))

        # if mode == "safe":
        now = rospy.Time.now().to_sec()
        plan = self._arm.plan()

        dt = rospy.Time.now().to_sec() - now
        temp = "plan time : {}".format(dt)
        rospy.loginfo(temp)

        now = rospy.Time.now().to_sec()
        ret = self._arm.execute(plan)

        dt = rospy.Time.now().to_sec() - now
        temp = "execute time : {}".format(dt)
        rospy.loginfo(temp)

        # elif mode == "cartesian":
        #     waypoints = [goal]
        #     (plan, fraction) = self._arm.compute_cartesian_path(waypoints, 100, 0)
        #     ret = self._arm.execute(plan, wait=True)

        return ret

    def graspBisco(self, target_gripper, bisco_name, wait, dist):
        # [TODO] change for servo
        touch_links = self._robot.get_link_names("arm0")
        self._handling_box[target_gripper] = bisco_name
        box_name = self._handling_box[target_gripper]
        self._scene.attach_box(
            self._arm.get_end_effector_link(), box_name, touch_links=touch_links
        )
        self.gripperMove(target_gripper, dist, wait)
        return True

    def releaseBisco(self, target_gripper):
        # box_name = self._biscos.getTargeName()
        box_name = self._handling_box[target_gripper]
        self._scene.remove_attached_object(
            self._arm.get_end_effector_link(), name=box_name
        )
        rospy.loginfo("remove start")
        rospy.sleep(0.1)
        self._scene.remove_world_object(box_name)
        rospy.sleep(0.1)
        self.gripperMove(target_gripper, 0, True)
        return True
        

    def setActions(self, actions):
        self._actions = actions
    
    # def doActions(self):
    #     for action in self._actions:
    #         ret = self.doAction(action)
    #         if not ret:
    #             return False
    #     return True
    
    # def doAction(self, action):
    #     command_type = action[0]
    #     if command_type == "move":
    #         ret = self.move(action[1])
    #     elif command_type == "above":
    #         ret = self.above(action[1])
    #     elif command_type == "grip":
    #         ret = self.graspBisco(*action[1:])
    #     elif command_type == "release":
    #         ret = self.releaseBisco(action[1])
    #     return ret

    def move(self, target_pose):
        self.setTargetPose(target_pose)
        return self.go()
    
    def above(self, z):
        target_pose = self.getTargetPose()
        target_pose.position.z = z
        self.setTargetPose(target_pose)
        return self.go()