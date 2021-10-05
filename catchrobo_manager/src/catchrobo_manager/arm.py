#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest
from moveit_msgs.srv import GetPositionIK


class Arm(object):
    def __init__(self):
          # sleep a bit to update roscore
        self._robot = moveit_commander.RobotCommander()
        # rospy.loginfo(self._robot.get_group_names())
        self._arm = moveit_commander.MoveGroupCommander("arm0")
        
        rospy.wait_for_service("compute_ik", timeout=10.0)
        self.compute_ik = rospy.ServiceProxy("compute_ik", GetPositionIK)

        self._pose_stamped = PoseStamped()
        self._pose_stamped.header.frame_id = "world"  # Hard coded for now

        # Create a moveit ik request
        self._ik_request = PositionIKRequest()
        self._ik_request.group_name = "arm0"  # Hard coded for now
        # self._ik_request.ik_link_names = hand1_attached_link_names
        self._ik_request.timeout.secs = 1.0
        self._ik_request.avoid_collisions = True
        # self._ik_request.attempts = 1000

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
        # self._arm.set_pose_target(self._target_pose)
        
        # return self._arm.go()


        goal = self._target_pose
        # rospy.loginfo(goal.position)
        now = rospy.Time.now()
        self._pose_stamped.header.stamp = now
        self._pose_stamped.pose = goal
        self._ik_request.pose_stamped = self._pose_stamped
        self._ik_request.robot_state = self._robot.get_current_state()

        request_value = self.compute_ik(self._ik_request)
        if request_value.error_code.val < 0:
            rospy.logerr("ik error {}".format(request_value.error_code.val))
            return False
        joints = request_value.solution.joint_state.position
        # rospy.loginfo(joints)

        self._arm.set_joint_value_target(joints[0:5])

        # self._arm.set_joint_value_target(goal, self._arm.get_end_effector_link(), True)
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

    def move(self, target_pose):
        self.setTargetPose(target_pose)
        return self.go()
    
    def above(self, z):
        target_pose = self.getTargetPose()
        target_pose.position.z = z
        self.setTargetPose(target_pose)
        return self.go()