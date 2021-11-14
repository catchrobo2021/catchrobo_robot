#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import copy
import math

import rospy
import moveit_commander
import tf

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Bool

class ArmExtensionState:
    NEAR2NEAR = 0
    NEAR2FAR = 1
    FAR2FAR = 2
    FAR2NEAR = 3


class Arm(object):
    def __init__(self):
          # sleep a bit to update roscore
        self._robot = moveit_commander.RobotCommander()
        self._arm = moveit_commander.MoveGroupCommander("arm0")
        

        accel = rospy.get_param("max_acceleration_scaling_factor")
        vel = rospy.get_param("max_velosity_scaling_factor")
        self._arm.set_max_acceleration_scaling_factor(accel)
        self._arm.set_max_velocity_scaling_factor(vel)
        # rospy.loginfo(self._robot.get_joint_names("arm0"))
        # rosp
        
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

        self.SAFE_JOINT2 = 90.0/180.0 * np.pi
        # self._ik_request.attempts = 1000
        self._enable_joints_publisher = rospy.Publisher('arm0_controller/enable_joints', Bool, queue_size=1)
        self._enable_joints_publisher.publish(True)


        self.MIDDLE_POINT_X = -0.52
        self.MIDDLE_POINT_Y = 0.35

        self._listener = tf.TransformListener()
        self.MIDDLE_POINT_RADIUS = 0.4

        self._listener.waitForTransform("/world", "/base/robot_tip", rospy.Time(), rospy.Duration(4.0))
        try:
            (trans,rot) = self._listener.lookupTransform('/world', '/base/robot_tip', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self._base_posi = trans


    def goStartup(self):
        for i in range(2):
            self.gripperMove(i, 0, False)
        self._arm.set_named_target("startup")
        self._arm.go()

    def goHome(self, color):
        # for i in range(2):
        #     self.gripperMove(i, 0, False)
        self._arm.set_named_target("home_"+color)
        self._arm.go()

    def setTargetPose(self, target_pose):
        self._target_pose = target_pose

    def getTargetPose(self):
        return self._target_pose

    def go(self, mode ="normal"):
        # self._arm.set_pose_target(self._target_pose)
        
        # return self._arm.go()


        goal = self._target_pose
        rospy.loginfo(goal.position)
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
        # rospy.loginfo(request_value)
        # rospy.loginfo(self._ik_request.ik_link_names)

        self._arm.set_joint_value_target(joints[0:4])

        # self._arm.set_joint_value_target(goal, self._arm.get_end_effector_link(), True)
        dt = rospy.Time.now().to_sec() - now.to_sec()
        rospy.loginfo("IK time : {}".format(dt))

        # if mode == "safe":
        now = rospy.Time.now().to_sec()
        if mode == "normal":
            plan = self._arm.plan()
        elif mode == "cartesian":
            waypoints = [goal]
            (plan, fraction) = self._arm.compute_cartesian_path(waypoints, 100, 0)

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

    def calcInverseKinematics(self, goal):

        rospy.loginfo(goal.position)
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
        # rospy.loginfo(request_value)
        # rospy.loginfo(self._ik_request.ik_link_names)

        # self._arm.set_joint_value_target(goal, self._arm.get_end_effector_link(), True)
        dt = rospy.Time.now().to_sec() - now.to_sec()
        rospy.loginfo("IK time : {}".format(dt))

        return joints[0:4]
    


    def checkExtention(self, final_target_joints):
        #### move joint1 -> joint2 version
        current_state = self._robot.get_current_state()
        current_position = current_state.joint_state.position
        current_arm_position = current_position[:4]


        # move joint2 to safety point.
        ### assumption: ０度を超えない
        abs_current_joint2 = abs(current_arm_position[1])
        abs_target_joint2 = abs(final_target_joints[1])
        sign = np.sign(final_target_joints[1])

        
        if abs_current_joint2 >= self.SAFE_JOINT2 and abs_target_joint2 >= self.SAFE_JOINT2:
            rospy.loginfo("normal move")
            state= ArmExtensionState.NEAR2NEAR
        elif abs_current_joint2 < self.SAFE_JOINT2 and  abs_target_joint2 >= self.SAFE_JOINT2:
            state= ArmExtensionState.FAR2NEAR
        elif abs_current_joint2 >= self.SAFE_JOINT2 and abs_target_joint2 < self.SAFE_JOINT2:
            state= ArmExtensionState.NEAR2FAR
        elif abs_current_joint2 < self.SAFE_JOINT2 and abs_target_joint2 < self.SAFE_JOINT2:
            state= ArmExtensionState.FAR2FAR
        return state


    def moveJoint1First(self):
        #### move joint1 -> joint2 version
        final_target_joints = self.calcInverseKinematics(target_pose)
        
        current_state = self._robot.get_current_state()
        current_position = current_state.joint_state.position
        current_arm_position = current_position[:4]


        # move joint2 to safety point.
        ### assumption: ０度を超えない
        abs_current_joint2 = abs(current_arm_position[1])
        abs_target_joint2 = abs(final_target_joints[1])
        sign = np.sign(final_target_joints[1])

        
        
        if abs_current_joint2 >= self.SAFE_JOINT2 and abs_target_joint2 >= self.SAFE_JOINT2:
            rospy.loginfo("normal move")
            move_joint2_first = False
            move_join1_only = False
            abs_safe_joint2 = abs_target_joint2
        elif abs_current_joint2 < self.SAFE_JOINT2 and  abs_target_joint2 >= self.SAFE_JOINT2:
            move_joint2_first = True
            move_join1_only = False
            abs_safe_joint2 = abs_target_joint2
        elif abs_current_joint2 >= self.SAFE_JOINT2 and abs_target_joint2 < self.SAFE_JOINT2:
            move_joint2_first = False
            move_join1_only = True
            abs_safe_joint2 = abs_current_joint2
        elif abs_current_joint2 < self.SAFE_JOINT2 and abs_target_joint2 < self.SAFE_JOINT2:
            move_joint2_first = True
            move_join1_only = True
            abs_safe_joint2 = self.SAFE_JOINT2

        safe_joint2 = sign * abs_safe_joint2
        if move_joint2_first:
            rospy.loginfo("move joint2")
            target_joints = list(copy.deepcopy(current_arm_position))
            target_joints[1] = safe_joint2
            self.go2TargetJoints(target_joints)

        # move joints all except joint2
        if move_join1_only:
            rospy.loginfo("move joint1")
            target_joints = list(copy.deepcopy(final_target_joints))
            target_joints[1] = safe_joint2
            self.go2TargetJoints(target_joints)

        ret = self.go2TargetJoints(final_target_joints)
        return ret

    def arriveMiddlePoint(self, target_pose):
        if target_pose.position.x < -0.8:
            return
        final_target_joints = self.calcInverseKinematics(target_pose)
        state = self.checkExtention(final_target_joints)
        if state == ArmExtensionState.NEAR2NEAR or state == ArmExtensionState.FAR2NEAR:
            return
        if state == ArmExtensionState.NEAR2FAR or state == ArmExtensionState.FAR2FAR:
            
            x = target_pose.position.x - self._base_posi[0]
            y = target_pose.position.y - self._base_posi[1]

            rad = math.atan2(y,x)
            middle_pose = copy.deepcopy(target_pose)
            middle_pose.position.x = self.MIDDLE_POINT_RADIUS * math.cos(rad) + self._base_posi[0]
            middle_pose.position.y = self.MIDDLE_POINT_RADIUS * math.sin(rad) + self._base_posi[1]
            self.setTargetPose(middle_pose)
            ret = self.go()
        

    def move(self, target_pose):
        self.arriveMiddlePoint(target_pose)
        self.setTargetPose(target_pose)
        ret = self.go()

        return ret 

    
    def go2TargetJoints(self, target_joints, mode ="normal"):
        self._arm.set_joint_value_target(target_joints)

        now = rospy.Time.now().to_sec()
        if mode == "normal":
            plan = self._arm.plan()
        elif mode == "cartesian":
            waypoints = [goal]
            (plan, fraction) = self._arm.compute_cartesian_path(waypoints, 100, 0)

        dt = rospy.Time.now().to_sec() - now
        temp = "plan time : {}".format(dt)
        # rospy.loginfo(temp)

        now = rospy.Time.now().to_sec()
        ret = self._arm.execute(plan)

        dt = rospy.Time.now().to_sec() - now
        temp = "execute time : {}".format(dt)
        # rospy.loginfo(temp)

        # elif mode == "cartesian":
        #     waypoints = [goal]
        #     (plan, fraction) = self._arm.compute_cartesian_path(waypoints, 100, 0)
        #     ret = self._arm.execute(plan, wait=True)

        return ret

    def above(self, z):
        target_pose = self.getTargetPose()
        target_pose.position.z = z
        self.setTargetPose(target_pose)
        return self.go("cartesian")