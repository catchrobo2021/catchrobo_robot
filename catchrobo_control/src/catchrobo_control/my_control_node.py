#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import thread
import actionlib

from actionlib import SimpleActionServer
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult 

from sensor_msgs.msg import JointState

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest
from moveit_msgs.srv import GetPositionIK

from catchrobo_control.myrobot import MyRobot

class Robot(object):
    def __init__(self):
        rospy.init_node("catch_control")
        self._as = SimpleActionServer("follow_joint_trajectory", 
            FollowJointTrajectoryAction,
            execute_cb=self.armExecuteCallback,
            auto_start=False)
        self._myrobot = MyRobot()
        self._pub_jointstate = rospy.Publisher("controller_joint_states", JointState, queue_size=1)
        self._as.start()
        self._joint_states = JointState()
        self._joint_states.name = ["arm/joint{}".format(i+1) for i in range(5)]
        self._lock = thread.allocate_lock()
        
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self._joint_states.header.stamp = now
            self._joint_states.position = self._myrobot.read()
            self._pub_jointstate.publish(self._joint_states)
            rate.sleep()

    
    def armExecuteCallback(self, goal):
        result = FollowJointTrajectoryResult()
        self._feedback = FollowJointTrajectoryFeedback()
        self._feedback.joint_names = goal.trajectory.joint_names

        points = goal.trajectory.points
        point_num = len(points)
        target_point_id = 0
        start_time = rospy.Time.now()


        last_point = goal.trajectory.points[0]
        for point in goal.trajectory.points[1:] :

            self._myrobot.write(point.positions)
            
            if self._as.is_preempt_requested():
				self._as.set_preempted()
            
            now = rospy.Time.now()
            self._feedback.desired = point
            with self._lock:
                self._feedback.actual.positions = self._joint_states.position
                self._feedback.actual.velocities = self._joint_states.velocity
                self._feedback.actual.time_from_start = now - start_time
            self._as.publish_feedback(self._feedback)
            # ---------------------------------------
            last_point = point

            
            rest_time = point.time_from_start -  (now - start_time)
            rospy.sleep(rest_time)

        result.error_code = 0
        self._as.set_succeeded(result)


if __name__ == "__main__":
    robot = Robot()
    robot.spin()



        



