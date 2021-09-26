#!/usr/bin/env python
# -*- coding: utf-8 -*-

from operator import sub
import numpy as np
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

        self._as.start()
        self._clock = rospy.get_param("trajectory_rate")
        self._period = 1.0 / rospy.get_param("trajectory_rate")
        self._feedback_period = 1.0 / rospy.get_param("feedback_rate")
        
        self._pub_cmd = rospy.Publisher("joint_control", JointState, queue_size=1)
        
        self._feedback = FollowJointTrajectoryFeedback()
        self._feedback.joint_names = ["arm/joint{}".format(i+1) for i in range(5)]

        
        self._joint_cmd = JointState()
        self._joint_cmd.name = self._feedback.joint_names

        self._lock = thread.allocate_lock()

        # self._timer = None
        # self._feedback_timer = None

        rospy.Subscriber("joint_states", JointState, self.readRobotCallback)
        
    
    def readRobotCallback(self, msg):
        with self._lock:
            self._joint_states = msg
        

    def armExecuteCallback(self, goal):

        # self.timerDelete()
        self._goal = goal
        
        points = goal.trajectory.points
        start_time = points[0].time_from_start
        action_time = points[-1].time_from_start - start_time
        
        periodic_trajectory_num = int(action_time.to_sec() //  self._period)
        rospy.loginfo("periodic_trajectory_num {}".format(periodic_trajectory_num))

        point_num = len(points)
        step = 0

        # self._periodic_trajectory = []
        rate = rospy.Rate(self._clock)
        for i in range(periodic_trajectory_num):
            t = rospy.Duration(i * self._period) + start_time

            while points[step+1].time_from_start < t:
                step+=1
            
            y_2 = np.array(points[step+1].positions)
            y_1 = np.array(points[step].positions)
            t_2_t_1 = (points[step+1].time_from_start - points[step].time_from_start).to_sec()
            t_t_1 = (t - points[step].time_from_start).to_sec()

            y =(y_2 - y_1)/t_2_t_1  *t_t_1  + y_1

            # self._periodic_trajectory.append(y.tolist())
            self._joint_cmd.position = y.tolist()
            self._pub_cmd.publish(self._joint_cmd)

            if self._as.is_preempt_requested():
				self._as.set_preempted()
            
            now = rospy.Time.now()
            self._feedback.desired = points[step]
            self._feedback.actual.time_from_start = now - start_time
            with self._lock:
                self._feedback.actual.positions = self._joint_states.position
                self._feedback.actual.velocities = self._joint_states.velocity
            self._as.publish_feedback(self._feedback)

            rate.sleep()
        self._joint_cmd.position = points[-1].positions
        self._pub_cmd.publish(self._joint_cmd)


        result = FollowJointTrajectoryResult()
        result.error_code = 0
        self._as.set_succeeded(result)

    # def timerStart(self):
    #     self._timer = rospy.Timer(rospy.Duration(self._period), self.timerCallback)
    #     self._feedback_timer = rospy.Timer(rospy.Duration(self._feedback_period), self.feedbackTimerCallback)

    # def timerDelete(self):
    #     if self._timer is not None:
    #         self._timer.shutdown()
    #         self._feedback_timer.shutdown()
        
    #     self._timer =None
    #     self._feedback_timer=None

    # def timerCallback(self, event):
    #     if self._as.is_preempt_requested():
    #         self._as.set_preempted()
    #     self._joint_cmd.position = self._periodic_trajectory[self._periodic_step]
    #     self._pub_cmd.publish(self._joint_cmd)
    #     self._periodic_step +=1
    #     if self._periodic_step == len(self._periodic_trajectory):
    #         result = FollowJointTrajectoryResult()
    #         result.error_code = 0
    #         self._as.set_succeeded(result)
    #         self.timerDelete()
            

    # def feedbackTimerCallback(self, event):
    #     with self._lock:
    #         self._feedback.actual.positions = self._joint_states.position
    #         self._feedback.actual.velocities = self._joint_states.velocity
    #     now = rospy.Time.now()
    #     self._feedback.actual.time_from_start = now - self._start_t

    #     point = JointTrajectoryPoint()
    #     point.positions = self._periodic_trajectory[self._periodic_step]
    #     point.time_from_start = self._feedback.actual.time_from_start
    #     self._feedback.desired = point

    #     self._as.publish_feedback(self._feedback)


if __name__ == "__main__":
    robot = Robot()
    rospy.spin()



        



