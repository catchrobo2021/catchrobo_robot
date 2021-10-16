#!/usr/bin/env python
# -*- coding: utf-8 -*-

from operator import sub
import numpy as np
import rospy
import thread

from actionlib import SimpleActionServer
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult 

from sensor_msgs.msg import JointState


class Robot(object):
    def __init__(self):
        rospy.init_node("catch_control")
        self._as = SimpleActionServer("follow_joint_trajectory", 
            FollowJointTrajectoryAction,
            execute_cb=self.armExecuteCallback,
            auto_start=False)

        self._as.start()
        self._clock = rospy.get_param("trajectory_rate")
        self._period = 1.0 / rospy.get_param("trajectory_rate", 100)
        self._feedback_period = 1.0 / rospy.get_param("feedback_rate", 100)
        
        self._pub_cmd = rospy.Publisher("joint_control", JointState, queue_size=1)
        
        self._feedback = FollowJointTrajectoryFeedback()
        self._feedback.joint_names = rospy.get_param("joints")
        
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


if __name__ == "__main__":
    robot = Robot()
    rospy.spin()



        



