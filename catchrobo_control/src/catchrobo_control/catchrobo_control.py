#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from actionlib import SimpleActionServer
from control_msgs.msg import FollowJointTrajectoryAction 

from sensor_msgs.msg import JointState

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest
from moveit_msgs.srv import GetPositionIK


class Robot(object):
    def __init__(self):
        rospy.init_node("robot_controller")
        self._action_server = SimpleActionServer("arm0_controller/follow_joint_trajectory", 
            FollowJointTrajectoryAction,
            execute_cb=self.armExecuteCallback,
            auto_start=False)
        
        self._pub_jointstate = rospy.Publisher("/joint_states", JointState)
        self._action_server.start()
        self._joint_states = JointState()
        
    def pubJointState(self):
        self._pub_jointstate(self._joint_states)
    
    def armExecuteCallback(self, goal):
        points = goal.action_goal.goal.joint_trajectory.points
        point_num = len(points)
        target_point_id = 0
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            target_point = points[target_point_id]

            now = rospy.Time.now()
            self._joint_states.header.stamp = now
            self._joint_states.position[:5] = target_point.positions
            
            self.pubJointState()
            rest_time = target_point.time_from_start -  (now - start_time)
            rospy.sleep(rest_time)


        now = rospy.Time.now().to_sec()



if __name__ == "__main__":
    robot = Robot()
    rospy.spin()



        



