#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class Float2JointState:
    def __init__(self, subscribe_name, publish_name, joint_name, arduino2moveit_func):
        rospy.Subscriber(subscribe_name, Float64, self.callback)
        self._joint_state_publisher = rospy.Publisher(publish_name, JointState, queue_size=1)
        self._state = JointState()
        self._state.name = [joint_name]
        self._state.header.frame_id ="world"
        self._state.position = [0] * len(self._state.name)#[0, 1.5707, -2.7925, 1.22173, 0]
        self.arduino2moveit = arduino2moveit_func
    
    def callback(self, msg):
        val = msg.data
        self._state.position[0] = self.arduino2moveit(val)
        self._joint_state_publisher.publish(self._state)

def arduino2moveitGripper(dist):
    move_small = 0.5
    return (0.115 - dist)/0.115/2.0  * move_small

def rad2deg(rad):
    return np.rad2deg(rad)


if __name__ == "__main__":
    rospy.init_node("arduino_sim")


    Float2JointState("gripper1", "gripper1_joint_state", "gripper/joint1", arduino2moveitGripper)
    Float2JointState("gripper2", "gripper2_joint_state", "gripper/joint2", arduino2moveitGripper)
    Float2JointState("guide", "guide_joint_state","guide/joint1",rad2deg)
    Float2JointState("sorter1", "sorter1_joint_state", "sorter/joint1",rad2deg)
    Float2JointState("sorter2", "sorter2_joint_state", "sorter/joint2",rad2deg)
    Float2JointState("sorter3", "sorter3_joint_state", "sorter/joint3",rad2deg)
    rospy.spin()