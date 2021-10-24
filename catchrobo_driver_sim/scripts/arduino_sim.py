#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class ArduinoSim:
    def __init__(self):
        self._robot = moveit_commander.RobotCommander()
        self._gripper = [
            moveit_commander.MoveGroupCommander("hand1"),
            moveit_commander.MoveGroupCommander("hand2"),
        ]
        # rospy.Subscriber("arduino_command", JointState, self.jointControlCallback)
        rospy.Subscriber("gripper1", Float64, self.gripper1Callback)
        rospy.Subscriber("gripper2", Float64, self.gripper2Callback)

    def arduino2moveit(self, dist):
        move_small = 0.5
        return (0.115 - dist)/0.115/2.0  * move_small

    def gripper1Callback(self, msg):
        val = msg.data
        dist = self.arduino2moveit(val)

        self._gripper[0].set_joint_value_target([dist, dist])
        self._gripper[0].go(False)
    
    def gripper2Callback(self, msg):
        dist = msg.data
        self._gripper[1].set_joint_value_target([dist, dist])
        self._gripper[1].go(False)


    # def jointControlCallback(self, msg):
    #     state = msg
    #     name = state.name[0] 
    #     if name == "gripper1":
    #         dist = state.position[0]
    #         self._gripper[0].set_joint_value_target([dist, dist])
    #         self._gripper[0].go(False)
    #     elif name == "gripper2":
    #         dist = state.position[0]
    #         self._gripper[1].set_joint_value_target([dist, dist])
    #         self._gripper[1].go(False)
        


if __name__ == "__main__":
    rospy.init_node("arduino_sim")
    joint_controller = ArduinoSim()
    rospy.spin()