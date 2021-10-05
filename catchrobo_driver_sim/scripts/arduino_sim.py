#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander

from sensor_msgs.msg import JointState

class ArduinoSim:
    def __init__(self):
        self._robot = moveit_commander.RobotCommander()
        self._gripper = [
            moveit_commander.MoveGroupCommander("hand1"),
            moveit_commander.MoveGroupCommander("hand2"),
        ]
        rospy.Subscriber("arduino_command", JointState, self.jointControlCallback)

    def jointControlCallback(self, msg):
        state = msg
        name = state.name[0] 
        if name == "gripper1":
            dist = state.position[0]
            self._gripper[0].set_joint_value_target([dist, dist])
            self._gripper[0].go(False)
        elif name == "gripper2":
            dist = state.position[0]
            self._gripper[1].set_joint_value_target([dist, dist])
            self._gripper[1].go(False)
        


if __name__ == "__main__":
    rospy.init_node("arduino_sim")
    joint_controller = ArduinoSim()
    rospy.spin()