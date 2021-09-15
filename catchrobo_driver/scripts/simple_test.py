#!/usr/bin/env python
import rospy
import sys
sys.path.append('/home/yutakage/catkin_ws/src/catchrobo_robot/catchrobo_driver/src')
from myhardware_bridge import MyHardwareBridge
from catchrobo_msgs.msg import CatchroboJointControl
import os


class test:
    def __init__(self):
        rospy.init_node("joint_control")
        communication_freq = 100
        self._hardware = MyHardwareBridge(communication_freq, "can0","can",1)
        joint_control = CatchroboJointControl()
        joint_control.position = [0] * 12
        joint_control.velocity = [0] * 12
        joint_control.kp = [5] * 12
        joint_control.kd = [0] * 12
        joint_control.effort = [0] * 12
        self._joint_control = joint_control
        self._add = 0.1
        self._hardware.reset_robot()
        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controlCallback)
        rospy.spin()

    def controlCallback(self, event):
        target = self._joint_control.position[4]
        target += self._add
        if target > 12 or target < 0:
            self._add *= -1
        self._joint_control.position[4] = target
        self._hardware.communicate(self._joint_control)
        joint = self._hardware.get_data()


if __name__ == "__main__":
    test()