#!/usr/bin/env python
from pickle import FALSE
import rospy
import sys
sys.path.append('/home/yutakage/catkin_ws/src/catchrobo_robot/catchrobo_driver/src')
from myhardware_bridge import MyHardwareBridge
from catchrobo_msgs.msg import CatchroboJointControl
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import os
import math
import time
import rosparam

class joint_controller:
    def __init__(self):
        self.JOINT_NUM = 1 # number of joints
        rospy.init_node("joint_controller")
        rospy.Subscriber("set_servo_on", Bool, self.servoOnCallback)
        rospy.Subscriber("set_joint_goal", CatchroboJointControl, self.setJointGoalCallback)
        self._joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=100)

        # joint control command
        joint_control = CatchroboJointControl()
        joint_control.position = [0] * self.JOINT_NUM
        joint_control.velocity = [0] * self.JOINT_NUM
        joint_control.kp = [0] * self.JOINT_NUM
        joint_control.kd = [0] * self.JOINT_NUM
        joint_control.effort = [0] * self.JOINT_NUM
        self._joint_control = joint_control

        # joint state feedback
        joint_state = JointState()
        joint_state.name = [""] * self.JOINT_NUM
        joint_state.position = [0] * self.JOINT_NUM
        joint_state.velocity = [0] * self.JOINT_NUM
        joint_state.effort = [0] * self.JOINT_NUM
        for i in range(self.JOINT_NUM):
            joint_state.name[i] = rosparam.get_param("joint"+str(i+1)+"/name")
        self._joint_state = joint_state

        # get ros params
        self._joint_position_limit_max = []
        self._joint_position_limit_min = []
        self._joint_position_offset = []
        self._joint_velocity_limit = []
        self._joint_currernt_limit = []
        self._joint_kp = []
        self._joint_kd = []
        for i in range(self.JOINT_NUM):
            self._joint_position_limit_max.append(rosparam.get_param("joint"+str(i+1)+"/position/limit/max"))
            self._joint_position_limit_min.append(rosparam.get_param("joint"+str(i+1)+"/position/limit/min"))
            self._joint_position_offset.append(rosparam.get_param("joint"+str(i+1)+"/position/offset"))
            self._joint_velocity_limit.append(rosparam.get_param("joint"+str(i+1)+"/velocity/limit"))
            self._joint_currernt_limit.append(rosparam.get_param("joint"+str(i+1)+"/current/limit"))
            self._joint_kp.append(rosparam.get_param("joint"+str(i+1)+"/kp"))
            self._joint_kd.append(rosparam.get_param("joint"+str(i+1)+"/kd"))

        # init module
        communication_freq = 500
        self._hardware = MyHardwareBridge(communication_freq, "can0","can")
        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controlCallback)
        rospy.spin()

    def addOffset(self,position):
        for i in range(self.JOINT_NUM):
            position[i] += self._joint_position_offset[i]

    def communicate(self):
        #self.addOffset(self._joint_control.position)
        self._hardware.communicate(self._joint_control) # send data to joints
        joint = self._hardware.get_data()            # receive data from joints
        # publish joint states to topic
        self._joint_state.position = joint[:self.JOINT_NUM]
        self._joint_state.velocity = joint[self.JOINT_NUM:self.JOINT_NUM*2]
        self._joint_state.effort = joint[self.JOINT_NUM*2:self.JOINT_NUM*3]
        #self.addOffset(self._joint_state.position)
        self._joint_state_publisher.publish(self._joint_state)
    

    def safetyCheck(self):
        # check joint states
        for i in range(self.JOINT_NUM):
            error_flag = False
            if self._joint_state.position[i] > self._joint_position_limit_max[i] or self._joint_state.position[i] < self._joint_position_limit_min[i]:
                rospy.logerr_once("Joint "+ str(i+1) + " over position")
                error_flag = True
            if abs(self._joint_state.velocity[i]) > self._joint_velocity_limit[i]:
                rospy.logerr_once("Joint "+ str(i+1) + " over velocity")
                error_flag = True
            if abs(self._joint_state.effort[i]) > self._joint_currernt_limit[i]:
                rospy.logerr_once("Joint "+ str(i+1) + " over current")
                error_flag = True
            if error_flag is True:
                self._hardware.disable_all_joints()
    

    def servoOnCallback(self,data):
        if data.data is True:
            self._hardware.enable_all_joints()
        else:
            self._hardware.disable_all_joints()


    def setJointGoalCallback(self,data):
        invalid_goal_flag = False
        # check if recveived goal is valid
        for i in range(self.JOINT_NUM):
            if abs(self._joint_state.position[i] - data.position[i]) > 0.1:
                rospy.logerr_once("Joint "+ str(i+1) + " goal position gap detected")
                invalid_goal_flag = True
            if self._joint_position_limit_max[i] <= data.position[i] and data.position[i] <= self._joint_position_limit_min[i]:
                rospy.logerr_once("Joint "+ str(i+1) + " goal violating position limit")
                invalid_goal_flag = True
            if abs(data.velocity[i]) > self._joint_velocity_limit[i]:
                rospy.logerr_once("Joint "+ str(i+1) + " goal violating velocity limit")
                invalid_goal_flag = True
            if abs(data.effort[i]) > self._joint_currernt_limit[i]:
                rospy.logerr_once("Joint "+ str(i+1) + " goal violating current limit")
                invalid_goal_flag = True
                
        if invalid_goal_flag is True:
                rospy.logwarn_once("Invalid goal ignored")
        else:
            for i in range(self.JOINT_NUM):
                self._joint_control.position[i] = data.position[i]
                self._joint_control.velocity[i] = data.velocity[i]
                self._joint_control.effort[i] = data.effort[i]
                self._joint_control.kp[i] = self._joint_kp[i]
                self._joint_control.kd[i] = self._joint_kd[i]


    def controlCallback(self, event):
        self.communicate()
        self.safetyCheck()
        

if __name__ == "__main__":
    joint_controller()