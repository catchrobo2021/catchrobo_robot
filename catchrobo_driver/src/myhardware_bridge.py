#!/usr/bin/env python
import os
import rospy
from motor_serial import MotorSerial
from motor_can import MotorCan
from myhardware_base import MyHardwareBase

class MyHardwareBridge(MyHardwareBase):
    def __init__(self, communication_freq, port,interface,MOTOR_NUM):
        self.MOTOR_NUM = MOTOR_NUM
        self._joint_data = [0] * 3 * self.MOTOR_NUM
        timeout = 1.0 / communication_freq / self.MOTOR_NUM
        if interface == "serial":
            self._motor_module = MotorSerial(port, timeout)
        elif interface == "can":
            try:
                self._motor_module = MotorCan(port, timeout)
                rospy.loginfo("Connected to port " + port)
            except:
                rospy.logfatal("Port "+ port + " not found")

    def communicate(self, joint_control):
        result = True
        for i in range(self.MOTOR_NUM):
            id = i + 1
            p_des = joint_control.position[i] * -1.0 # really?
            v_des = joint_control.velocity[i] * -1.0 # really?
            kp = joint_control.kp[i]
            kd = joint_control.kd[i]
            i_ff = joint_control.effort[i] * -1.0 # really?

            self._motor_module.send_command(id, p_des, v_des, kp, kd, i_ff)
            ret = self._motor_module.receive()
            if ret is not None:
                id, posi, vel,current= ret
                i = id - 1
                if i < self.MOTOR_NUM:
                    self._joint_data[i] = posi * -1.0 # really?
                    self._joint_data[i + self.MOTOR_NUM * 1] = vel * -1.0 # really?
                    self._joint_data[i + self.MOTOR_NUM * 2] = current * -1.0 # really?
            else:
                result = False
        return result

    def reset_robot(self):
        for i in range(self.MOTOR_NUM):
            self._motor_module.enable_motor(i + 1)
            self._motor_module.zero_position(i + 1)
        rospy.loginfo("Reset")

    def enable_all_joints(self):
        for i in range(self.MOTOR_NUM):
            self._motor_module.enable_motor(i + 1)
        rospy.loginfo("Enable all joints")

    def disable_all_joints(self):
        for i in range(self.MOTOR_NUM):
            self._motor_module.disable_motor(i + 1)
        rospy.loginfo("Disable all joints")

    def get_data(self):
        return self._joint_data