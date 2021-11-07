#!/usr/bin/env python3
from logging import error
from odrive_node import ODriveNode
from sensor_msgs.msg import JointState
import time
import rospy
import odrive
from odrive.enums import *
from odrive.utils import *



class ODriveBridge:
    def __init__(self, MOTOR_NUM, config):
        self.MOTOR_NUM = MOTOR_NUM # number of joints
        self.ODRIVE_NUM = (MOTOR_NUM+1) // 2
        # odrive node 
        odrive0 = ODriveNode()
        odrive1 = ODriveNode()
        odrive2 = ODriveNode()

        self._odrv = [odrive0,odrive1,odrive2]
        self._serial_number = [config['odrive_serial_number']['odrive0'],config['odrive_serial_number']['odrive1']]    
        self._joint_config = [  [config['joint1']['odrive']['id'], config['joint1']['odrive']['axis']],
                                [config['joint2']['odrive']['id'], config['joint2']['odrive']['axis']],
                                [config['joint3']['odrive']['id'], config['joint3']['odrive']['axis']],
                                [config['joint4']['odrive']['id'], config['joint4']['odrive']['axis']]]

    def connect(self):
        for i in range(self.ODRIVE_NUM):
            self._odrv[i].connect(serial_number=self._serial_number[i])

    def disconnect(self):
        for i in range(self.ODRIVE_NUM):
            self._odrv[i].disconnect()
        
    def engage(self, joint, index_search=False):
        if joint < self.MOTOR_NUM :
            self._odrv[self._joint_config[joint][0]].engage(axis=self._joint_config[joint][1], mode=index_search)
        else:
            pass
            
    def idle(self, joint):
        if joint < self.MOTOR_NUM :
            self._odrv[self._joint_config[joint][0]].idle(axis=self._joint_config[joint][1])

    def engage_all(self,index_search=False):
        for i in range(self.MOTOR_NUM):
            self.engage(joint=i, index_search=index_search)
            time.sleep(0.1)

    def idle_all(self):
        for i in range(self.MOTOR_NUM):
            self.idle(joint=i)

    def set_mode(self, mode):
        for i in range(self.MOTOR_NUM):
            self._odrv[self._joint_config[i][0]].control_mode(axis=self._joint_config[i][1], mode=mode)
            
    def write(self, position):
        for i in range(self.MOTOR_NUM):
            self._odrv[self._joint_config[i][0]].drive_pos(axis=self._joint_config[i][1], val=position[i])
    
    def read(self):
        motor_state = JointState()
        motor_state.name = [""] * self.MOTOR_NUM
        motor_state.position = [0] * self.MOTOR_NUM
        motor_state.velocity = [0] * self.MOTOR_NUM
        motor_state.effort = [0] * self.MOTOR_NUM
        for i in range(self.MOTOR_NUM):
            motor_state.position[i] = self._odrv[self._joint_config[i][0]].get_pos(axis = self._joint_config[i][1])
            motor_state.velocity[i] = self._odrv[self._joint_config[i][0]].get_vel(axis = self._joint_config[i][1])
            motor_state.effort[i] = self._odrv[self._joint_config[i][0]].get_current(axis = self._joint_config[i][1])
        return motor_state

    def search_index(self,joint):
        self._odrv[self._joint_config[joint][0]].search_index(axis=self._joint_config[joint][1])
        rospy.loginfo("Joint " + str(joint+1) + " index search finished")
        return True

    def search_index_all(self):
        for i in range(self.MOTOR_NUM):
            self.search_index(joint=i)
            time.sleep(2.0)
    
    def get_errors(self,joint,clear=True):
        error_string = self._odrv[self._joint_config[joint][0]].get_errors(clear=clear)
        return error_string

    def get_errors_all(self,clear=True):
        error_string = ""
        for i in range(self.ODRIVE_NUM):
            error_string_joint = self._odrv[i].get_errors(clear=clear)
            if error_string_joint is not None:
                error_string += error_string_joint
        return error_string

    def hard_stop(self,joint,current_limit=1,direction=1,velocity=0.1):
        time.sleep(3.0)
        self.engage(joint=joint,index_search=False)
        time.sleep(0.5)
        start_position = self._odrv[self._joint_config[joint][0]].get_pos(axis = self._joint_config[joint][1])
        target_position = start_position
        while True:
            if abs(self._odrv[self._joint_config[joint][0]].get_current(axis=self._joint_config[joint][1])) > current_limit:
                self.idle(joint=joint)
                break
            else:
                target_position += direction * velocity * 0.05
                self._odrv[self._joint_config[joint][0]].drive_pos(axis=self._joint_config[joint][1], val=target_position)
                time.sleep(0.05)
        self.search_index(joint=joint)