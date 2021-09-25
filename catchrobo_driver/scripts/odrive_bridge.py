#!/usr/bin/env python3
from odrive_node import ODriveNode
from sensor_msgs.msg import JointState

class Motor:
    def __init__(self, serial_number, axis,driver_id):
        self.serial_number = serial_number
        self.axis = axis
        self.driver_id = driver_id


class ODriveBridge:
    def __init__(self, MOTOR_NUM, config):
        self.MOTOR_NUM = MOTOR_NUM # number of joints
        self.ODRIVE_NUM = (MOTOR_NUM+1) // 2
        # odrive node 
        odrive1 = ODriveNode()
        odrive2 = ODriveNode()
        odrive3 = ODriveNode()
        self._odrv = [odrive1,odrive2,odrive3]
        self._serial_number = [config['odrive_serial_number']['odrive1'],config['odrive_serial_number']['odrive2'],config['odrive_serial_number']['odrive3']]    
        self._joint_config = [  [config['joint1']['odrive']['id'], config['joint1']['odrive']['axis']],
                                [config['joint2']['odrive']['id'], config['joint2']['odrive']['axis']],
                                [config['joint3']['odrive']['id'], config['joint3']['odrive']['axis']],
                                [config['joint4']['odrive']['id'], config['joint4']['odrive']['axis']],
                                [config['joint5']['odrive']['id'], config['joint5']['odrive']['axis']]]

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

    def idle_all(self):
        for i in range(self.MOTOR_NUM):
            self.idle(joint=i, index_search=False)

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
            motor_state.effort = self._odrv[self._joint_config[i][0]].get_current(axis = self._joint_config[i][1])
        return motor_state

    def search_index(self):
        search_index