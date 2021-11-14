#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from sensor_msgs.msg import JointState
import math

class PositionConverter:
    def __init__(self, joint_offset, MOTOR_NUM=4):
        self.MOTOR_NUM = MOTOR_NUM
        self.__jointOffset = joint_offset

    def convert_round_to_rad(self, round):
        rad = [0] * self.MOTOR_NUM
        for i in range(self.MOTOR_NUM):
            rad[i] = round[i] * 2 * math.pi
        return rad


    def convert_rad_to_round(self, rad):
        round = [0] * self.MOTOR_NUM
        for i in range(self.MOTOR_NUM):
            round[i] = rad[i] / (2.0 * math.pi)
        return round


    def convert_motor_to_joint(self, motor_state_round):
        motor_state_rad = JointState()
        motor_state_rad.name = [""] * self.MOTOR_NUM
        motor_state_rad.position = [0] * self.MOTOR_NUM
        motor_state_rad.velocity = [0] * self.MOTOR_NUM
        motor_state_rad.effort = [0] * self.MOTOR_NUM

        motor_state_rad.position = self.convert_round_to_rad(motor_state_round.position)
        motor_state_rad.velocity = self.convert_round_to_rad(motor_state_round.velocity)

        joint_state = JointState()
        joint_state.name = [""] * self.MOTOR_NUM
        joint_state.position = [0] * self.MOTOR_NUM
        joint_state.velocity = [0] * self.MOTOR_NUM
        joint_state.effort = [0] * self.MOTOR_NUM

        joint_state.position[0] = -motor_state_rad.position[0] / 8.0 + self.__jointOffset[0]
        joint_state.position[1] = -motor_state_rad.position[1] / 8.0 + self.__jointOffset[1]
        joint_state.position[2] = +motor_state_rad.position[2] / 2.0 + self.__jointOffset[2]
        joint_state.position[3] = +motor_state_rad.position[3] / (2 * math.pi) * 0.008 + self.__jointOffset[3]

        joint_state.velocity[0] = -motor_state_rad.velocity[0] / 8.0
        joint_state.velocity[1] = -motor_state_rad.velocity[1] / 8.0
        joint_state.velocity[2] = +motor_state_rad.velocity[2] / 2.0
        joint_state.velocity[3] = +motor_state_rad.velocity[3] / (2 * math.pi) * 0.008

        return joint_state


    def convert_joint_to_motor(self, joint_state):
        motor_state_rad = JointState()
        motor_state_rad.name = [""] * self.MOTOR_NUM
        motor_state_rad.position = [0] * self.MOTOR_NUM
        motor_state_rad.velocity = [0] * self.MOTOR_NUM
        motor_state_rad.effort = [0] * self.MOTOR_NUM
        
        motor_state_rad.position[0] = -(joint_state.position[0] - self.__jointOffset[0]) * 8.0
        motor_state_rad.position[1] = -(joint_state.position[1] - self.__jointOffset[1]) * 8.0
        motor_state_rad.position[2] = +(joint_state.position[2] - self.__jointOffset[2]) * 2.0
        motor_state_rad.position[3] = +(joint_state.position[3] - self.__jointOffset[3]) * (2 * math.pi) / 0.008

        motor_state_rad.velocity[0] = -joint_state.velocity[0] * 8.0
        motor_state_rad.velocity[1] = -joint_state.velocity[1] * 8.0
        motor_state_rad.velocity[2] = +joint_state.velocity[2] * 2.0
        motor_state_rad.velocity[3] = +joint_state.velocity[3] * (2 * math.pi) / 0.008
        
        motor_state_round = JointState()
        motor_state_round.name = [""] * self.MOTOR_NUM
        motor_state_round.position = [0] * self.MOTOR_NUM
        motor_state_round.velocity = [0] * self.MOTOR_NUM
        motor_state_round.effort = [0] * self.MOTOR_NUM

        motor_state_round.position = self.convert_rad_to_round(motor_state_rad.position)
        motor_state_round.velocity = self.convert_rad_to_round(motor_state_rad.velocity)

        return motor_state_round