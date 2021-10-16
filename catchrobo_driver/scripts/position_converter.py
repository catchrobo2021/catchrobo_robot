#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import rosparam
import roslib
import math

class PositionConverter:
    def __init__(self, joint_offset, MOTOR_NUM=5):
        # reductionRatio[0~2]は各アクチュエータの減速比(J2 is 35:72)
        # reductionRatio[3]=1.0は第三アクチュエータベルトにおける、第二関節と第三関節間の減速比
        # reductionRatio[4]=2.0は第四・五アクチュエータベルトにおける、根本と第三関節間の減速比
        # reductionRatio[5]=1.0は第四・五アクチュエータベルトにおける、第三関節と第四・五関節間の減速比
        # reductionRatio[6]=1.4は傘歯車の減速比(25:35)
        self.__reductionRatio = [8.0, 16.45, 8.0, 1.0, 2.0, 1.0, 1.4] 
        self.MOTOR_NUM = MOTOR_NUM
        # オフセット量 (rad)
        self.__jointOffset = joint_offset
        rospy.loginfo(self.__jointOffset)

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

    def convert_motor_to_joint(self, motor_state_rad):
        # 回転位相情報を角度情報に変換
        motor_state = JointState()
        motor_state.name = [""] * self.MOTOR_NUM
        motor_state.position = [0] * self.MOTOR_NUM
        motor_state.velocity = [0] * self.MOTOR_NUM
        motor_state.effort = [0] * self.MOTOR_NUM
        motor_state.position = self.convert_round_to_rad(motor_state_rad.position)
        #### 仮定: モータは、出力側から見て時計周りにベルトが回転する向きが正である ####
        # 第一関節　joint-motor 方向不一致
        # 第二関節　joint-motor 方向不一致
        # 第三関節　joint-motor 方向不一致
        # 第四関節　joint-motor 方向一致
        # 第五関節　joint-motor 方向不一致
        joint_state = JointState()
        joint_state.name = [""] * self.MOTOR_NUM
        joint_state.position = [0] * self.MOTOR_NUM
        joint_state.velocity = [0] * self.MOTOR_NUM
        joint_state.effort = [0] * self.MOTOR_NUM
        # Offsetしていない状態でのJoint角
        # ずれを含んだ角度であるため、数値の意味を持たない
        joint_state_without_offset = [0.0, 0.0, 0.0, 0.0, 0.0]
        # 他のモータの影響で生じたずれ[rad]
        listAffectedDiff = [0.0, 0.0, 0.0, 0.0, 0.0]
        # 第一関節
        joint_state_without_offset[0] = (-motor_state.position[0] / self.__reductionRatio[0]) 
        joint_state.position[0] = joint_state_without_offset[0] + self.__jointOffset[0]
        # 第二関節
        joint_state_without_offset[1] = (-motor_state.position[1] / self.__reductionRatio[1])
        joint_state.position[1] = joint_state_without_offset[1] + self.__jointOffset[1]
        # 第三関節
        listAffectedDiff[2] = -joint_state.position[1] / self.__reductionRatio[3] # 特に符号に注意
        joint_state_without_offset[2] = (motor_state.position[2] / self.__reductionRatio[2]) + listAffectedDiff[2] 
        joint_state.position[2] = joint_state_without_offset[2] + self.__jointOffset[2]
        # 手先
        # 減速比
        # TODO: 第五関節が未調整
        listAffectedDiff[3] = -joint_state.position[1] / self.__reductionRatio[4] - joint_state.position[2] / self.__reductionRatio[5] # 特に符号に注意 & joint_state.position[2]なのか,(joint_state.position[2]-joint_state.position[1])なのか迷う
        listAffectedDiff[4] = -joint_state.position[1] / self.__reductionRatio[4] - joint_state.position[2] / self.__reductionRatio[5] 
        joint_state_without_offset[3] = (-motor_state.position[3] + motor_state.position[4]) / 2.0 / self.__reductionRatio[4] + listAffectedDiff[3]
        joint_state_without_offset[4] = (-motor_state.position[3] - motor_state.position[4]) / self.__reductionRatio[6] / self.__reductionRatio[4] / 2.0
        joint_state.position[3] = joint_state_without_offset[3] + self.__jointOffset[3]
        joint_state.position[4] = joint_state_without_offset[4] + self.__jointOffset[4]
        return joint_state

    def convert_joint_to_motor(self, joint_state):
        motor_state = JointState()
        motor_state.name = [""] * self.MOTOR_NUM
        motor_state.position = [0] * self.MOTOR_NUM
        motor_state.velocity = [0] * self.MOTOR_NUM
        motor_state.effort = [0] * self.MOTOR_NUM
        joint_state_without_offset = [0.0, 0.0, 0.0, 0.0, 0.0]
        listAffectedDiff = [0.0, 0.0, 0.0, 0.0, 0.0]
        # 第一関節
        motor_state.position[0] = (-joint_state.position[0] + self.__jointOffset[0]) * self.__reductionRatio[0]
        # 第二関節
        motor_state.position[1] = (-joint_state.position[1] + self.__jointOffset[1]) * self.__reductionRatio[1]
        # 第三関節
        listAffectedDiff[2] = -joint_state.position[1] / self.__reductionRatio[3]
        motor_state.position[2] = (joint_state.position[2] - self.__jointOffset[2] - listAffectedDiff[2]) * self.__reductionRatio[2]
        # 第四関節・第五関節
        listAffectedDiff[3] = -joint_state.position[1] / self.__reductionRatio[4] - joint_state.position[2] / self.__reductionRatio[5]
        joint_state_without_offset[3] = joint_state.position[3] - self.__jointOffset[3]
        joint_state_without_offset[4] = joint_state.position[4] - self.__jointOffset[4]
        invp3_add_p4 = (joint_state_without_offset[3] - listAffectedDiff[3]) * 2.0 * self.__reductionRatio[4]
        invp3_diff_p4 = (joint_state_without_offset[4]) * 2.0 * self.__reductionRatio[6] * self.__reductionRatio[4]
        motor_state.position[3] = -(invp3_add_p4 + invp3_diff_p4) / 2.0
        motor_state.position[4] = (invp3_add_p4 - invp3_diff_p4) / 2.0
        # 角度情報を回転位相情報に変換
        motor_state_round = JointState()
        motor_state_round.name = [""] * self.MOTOR_NUM
        motor_state_round.position = [0] * self.MOTOR_NUM
        motor_state_round.velocity = [0] * self.MOTOR_NUM
        motor_state_round.effort = [0] * self.MOTOR_NUM
        motor_state_round.position = self.convert_rad_to_round(motor_state.position)
        return motor_state_round


