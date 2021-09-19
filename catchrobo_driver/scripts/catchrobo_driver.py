#!/usr/bin/env python3
from logging import error
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool,Float32
from catchrobo_msgs.msg import CatchroboJointControl
import rosparam
import roslib
roslib.load_manifest('diagnostic_updater')
import diagnostic_updater
import diagnostic_msgs
import odrive
from odrive.enums import *
import time

class catchrobo_driver:
    def __init__(self):
        self.JOINT_NUM = 1 # number of joints
        rospy.init_node("catcrobo_driver")
        rospy.Subscriber("enable_joints", Bool, self.enableJointsCallback)
        rospy.Subscriber("joint_control", CatchroboJointControl, self.jointControlCallback)
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
            joint_state.name[i] = rosparam.get_param("arm/joint"+str(i+1)+"/name") 
        self._joint_state = joint_state

        # state flags
        self._joint_err_state = False    # nominal=True, error=False
        self._joint_com_state = False    # nominal=True, lost=False
        self._joint_enable_state = False # enabled=True, disabled=False
        self._joint_err_state_old = True
        self._robot_state = 0          # OK=0, WARN=1, ERROR=2, STALE=3
        self._joint_error_message = ""

        # get ros params
        self._joint_position_limit_max = []
        self._joint_position_limit_min = []
        self._joint_position_offset = []
        self._joint_position_tolerence = []
        self._joint_velocity_limit = []
        self._joint_currernt_limit = []
        for i in range(self.JOINT_NUM):
            self._joint_position_limit_max.append(rosparam.get_param("arm/joint"+str(i+1)+"/position/limit/max"))
            self._joint_position_limit_min.append(rosparam.get_param("arm/joint"+str(i+1)+"/position/limit/min"))
            self._joint_position_offset.append(rosparam.get_param("arm/joint"+str(i+1)+"/position/offset"))
            self._joint_position_tolerence.append(rosparam.get_param("arm/joint"+str(i+1)+"/position/tolerence"))
            self._joint_velocity_limit.append(rosparam.get_param("arm/joint"+str(i+1)+"/velocity/limit"))
            self._joint_currernt_limit.append(rosparam.get_param("arm/joint"+str(i+1)+"/current/limit"))

        # set kp, kd
        #for i in range(self.JOINT_NUM):
        #    joint_control.kp[i]  = rosparam.get_param("arm/joint"+str(i+1)+"/kp")
        #    joint_control.kd[i]  = rosparam.get_param("arm/joint"+str(i+1)+"/kd")

        # set diagnostic updater
        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID("arm")
        self._diagnostic_updater.add("robot_state",self.diagnosticUpdater)

        # init module
        communication_freq = rosparam.get_param("arm/communincation_frequency") 
        self.my_drive = odrive.find_any()
        self.searchIndex()
        rospy.loginfo("catchrobo driver is ready")
        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controllCallback)
        rospy.spin()

    def convertJointToMotor(self,position):
        pass

    def convertMotorToJoint(self,position):
        pass

    def read(self):
        self._joint_state.position[0] = self.my_drive.axis0.encoder.pos_estimate
        self._joint_state.velocity[0] = self.my_drive.axis0.encoder.vel_estimate
        self._joint_state.effort[0] = self.my_drive.axis0.motor.current_control.Iq_measured
        #self.convertMotorToJoint(self._joint_state)
        self._joint_state_publisher.publish(self._joint_state)

    def write(self):
        #self.convertJointToMotor(_joint_control)
        self.my_drive.axis0.controller.input_pos = self._joint_control.position[0]
        pass
        

    def disableAllJoints(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_IDLE
        self._joint_enable_state = False


    def enableAllJoints(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self._joint_enable_state = True

    def searchIndex(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        while self.my_drive.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

    def safetyCheck(self):
        self._joint_err_state = True
        self._joint_error_message = ""
        for i in range(self.JOINT_NUM):
            if self._joint_state.position[i] > self._joint_position_limit_max[i] or self._joint_state.position[i] < self._joint_position_limit_min[i]:
                self._joint_err_state = False
                self._joint_error_message += ("Joint "+ str(i+1) + " over position, ")
            if abs(self._joint_state.velocity[i]) > self._joint_velocity_limit[i]:
                self._joint_err_state = False
                self._joint_error_message += ("Joint "+ str(i+1) + " over velocity, ")
            if abs(self._joint_state.effort[i]) > self._joint_currernt_limit[i]:
                self._joint_err_state = False
                self._joint_error_message += ("Joint "+ str(i+1) + " over current, ")
        if self._joint_err_state is False and self._joint_err_state_old is True:
            self.disableAllJoints()
        if self._joint_err_state is True and self._joint_err_state_old is False:
            rospy.loginfo("Robot recovered from error state")
        self._joint_err_state_old = self._joint_err_state

        
    def diagnosticUpdater(self,state):
        if self._joint_com_state is True:
            if self._joint_err_state is True:
                if self._joint_enable_state is True:
                    self._robot_state = 0
                    state.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Ready")
                else:
                    self._robot_state = 1
                    state.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Joints disabled")
            else:
                self._robot_state = 2
                state.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,self._joint_error_message )
        else:
            self._robot_state = 3
            state.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE, "Not connected")


    def enableJointsCallback(self,data):
        # TODO
        if self._robot_state < 4:
            if data.data is True:
                self.enableAllJoints()
            else:
                self.disableAllJoints()
        else:
            rospy.logerr("Robot is not connected. Failed to send enable/disable command")


    def jointControlCallback(self,data):
        invalid_goal_flag = False

        # check error state
        #if self._robot_state is not 0:
        #    rospy.logwarn("Robot is not ready")
        #    invalid_goal_flag = True

        # check if recveived goal is valid
        for i in range(self.JOINT_NUM):
            if abs(self._joint_state.position[i] - data.position[i]) > self._joint_position_tolerence[i]:
                rospy.logwarn("Joint "+ str(i+1) + " target position gap detected")
                invalid_goal_flag = True
            if self._joint_position_limit_max[i] <= data.position[i] and data.data[i] <= self._joint_position_limit_min[i]:
                rospy.logwarn("Joint "+ str(i+1) + " target position violating joint limit")
                invalid_goal_flag = True
                
        if invalid_goal_flag is False:
            for i in range(self.JOINT_NUM):
                self._joint_control.position[i] = data.position[i]
                self._joint_control.velocity[i] = data.velocity[i]
                self._joint_control.effort[i] = data.effort[i]
        else:
            rospy.logwarn("Invalid goal ignored")
            
    def controllCallback(self,event):
        # over write control command if joints are not enabled:
        if self._joint_enable_state is False:
            for i in range(self.JOINT_NUM):
                self._joint_control.position[i] = self._joint_state.position[i]
        self.read()
        self.safetyCheck()
        self._diagnostic_updater.update()
        self.write()
        # check communication state

if __name__ == "__main__":
    catchrobo_driver()