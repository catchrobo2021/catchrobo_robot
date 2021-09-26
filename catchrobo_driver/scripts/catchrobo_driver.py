#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import rosparam
import roslib
roslib.load_manifest('diagnostic_updater')
import diagnostic_updater
import diagnostic_msgs
from odrive.enums import *
import time
from odrive_bridge import ODriveBridge


class catchrobo_driver:
    def __init__(self):
        self.MOTOR_NUM = 4 # number of joints
        rospy.init_node("catcrobo_driver")
        rospy.Subscriber("enable_joints", Bool, self.engage_idle_callback)
        rospy.Subscriber("joint_control", JointState, self.joint_control_callback)
        self._joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=100)

        # joint control
        joint_control = JointState()
        joint_control.name = [""]
        joint_control.position = [0] * self.MOTOR_NUM
        joint_control.velocity = [0] * self.MOTOR_NUM
        joint_control.effort = [0] * self.MOTOR_NUM
        self._joint_control = joint_control

        # joint state
        joint_state = JointState()
        joint_state.name = [""] * self.MOTOR_NUM
        joint_state.position = [0] * self.MOTOR_NUM
        joint_state.velocity = [0] * self.MOTOR_NUM
        joint_state.effort = [0] * self.MOTOR_NUM
        for i in range(self.MOTOR_NUM):
            joint_state.name[i] = rosparam.get_param("arm/joint"+str(i+1)+"/name") 
        self._joint_state = joint_state

        # state flags
        self._joint_error_state = True    # nominal=True, error=False
        self._joint_com_state = False    # nominal=True, lost=False
        self._joint_enable_state = False # enabled=True, disabled=False
        self._index_search_finished  = False    # done=True, not yet=False
        self._robot_state = 0          # OK=0, WARN=1, ERROR=2, STALE=3
        self._joint_error_message = ""

        # get param
        self.get_param()

        # set diagnostic updater
        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID("arm")
        self._diagnostic_updater.add("arm_state",self.diagnostics_updater)

        # setup odrive
        self._odrive_bridge = ODriveBridge(MOTOR_NUM=self.MOTOR_NUM,config=rosparam.get_param("arm"))
        rospy.loginfo("Waiting for Odrive...")
        try:
            self._odrive_bridge.connect()
            self._odrive_bridge.set_mode(mode="POS")
            rospy.loginfo("Catchrobo driver is ready")
            rospy.Timer(rospy.Duration(1.0 / self._communication_freq), self.controll_callback)
            rospy.spin()
        except:
            rospy.logerr("Failed on startup")


    def get_param(self):
        self._joint_position_limit_max = []
        self._joint_position_limit_min = []
        self._joint_position_offset = []
        self._joint_position_tolerence = []
        self._joint_velocity_limit = []
        self._joint_currernt_limit = []
        for i in range(5):
            self._joint_position_limit_max.append(rosparam.get_param("arm/joint"+str(i+1)+"/limit/position/max"))
            self._joint_position_limit_min.append(rosparam.get_param("arm/joint"+str(i+1)+"/limit/position/min"))
            self._joint_velocity_limit.append(rosparam.get_param("arm/joint"+str(i+1)+"/limit/velocity"))
            self._joint_currernt_limit.append(rosparam.get_param("arm/joint"+str(i+1)+"/limit/current"))
            self._joint_position_offset.append(rosparam.get_param("arm/joint"+str(i+1)+"/offset"))
            self._joint_position_tolerence.append(rosparam.get_param("arm/joint"+str(i+1)+"/tolerence"))
        self._communication_freq = rosparam.get_param("arm/communincation_frequency") 
        # set kp, kd
        #for i in range(self.MOTOR_NUM):
        #    joint_control.kp[i]  = rosparam.get_param("arm/joint"+str(i+1)+"/kp")
        #    joint_control.kd[i]  = rosparam.get_param("arm/joint"+str(i+1)+"/kd")

    def convert_joint_to_motor(self,position):
        pass

    def convert_motor_to_joint(self,position):
        pass

    def read(self):
        motor_state = self._odrive_bridge.read()
        #self.convert_motor_to_joint(motor_state))
        self._joint_state.position = motor_state.position
        self._joint_state.velocity = motor_state.velocity
        self._joint_state.effort = motor_state.effort
        self._joint_state_publisher.publish(self._joint_state)
        
    def write(self):
        # over write control command if joints are not enabled:
        if self._joint_enable_state is False:
            for i in range(self.MOTOR_NUM):
                self._joint_control.position[i] = self._joint_state.position[i]
        #self.convert_joint_to_motor(_joint_control)
        if self._index_search_finished is True:
            self._odrive_bridge.write(position=self._joint_control.position)
        else:
            pass

    def safety_check(self):
        joint_error = True
        joint_error_message = ""

        # check odrive errors
        #driver_error_string = self._odrive_bridge.get_errors_all(clear=True)
        #if driver_error_string is not "":
        #    joint_error = False
        #    joint_error_message += driver_error_string

        # check joint errors
        for i in range(self.MOTOR_NUM):
            if self._joint_state.position[i] > self._joint_position_limit_max[i] or self._joint_state.position[i] < self._joint_position_limit_min[i]:
                joint_error = False
                joint_error_message += ("Joint "+ str(i+1) + " over position, ")
            if abs(self._joint_state.velocity[i]) > self._joint_velocity_limit[i]:
                joint_error = False
                joint_error_message += ("Joint "+ str(i+1) + " over velocity, ")
            if abs(self._joint_state.effort[i]) > self._joint_currernt_limit[i]:
                joint_error = False
                joint_error_message += ("Joint "+ str(i+1) + " over current, ")

        # check previous state
        if joint_error is False and self._joint_error_state is True:
            self._odrive_bridge.idle_all()
            rospy.logerr(joint_error_message)
        if joint_error is True and self._joint_error_state is False:
            rospy.loginfo("Robot recovered from error state")

        # update state and message
        self._joint_error_state = joint_error
        self._joint_error_message = joint_error_message
        
    def diagnostics_updater(self,state):
        if self._joint_com_state is True:
            if self._joint_error_state is True:
                if self._joint_enable_state is True:
                    self._robot_state = 0
                    state.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Ready")
                else:
                    self._robot_state = 1
                    state.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Idle")
            else:
                self._robot_state = 2
                state.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,self._joint_error_message )
        else:
            self._robot_state = 3
            state.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE, "Not connected")


    def engage_idle_callback(self,data):
        if self._robot_state < 2:
            if data.data is True:
                if self._index_search_finished  is True:
                    rospy.loginfo("Engage joints")
                    self._odrive_bridge.engage_all(index_search=False)
                    self._joint_enable_state = True
                else:
                    rospy.loginfo("Starting index search")
                    self._odrive_bridge.search_index_all()
                    self._index_search_finished  = True
            else:
                rospy.loginfo("Idle joints")
                self._odrive_bridge.idle_all()
                self._joint_enable_state = False
        else:
            rospy.logerr("Robot is not ready. Failed to engage/idle")


    def joint_control_callback(self,data):
        invalid_goal_flag = False

        # check if robot if ready
        if self._robot_state is not 0:
            rospy.logwarn_throttle(1,"Robot is not ready. Received goal was ignored.")
            invalid_goal_flag = True

        else:
            # check if recveived goal is valid
            for i in range(self.MOTOR_NUM):
                if abs(self._joint_state.position[i] - data.position[i]) > self._joint_position_tolerence[i]:
                    rospy.logwarn_throttle(1,"Joint "+ str(i+1) + " target position gap detected.")
                    invalid_goal_flag = True
                if self._joint_position_limit_max[i] <= data.position[i] and data.position[i] <= self._joint_position_limit_min[i]:
                    rospy.logwarn_throttle(1,"Joint "+ str(i+1) + " target position violating joint limit.")
                    invalid_goal_flag = True
        
        # send goal if goal is valid
        if invalid_goal_flag is False:
            for i in range(self.MOTOR_NUM):
                self._joint_control.position[i] = data.position[i]
                self._joint_control.velocity[i] = data.velocity[i]
                self._joint_control.effort[i] = data.effort[i]
        else:
            pass
            

    def controll_callback(self,event):
        try:
            self.read()
            self.safety_check()
            self.write()
            self._joint_com_state = True
        except:
            self._odrive_bridge.idle_all()
            self._joint_com_state = False
            rospy.logerr_throttle(1,"ERROR DETECTED")
        self._diagnostic_updater.update()

if __name__ == "__main__":
    try:
        catchrobo_driver()
    except rospy.ROSInterruptException: 
        pass
    