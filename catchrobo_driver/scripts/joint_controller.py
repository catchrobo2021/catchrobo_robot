#!/usr/bin/env python
from logging import error
import rospy
import sys
sys.path.append('/home/yutakage/catkin_ws/src/catchrobo_robot/catchrobo_driver/src')
from myhardware_bridge import MyHardwareBridge
from catchrobo_msgs.msg import CatchroboJointControl
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool,Int16
import rosparam
import roslib
roslib.load_manifest('diagnostic_updater')
import diagnostic_updater
import diagnostic_msgs

class joint_controller:
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
        self._robot_status = 0          # OK=0, WARN=1, ERROR=2, STALE=3
        self._joint_error_message = ""

        # get ros params
        self._joint_position_limit_max = []
        self._joint_position_limit_min = []
        self._joint_position_offset = []
        self._joint_velocity_limit = []
        self._joint_currernt_limit = []
        for i in range(self.JOINT_NUM):
            self._joint_position_limit_max.append(rosparam.get_param("arm/joint"+str(i+1)+"/position/limit/max"))
            self._joint_position_limit_min.append(rosparam.get_param("arm/joint"+str(i+1)+"/position/limit/min"))
            self._joint_position_offset.append(rosparam.get_param("arm/joint"+str(i+1)+"/position/offset"))
            self._joint_velocity_limit.append(rosparam.get_param("arm/joint"+str(i+1)+"/velocity/limit"))
            self._joint_currernt_limit.append(rosparam.get_param("arm/joint"+str(i+1)+"/current/limit"))

        # set kp, kd
        for i in range(self.JOINT_NUM):
            joint_control.kp[i]  = rosparam.get_param("arm/joint"+str(i+1)+"/kp")
            joint_control.kd[i]  = rosparam.get_param("arm/joint"+str(i+1)+"/kd")

        # set diagnostic updater
        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID("arm")
        self._diagnostic_updater.add("robot_state",self.diagnosticUpdater)

        # init module
        communication_freq = rosparam.get_param("arm/communincation_frequency") 
        self._hardware = MyHardwareBridge(communication_freq, "can0","can",self.JOINT_NUM)
        self.disableAllJoints()

        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controlCallback)
        rospy.spin()

    def addOffset(self,position):
        for i in range(self.JOINT_NUM):
            position[i] += self._joint_position_offset[i]

    def communicate(self):
        # over write control command if joints are not enabled:
        if self._joint_enable_state is False:
            for i in range(self.JOINT_NUM):
                self._joint_control.position[i] = self._joint_state.position[i]
                self._joint_control.velocity[i] = 0
                self._joint_control.effort[i] = 0
        #self.addOffset(self._joint_control.position)
        self._joint_com_state = self._hardware.communicate(self._joint_control) # send data to joints
        joint = self._hardware.get_data()            # receive data from joints
        # publish joint states to topic
        self._joint_state.position = joint[:self.JOINT_NUM]
        self._joint_state.velocity = joint[self.JOINT_NUM:self.JOINT_NUM*2]
        self._joint_state.effort = joint[self.JOINT_NUM*2:self.JOINT_NUM*3]
        #self.addOffset(self._joint_state.position)
        self._joint_state_publisher.publish(self._joint_state)
    

    def disableAllJoints(self):
        self._hardware.disable_all_joints()
        self._joint_enable_state = False


    def enableAllJoints(self):
        self._hardware.enable_all_joints()
        self._joint_enable_state = True


    def jointSafetyCheck(self):
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
                    self._robot_status = 0
                    state.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Ready")
                else:
                    self._robot_status = 1
                    state.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Joints disabled")
            else:
                self._robot_status = 2
                state.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,self._joint_error_message )
        else:
            self._robot_status = 3
            state.summary(diagnostic_msgs.msg.DiagnosticStatus.STALE, "Not connected")


    def enableJointsCallback(self,data):
        if self._robot_status < 2:
            if data.data is True:
                self.enableAllJoints()
            else:
                self.disableAllJoints()
        else:
            rospy.logerr("Robot is not connected. Failed to send enable/disable command")


    def jointControlCallback(self,data):
        invalid_goal_flag = False

        # check error state
        if self._robot_status is not 0:
            rospy.logwarn("Robot is not ready")
            invalid_goal_flag = True

        # check if recveived goal is valid
        for i in range(self.JOINT_NUM):
            if abs(self._joint_state.position[i] - data.position[i]) > 0.1:
                rospy.logwarn("Joint "+ str(i+1) + " goal position gap detected")
                invalid_goal_flag = True
            if self._joint_position_limit_max[i] <= data.position[i] and data.position[i] <= self._joint_position_limit_min[i]:
                rospy.logwarn("Joint "+ str(i+1) + " goal violating position limit")
                invalid_goal_flag = True
            if abs(data.velocity[i]) > self._joint_velocity_limit[i]:
                rospy.logwarn("Joint "+ str(i+1) + " goal violating velocity limit")
                invalid_goal_flag = True
            if abs(data.effort[i]) > self._joint_currernt_limit[i]:
                rospy.logwarn("Joint "+ str(i+1) + " goal violating current limit")
                invalid_goal_flag = True
                
        if invalid_goal_flag is False:
            for i in range(self.JOINT_NUM):
                self._joint_control.position[i] = data.position[i]
                self._joint_control.velocity[i] = data.velocity[i]
                self._joint_control.effort[i] = data.effort[i]
        else:
            rospy.logwarn("Invalid goal ignored")
            
    def controlCallback(self, event):
        self.communicate()
        self.jointSafetyCheck()
        self._diagnostic_updater.update()

if __name__ == "__main__":
    joint_controller()