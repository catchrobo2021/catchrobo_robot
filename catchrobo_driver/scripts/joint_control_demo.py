#!/usr/bin/env python
import rospy
from catchrobo_msgs.msg import CatchroboJointControl
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import math
import time
import rosparam

class test:
    def __init__(self):
        self.JOINT_NUM = 1 # number of joints
        rospy.init_node("joint_control_test")
        self._servo_on_publisher = rospy.Publisher('set_servo_on', Bool, queue_size=10)
        self._joint_control_publisher = rospy.Publisher('set_joint_goal', CatchroboJointControl, queue_size=10)
        rospy.Subscriber("joint_states", JointState, self.jointStateCallback)

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
        
        # params
        self._has_arrived = [True] * self.JOINT_NUM
        self._acceleration = [0.1] * self.JOINT_NUM    # avoid zero division
        self._velocity_max = [0.1] * self.JOINT_NUM    # avoid zero division
        self._position = [0] * self.JOINT_NUM
        self._position_old = [0] * self.JOINT_NUM
        self._initial_position = [0] * self.JOINT_NUM
        self._initial_time = [0] * self.JOINT_NUM
        self._count = 0
        rospy.Timer(rospy.Duration(0.01), self.controlCallback)
        rospy.spin()
    

    def jointStateCallback(self,data):
        self._joint_state = data


    def setJointGoal(self):
        for i in range(self.JOINT_NUM):
            # initialize params when target position is updated
            if self._position[i] is not self._position_old[i]:
                self._position_old[i] = self._position[i]
                self._has_arrived[i] = False
                self._initial_position[i] = self._joint_state.position[i]
                self._initial_time[i] = time.time()
                self._velocity_max[i] = rosparam.get_param("joint"+str(i+1)+"/velocity/default")
                self._acceleration[i] = rosparam.get_param("joint"+str(i+1)+"/acceleration/default")

            # calculte position and velocity when joint has not reached target position yet
            if self._has_arrived[i] is False:
                # local params
                position_diff = self._position[i] - self._initial_position[i]
                time_from_start = time.time() - self._initial_time[i]
                
                # check if target position is negative or positive
                if position_diff < 0:
                    self._acceleration[i] = abs(self._acceleration[i]) * -1.0
                    self._velocity_max[i] = abs(self._velocity_max[i]) * -1.0

                # Check if the maximum velocity can be reached or not
                if pow(self._velocity_max[i],2) > self._acceleration[i] * position_diff:
                    time_mid = math.sqrt(abs(position_diff / self._acceleration[i]))
                    time_end = time_mid * 2
                else :
                    time_mid = self._velocity_max[i] / self._acceleration[i]
                    time_end = time_mid + position_diff / self._velocity_max[i]

                # Calculate velocity and posiion
                if (0 <= time_from_start) & (time_from_start <= time_mid):    # self._acceleration phase
                    self._joint_control.velocity[i] = self._acceleration[i] * time_from_start
                    self._joint_control.position[i] = 1 / 2.0 * self._acceleration[i] * pow(time_from_start , 2)  + self._initial_position[i]
                elif (time_mid < time_from_start) & (time_from_start <= time_end - time_mid): # Constant Velocity phase
                    self._joint_control.velocity[i] = self._velocity_max[i]
                    self._joint_control.position[i] = self._velocity_max[i] * (time_from_start - time_mid / 2)  + self._initial_position[i]
                elif (time_end - time_mid < time_from_start) & (time_from_start <= time_end): # Deacceleration phase
                    self._joint_control.velocity[i] = self._acceleration[i] * (time_end - time_from_start)
                    self._joint_control.position[i] = position_diff - self._acceleration[i] / 2.0 * pow((time_end - time_from_start),2)  + self._initial_position[i]
                else:   # Reached target
                    self._has_arrived[i] = True
                    #rospy.loginfo("Joint "+ str(i+1) + " has reached target position")
                    #rospy.loginfo("Target: "+ str(self._position[i]) + "  Actual: "+ str(self._joint_state.position[i]) )
        self._joint_control_publisher.publish(self._joint_control)

    #def calculateTorque(self):
    #    for i in range(self.JOINT_NUM):
    #       mass = 0.35 # kg
    #       length = 0.3#0.301 # kg
    #       gravity = 9.80665
    #       self._joint_control.effort[i] = mass * gravity * length * math.sin(self._joint_state.position[i])

    def controlCallback(self, event):
        if self._count is 0:
            
            if self._has_arrived[0] is True:
                self._position[0] = 90.0 / 180.0 * 3.14
                self._count += 1
        else:
            if self._has_arrived[0] is True:
                self._position[0] = self._position[0] * -1.0
        self.setJointGoal()
        

if __name__ == "__main__":
    test()