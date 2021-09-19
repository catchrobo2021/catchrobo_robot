#!/usr/bin/env python3
import odrive
import time
import rospy
from odrive.enums import *
from sensor_msgs.msg import JointState

class OdriveDemo:
    def __init__(self):
        rospy.init_node("odrive_demo_node")
        self._joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=100)
        self.JOINT_NUM = 1
        
        # joint state feedback
        joint_state = JointState()
        joint_state.name = [""] * self.JOINT_NUM
        joint_state.position = [0] * self.JOINT_NUM
        joint_state.velocity = [0] * self.JOINT_NUM
        joint_state.effort = [0] * self.JOINT_NUM
        for i in range(self.JOINT_NUM):
            joint_state.name[i] = "joint"
        self._joint_state = joint_state



        rospy.loginfo("finding an odrive...")
        self.my_drive = odrive.find_any()

        communication_freq = 100
        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controlCallback)
        rospy.spin()

        rospy.loginfo("index search...")
        self.my_drive.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        while self.my_drive.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        rospy.loginfo("index search done")

        # Calibrate motor and wait for it to finish
        #print("starting calibration...")
        #self.my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        #while self.my_drive.axis0.current_state != AXIS_STATE_IDLE:
        #    time.sleep(0.1)

        # To read a value, simply read the property
        rospy.loginfo("Bus voltage: " + str(self.my_drive.vbus_voltage) + "V")
        rospy.loginfo("Serial number: " + str(self.my_drive.serial_number))
        rospy.loginfo("Hardware version: " + str(self.my_drive.hw_version_major)+ "." + str(self.my_drive.hw_version_minor)+ "." + str(self.my_drive.hw_version_variant))
        rospy.loginfo("Firmware version: " + str(self.my_drive.fw_version_major)+ "." + str(self.my_drive.fw_version_minor)+  "." +str(self.my_drive.fw_version_revision))

        # Or to change a value, just assign to the property
        #self.my_drive.axis0.controller.pos_setpoint = 3.14
        #print("Position setpoint is " + str(self.my_drive.axis0.controller.pos_setpoint))

        # And this is how function calls are done:
        #for i in [1,2,3,4]:
        #    print('voltage on GPIO{} is {} Volt'.format(i, self.my_drive.get_adc_voltage(i)))

        # A sine wave to test

        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        time.sleep(0.1)
        self.my_drive.axis0.controller.input_pos = 3.0
        time.sleep(1)
 

    def controlCallback(self, event):
        
        self._joint_state.position[0] = self.my_drive.axis0.encoder.pos_estimate
        self._joint_state.velocity[0] = self.my_drive.axis0.encoder.vel_estimate
        self._joint_state.effort[0] = self.my_drive.axis0.motor.current_control.Iq_measured
        #self.addOffset(self._joint_state.position)
        self._joint_state_publisher.publish(self._joint_state)


if __name__ == "__main__":
    OdriveDemo()



