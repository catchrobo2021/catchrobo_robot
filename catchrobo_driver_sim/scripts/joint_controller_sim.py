#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
# from catchrobo_control.okamoto import Okamoto
import thread

class JointControllerSim(object):
    def __init__(self):
        # motors = [Okamoto() for _ in range(5)]
        rospy.init_node("joint_controller_sim")
        self._clock = 100
        self._joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=1)

        self._lock = thread.allocate_lock()
        self._state = JointState()
        self._state.name = rospy.get_param("joints") #["arm/joint{}".format(i+1) for i in range(5)]
        self._state.header.frame_id ="world"
        self._state.position = [0] * len(self._state.name)#[0, 1.5707, -2.7925, 1.22173, 0]
        # self._state.position =[-1.57, 0.25, -2.63, 0, 0]
        

        rospy.Subscriber("joint_control", JointState, self.jointControlCallback)

    def spin(self):
        rate = rospy.Rate(self._clock)
        while not rospy.is_shutdown():

            with self._lock:
                self._joint_state_publisher.publish(self._state)
            
            now = rospy.Time.now()
            self._state.header.stamp = now
            
            rate.sleep()

    def jointControlCallback(self, msg):
        with self._lock:
            self._state = msg


if __name__ == "__main__":
    joint_controller = JointControllerSim()
    joint_controller.spin()