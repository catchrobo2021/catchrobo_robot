#!/usr/bin/env python
from pickle import FALSE
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class JoyButtonEnum:
    CIRCLE = 1
    CROSS = 0
    TRIANGLE = 2
    SQUARE = 3
    R2 = 7


class gamepad_ctrl:
    def __init__(self):
        rospy.init_node('gamepad_ctrl')
        rospy.Subscriber("joy",Joy,self.joyCallback)
        self._enable_joints_publisher = rospy.Publisher('enable_joints', Bool, queue_size=10)
        self._guide_enable_publisher =  rospy.Publisher("guide_enable", Bool, queue_size=1)
        self._manual_flag_pub = rospy.Publisher('on_manual', Bool, queue_size=1)


        rospy.spin()

    def joyCallback(self, data):
        if data.buttons[JoyButtonEnum.CIRCLE] is 1:
            self._enable_joints_publisher.publish(True)
        if data.buttons[JoyButtonEnum.CROSS] is 1:
            self._enable_joints_publisher.publish(False)
            self._guide_enable_publisher.publish(False)
        if data.buttons[JoyButtonEnum.TRIANGLE] == 1:
            self._manual_flag_pub.publish(True)
        if data.buttons[JoyButtonEnum.SQUARE] == 1:
            self._manual_flag_pub.publish(False)
        if data.buttons[JoyButtonEnum.R2] == 1:
            self._enable_joints_publisher.publish(True)
            self._guide_enable_publisher.publish(True)


 
if __name__ == "__main__":
    gamepad_ctrl()