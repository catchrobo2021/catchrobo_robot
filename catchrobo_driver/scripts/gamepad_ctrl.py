#!/usr/bin/env python
import rospy
from rospy.core import rospyinfo
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class gamepad_ctrl:
    def __init__(self):
        rospy.init_node('gamepad_ctrl')
        rospy.Subscriber("joy",Joy,self.joyCallback)
        self._enable_joints_publisher = rospy.Publisher('enable_joints', Bool, queue_size=10)

        self._manual_flag_pub = rospy.Publisher('pause', Bool, queue_size=1)
        rospy.spin()

    def joyCallback(self, data):
        if data.buttons[1] is 1:
            self._enable_joints_publisher.publish(True)
        if data.buttons[0] is 1:
            self._enable_joints_publisher.publish(False)
        if data.buttons[2] == 1:
            self._manual_flag_pub.publish(True)
        if data.buttons[3] == 1:
            self._manual_flag_pub.publish(False)
 
if __name__ == "__main__":
    gamepad_ctrl()