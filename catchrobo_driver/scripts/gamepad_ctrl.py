#!/usr/bin/env python
import rospy
from rospy.core import rospyinfo
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class gamepad_ctrl:
    def __init__(self):
        rospy.init_node('gamepad_ctrl')
        rospy.Subscriber("joy",Joy,self.joyCallback)
        self._servo_on_publisher = rospy.Publisher('set_servo_on', Bool, queue_size=10)
        rospy.spin()

    def joyCallback(self, data):
        if data.buttons[1] is 1:
            self._servo_on_publisher.publish(True)
        if data.buttons[0] is 1:
            self._servo_on_publisher.publish(False)
    
if __name__ == "__main__":
    gamepad_ctrl()