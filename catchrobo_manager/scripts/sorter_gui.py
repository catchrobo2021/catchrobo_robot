#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int8
from catchrobo_manager.sorter import SorterClient

class SorterFurifuri:
    def __init__(self, color):
        self._color = color
        self._sorter = SorterClient()
        rospy.Subscriber("sorter/open_row_gui", Int8, callback=self.open)
        rospy.Subscriber("sorter/close_row_gui", Int8, callback=self.close)

    def open(self, msg):
        if(self._color == "blue"):
            self._sorter.open(msg.data)
        else:
            self._sorter.open(5-msg.data)
    
    def close(self, msg):
        if(self._color == "blue"):
            self._sorter.close(msg.data)
        else:
            self._sorter.close(5-msg.data)
        
if __name__ == '__main__':    
    rospy.init_node("sorter_", anonymous=True)
    color = rospy.get_param("color")
    sorter = SorterFurifuri(color)
    rospy.spin()


