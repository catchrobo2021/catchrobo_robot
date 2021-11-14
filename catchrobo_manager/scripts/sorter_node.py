#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from catchrobo_manager.sorter import SorterServer

if __name__ == "__main__":
    rospy.init_node("sorter")
    color = rospy.get_param("color")
    sorter = SorterServer(color)
    rospy.spin()
