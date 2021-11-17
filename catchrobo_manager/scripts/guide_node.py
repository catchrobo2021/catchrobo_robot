#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from catchrobo_manager.guide import GuideServer

if __name__ == "__main__":
    rospy.init_node("guide", anonymous=True)
    sorter = GuideServer()
    rospy.spin()
