#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_manager.bisco.bisco_rviz import BiscoRviz

if __name__ == "__main__":
    rospy.init_node("test_bisco")
    BiscoRviz().cleanRviz()
