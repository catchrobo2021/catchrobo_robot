#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_manager.bisco.bisco_manager import BiscoManager

if __name__ == "__main__":
    rospy.init_node("test_bisco")
    manager = BiscoManager("red", False)
    rate = rospy.Rate(1)    
    while not rospy.is_shutdown():
        manager.calcTargetTwin()
        targets, twin = manager.getTargetTwin()
        temp = "targets : {}, {} (twin: {})".format(targets[0].name, targets[1].name, twin)
        temp = manager.isCommonExist()
        rospy.loginfo(temp)
        manager._gui.sendGUI()
        rate.sleep()
