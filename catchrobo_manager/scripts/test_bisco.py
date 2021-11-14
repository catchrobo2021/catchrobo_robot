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
        if targets[0] is not None and targets[1] is not None:
            temp = "targets : {}, {} (twin: {})".format(targets[0].name, targets[1].name, twin)
        elif targets[0] is not None and targets[1] is None:
            temp = "targets : {}, {} (twin: {})".format(targets[0].name, None, twin)
        else:
            temp = "targets : {}, {} (twin: {})".format(None, None, twin)

        rospy.loginfo(temp)
        manager._gui.sendGUI()
        rate.sleep()
