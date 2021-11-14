#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_manager.shooting_box_manager import ShootingBoxManager

if __name__ == "__main__":
    rospy.init_node("test_shooter")
    manager = ShootingBoxManager("blue")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        manager.calcTargetTwin()
        targets, _ = manager.getTargetTwin()

        if targets[0] is not None and targets[1] is not None:
            temp = "targets : {}, {} (twin: {})".format(
                targets[0].name, targets[1].name, _)
        elif targets[0] is not None and targets[1] is None:
            temp = "targets : {}, {} (twin: {})".format(
                targets[0].name, None, _)
        else:
            temp = "targets : {}, {} (twin: {})".format(None, None, _)

        rospy.loginfo(temp)
        # temp = manager.isCommonExist()
        # rospy.loginfo(temp)
        manager.sendGUI()
        rate.sleep()
