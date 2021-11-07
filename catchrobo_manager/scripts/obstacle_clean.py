#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_manager.bisco.bisco_rviz import BiscoRviz
from catchrobo_manager.obstacle import Obstacle

if __name__ == "__main__":
    rospy.init_node("test_bisco")
    color = rospy.get_param("color")
    BiscoRviz().cleanRviz()
    obstacle = Obstacle(color)
    obstacle.makeCommonAreaMiddleObstacle()
    obstacle.deleteCommonAreaMiddleObstacle()
    obstacle.deleteCommonAreaObstacle()
    
