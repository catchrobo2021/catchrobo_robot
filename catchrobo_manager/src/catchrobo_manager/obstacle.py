#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


class Obstacle:
    def __init__(self, color):
        rospy.wait_for_service("/get_planning_scene", timeout=10.0)
        rospy.wait_for_service("/apply_planning_scene", timeout=10.0)
        self._scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self._color = color
        self._common_area_middle_obstacle_exist = False
        self.makeCommonAreaObstacle()
        

    def deleteCommonAreaObstacle(self):
        self._scene.remove_world_object("avoid_common_area")
        rospy.sleep(0.1)

    def makeCommonAreaObstacle(self):
        p = PoseStamped()
        p.header.frame_id = "world"  

        # size = 0.15, 0.2, 0.5
        # p.pose.position.x = 0.850
        # p.pose.position.y = 0.100
        # p.pose.position.z = size[2]/2 + 0.05

        # if color == "blue":
        #     p.pose.position.x = -p.pose.position.x 
        # p.pose.orientation.w = 1.0
        # rospy.sleep(0.1)
        # self._scene.add_box("avoid_back", p, size)

        size = 0.01, 1.350, 0.5
        p.pose.position.x = 0.15
        if self._color == "blue":
            p.pose.position.x = -p.pose.position.x 
        p.pose.position.y = size[1] /2
        p.pose.position.z = size[2]/2 + 0.1
        p.pose.orientation.w = 1.0
        rospy.sleep(0.1)
        self._scene.add_box("avoid_common_area", p, size)
    
    def deleteCommonAreaMiddleObstacle(self):
        if self._common_area_middle_obstacle_exist:
            self._common_area_middle_obstacle_exist = False
            self._scene.remove_world_object("avoid_common_area_middle")
            rospy.sleep(0.1)


    def makeCommonAreaMiddleObstacle(self):
        if self._common_area_middle_obstacle_exist:
            return
        self._common_area_middle_obstacle_exist = True
        p = PoseStamped()
        p.header.frame_id = "world"  

        # size = 0.15, 0.2, 0.5
        # p.pose.position.x = 0.850
        # p.pose.position.y = 0.100
        # p.pose.position.z = size[2]/2 + 0.05

        # if color == "blue":
        #     p.pose.position.x = -p.pose.position.x 
        # p.pose.orientation.w = 1.0
        # rospy.sleep(0.1)
        # self._scene.add_box("avoid_back", p, size)

        size = 0.01, 1.350, 0.5
        p.pose.position.x = 0
        if self._color == "blue":
            p.pose.position.x = -p.pose.position.x 
        p.pose.position.y = size[1] /2
        p.pose.position.z = size[2]/2 + 0.1
        p.pose.orientation.w = 1.0
        rospy.sleep(0.1)
        self._scene.add_box("avoid_common_area_middle", p, size)


        