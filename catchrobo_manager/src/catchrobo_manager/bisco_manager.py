#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np

import rospy
import rospkg
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

# out  (0.39683606136380156, 0.15715049815585772, 2.4719723905968984, 0.5124697648376378, 1.9676323881188857, 0.0, 0.0, 0.0, 0.0)
# safe (0.3435365473082434, 0.22986438234656048, 2.3714388879977952, 0.5402893832376002, 1.9143328743785886, 0.0, 0.0, 0.0, 0.0)

class BiscoManager():
    def __init__(self, color):

        self._count_key = "exist"
        self._twin = False
        self._name = "bisco"

        self.BISCO_SIZE = 0.086, 0.029, 0.136

        self._LINK_NAME = "arm/link_tip"
        self._scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        self.readCsv(color)
        self.addBox2Scene()

    def readCsv(self, color):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_manager")
        config_path = pkg_path + "/config/"
        bisco_csv = config_path + color + "_bisco.csv"
        self._objects = pd.read_csv(bisco_csv, index_col=0)

    def calcTargetId(self):
        exist = self._objects[self._count_key]
        # print(self._objects[exist]["priority"])
        if np.sum(exist) == 0:
            self._target_id = None
        else:
            self.updatePriority()
            self._target_id = self._objects[exist]["priority"].idxmin()

    # [TODO]
    def updatePriority(self):
        pass

    def getTargetId(self):
        return self._target_id

    def getTagetObj(self):
        return self._objects.iloc[self._target_id]

    def updateState(self, id, value):
        self._objects.loc[id, self._count_key] = value

    def getPosi(self, id):
        posi = Point()
        posi.x = self._objects.loc[id, "x"]
        posi.y = self._objects.loc[id, "y"]
        posi.z = self._objects.loc[id, "z"]
        return posi

    def getName(self, id):
        return "{}{}".format(self._name, id)

    def getTargeName(self):
        return self.getName(self.getTargetId())

    def isExist(self, id):
        return self._objects.loc[id, "exist"]

    def getState(self, id, key):
        return self._objects.loc[id, key]
    
    def getObj(self, id):
        if id is None:
            return None
        else:
            return self._objects.loc[id]
    
    def delete(self, id):
        self._objects.loc[id, self._count_key] = False

    
    def getTargetTwin(self):
        return [self.getObj(id) for id in self._target_ids], self._twin

    def calcTargetTwin(self):
        self.calcTargetId()
        first = self.getTargetId()
        if first is None:
            self._target_ids = None, None
            self._twin = False
            return

        self.updateState(first, False)

        self.calcTargetId()
        second = self.getTargetId()
        self.updateState(first, True)
        self._target_ids = first, second

        self._twin = False
        if first is None:
            return False
        if first is not None and second is not None:
            areas = [self.getState(first, "my_area"), self.getState(second, "my_area")]
            positions = [self.getPosi(first), self.getPosi(second)]
            if areas[0] == areas[1] == True:
                if abs(first - second) == 6:
                    self._twin = True
            elif areas[0] == areas[1] == False:
                if abs(first - second) == 1:
                    self._twin = True
        
        return True

    def attach(self, bisco_id):
        # [TODO] change for servo
        # touch_links = moveit_commander.RobotCommander().get_link_names("arm0")
        box_name = self.getName(bisco_id)
        self._scene.attach_box(self._LINK_NAME, box_name)

    def release(self, bisco_id):
        box_name = self.getName(bisco_id)
        self._scene.remove_attached_object(self._LINK_NAME, name=box_name)
        rospy.sleep(0.1)
        self._scene.remove_world_object(box_name)
        rospy.sleep(0.1)
    
    def addBox2Scene(self):
        rospy.wait_for_service("/get_planning_scene", timeout=10.0)
        rospy.wait_for_service("/apply_planning_scene", timeout=10.0)
        BISCO_SIZE = self.BISCO_SIZE
        bisco_num = len(self._objects)
        # rospy.sleep(2)
        for i in range(bisco_num):
            if not self.isExist(i):
                continue
            p = PoseStamped()
            p.header.frame_id = "world"
            p.pose.position = self.getPosi(i)
            p.pose.position.z += BISCO_SIZE[2] / 2 + 0.0005
            p.pose.orientation.w = 1.0
            size = BISCO_SIZE[0], BISCO_SIZE[1], BISCO_SIZE[2] - 0.001
            # rospy.sleep(0.1)
            self._scene.add_box(self.getName(i), p, size)
        

