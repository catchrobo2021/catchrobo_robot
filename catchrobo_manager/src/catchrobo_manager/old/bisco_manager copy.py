#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np

import rospy
import rospkg
import moveit_commander

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import PointField

import copy

class BiscoDatabase:
    def __init__(self):
        self._count_key = "exist"

    def readCsv(self, color):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_manager")
        config_path = pkg_path + "/config/"
        bisco_csv = config_path + color + "_bisco.csv"
        self._objects = pd.read_csv(bisco_csv, index_col=0)
    
    def updateState(self, id, key, value):
        self._objects.loc[id, key] = value
    
    def getState(self, id, key):
        return self._objects.loc[id, key]

    def getPosi(self, id):
        posi = Point()
        posi.x = self._objects.loc[id, "x"]
        posi.y = self._objects.loc[id, "y"]
        posi.z = self._objects.loc[id, "z"]
        return posi

    def isExist(self, id):
        return self._objects.loc[id, "exist"]

    def getObj(self, id):
        if id is None:
            return None
        else:
            return self._objects.loc[id]
    
    def delete(self, id):
        self._objects.loc[id, self._count_key] = False
    
    def getNum(self):
        return len(self._objects)

class RvizBisco:
    def __init__(self):
        
        self._name = "bisco"

        self.BISCO_SIZE = 0.086, 0.029, 0.136
        self._LINK_NAME = "arm/link_tip"

        self._scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        # self.cleanRviz()

    def cleanRviz(self):
        bisco_num = len(self._objects)
        for i in range(bisco_num):
            self.release(i)
            
    def getName(self, id):
        return "{}{}".format(self._name, id)
    
    def attach(self, bisco_id):
        # [TODO] change for servo
        # touch_links = moveit_commander.RobotCommander().get_link_names("arm0")
        box_name = self.getName(bisco_id)
        self._scene.attach_box(self._LINK_NAME, box_name)

    def release(self, bisco_id):
        box_name = self.getName(bisco_id)
        self._scene.remove_attached_object(self._LINK_NAME, name=box_name)
        rospy.sleep(0.01)
        self._scene.remove_world_object(box_name)
        rospy.sleep(0.01)
    
    def addBox2Scene(self, database):

        rospy.wait_for_service("/get_planning_scene", timeout=10.0)
        rospy.wait_for_service("/apply_planning_scene", timeout=10.0)
        # p = PoseStamped()
        # p.header.frame_id = "world"
        # p.pose.position.z = _pub2gui0.8
        # p.pose.orientation.w = 1.0
        # size = 4, 4, 0.001
        # # rospy.sleep(0.1)
        # self._scene.add_box("cell", p, size)

        BISCO_SIZE = self.BISCO_SIZE
        bisco_num = database.getNum()
        # rospy.sleep(2)
        for i in range(bisco_num):
            if not database.isExist(i):
                continue
            p = PoseStamped()
            p.header.frame_id = "world"
            p.pose.position = database.getPosi(i)
            p.pose.position.z += BISCO_SIZE[2] / 2 + 0.0005
            p.pose.orientation.w = 1.0
            size = BISCO_SIZE[0], BISCO_SIZE[1], BISCO_SIZE[2] - 0.001
            # rospy.sleep(0.1)
            self._scene.add_box(self.getName(i), p, size)
        
class BiscoGUI():
    def __init__(self, *database):
        self._database = database
        self._pub2gui = rospy.Publisher("ob", Int32MultiArray, queue_size=1)
        rospy.Subscriber("obj", Int32MultiArray, self.guiCallback)

    def sendGUI(self):
        info = Int32MultiArray()
        bisco_num = self._databasee.getNum()
        info.data = [0] * bisco_num 
        for i in range(bisco_num):
            info.data[i] = self._database.isExist(i)
        self._pub2gui.publish(info)

    def guiCallback(self,msg):
        for i, val in enumerate(msg.data):
            self._database.updateState(i,"exist", val)

class TargetBiscoCalculator:
    def getMininumPriorityId(self, database):
        exist = database._objects["exist"]
        # print(self._objects[exist]["priority"])
        if np.sum(exist) == 0:
            minimum = None
        else:
            minimum = database._objects[exist]["priority"].idxmin()
        return minimum
            

    def calcTargetTwin(self, database):
        biscos = copy.deepcopy(database)
        first = self.getMininumPriorityId(biscos)
        if first is None:
            target_ids = None, None
            twin = False
            return target_ids, twin

        biscos.delete(first)
        second = self.getMininumPriorityId(biscos)
        twin = self.isNeighbor(biscos, first, second)
        return [first, second], twin
    

    
    def isNeighbor(self, biscos, first, second):
        ret = False
        if first is None or second is None:
            return False
        areas = [biscos.getState(first, "my_area"), biscos.getState(second, "my_area")]
        if areas[0] == areas[1] == True:
            if abs(first - second) == 6:
                ret = True
        elif areas[0] == areas[1] == False:
            if abs(first - second) == 1:
                ret = True
        return ret
        

class BiscoManager():
    def __init__(self, color):
        self._database = BiscoDatabase()
        self._database.readCsv(color)

        self._calculator = TargetBiscoCalculator()
        self._rviz = RvizBisco()
        self._gui = BiscoGUI(*self._database)

        self._rviz.addBox2Scene(self._database)
        self._gui.sendGUI()

    def pick(self, id):
        self._rviz.attach(id)
        self._database.delete(id)
        self._gui.sendGUI()

    def release(self, id):
        self._rviz.release(id)

    def calcTargetTwin(self):
        self._target_ids, self._twin =  self._calculator.calcTargetTwin(self._database)

    def getTargetTwin(self):
        return [self._database.getObj(id) for id in self._target_ids], self._twin 
    