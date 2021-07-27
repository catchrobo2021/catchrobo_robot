#!/usr/bin/env python
# -*- coding: utf-8 -*-

from os import wait
import pandas as pd
import numpy as np

import rospy
import rospkg
import tf
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest
from moveit_msgs.srv import GetPositionIK


from catchrobo_manager.object_database import ObjectDatabase


class ActionState(object):
    BISCO = 1
    SHOOT = 2
    FINISH =3

class GameManager(object):
    def __init__(self):
        self.BISCO_NUM = 27
        self._color = rospy.get_param("/color")


        self._can_go_common = False
        #[TODO] load from the retry point
        self.readCsvs()

        rospy.init_node("moveit_command_sender")
        self._robot = moveit_commander.RobotCommander()
        rospy.loginfo(self._robot.get_group_names())
        self._arm = moveit_commander.MoveGroupCommander("arm0")

        rospy.wait_for_service('compute_ik', timeout=10.0)  # Wait for 10 seconds and assumes we don't want IK

        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    def readCsvs(self):
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        pkg_path = rospack.get_path('catchrobo_manager')
        
        config_path = pkg_path + "/config/"


        bisco_csv = config_path + self._color + "_bisco.csv"
        self._biscos =  ObjectDatabase(bisco_csv, "exist")

        shoot_csv = config_path +self._color+ "_shoot.csv"
        self._shoots =  ObjectDatabase(shoot_csv, "open")
    
    # def checkGoCommon(self):
    #     if not self._can_go_common:
    #         self._can_go_common = self._box[self._box["open"]]["priority"].max() <= - 3  #0,1,2が埋まればcommonに入ってよい
    #     return self._can_go_common

    #[TODO]
    def mannualAction(self):

        rospy.loginfo(self._arm.get_current_pose())
        pass

    #[TODO]
    def hand(self, close):
        pass

    def goAboveObj(self, obj, above_val, mode):
        target_pose = Pose()
        target_pose.position = obj.getPosi(obj.getTargetId())
        # target_pose.position.x = 0.367048603838
        # target_pose.position.y -= 0.05
        target_pose.position.z += above_val
        
        # target_pose.position =Point(0.307048603838,0.665251814232, 0.238604612133)
        quat = tf.transformations.quaternion_from_euler(np.pi, 0,-np.pi/2)
        target_pose.orientation = Quaternion(*quat)
        # target_pose.orientation = Quaternion( -0.247424555486, -0.47564766276,  -0.0118072900308, 0.84403849329)
        rospy.loginfo(target_pose)
        ret = self.go(target_pose, mode)
        # self._arm.stop()
        
        return ret
    

    def go(self, goal, mode):
        if mode == "free":
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id ="/world" # Hard coded for now
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = goal 
            
            # Create a moveit ik request
            ik_request = PositionIKRequest() 
            ik_request.group_name = 'arm0' # Hard coded for now
            ik_request.pose_stamped = pose_stamped
            ik_request.timeout.secs = 0.1
            ik_request.avoid_collisions = True 
            
            request_value = self.compute_ik(ik_request)

            rospy.loginfo(request_value)
            rospy.sleep(1)

            
            # goal = 
            plan = RobotTrajectory()
            plan.joint_trajectory.header.frame_id = "/world"
            plan.joint_trajectory.joint_names =  ["arm/joint1", "arm/joint2","arm/joint3", "arm/joint4", "arm/joint5"]
            
            point = JointTrajectoryPoint()
            point.positions = request_value.solution.joint_state.position

            current_point = JointTrajectoryPoint()
            current_point.positions = self._arm.get_current_joint_values()
            plan.joint_trajectory.points = [ current_point, point]

            ret = self._arm.execute(plan, wait=True)
        
        elif mode == "safe":
            # now =  rospy.Time.now().to_sec()
            # pose_stamped = PoseStamped()
            # pose_stamped.header.frame_id ="/world" # Hard coded for now
            # pose_stamped.header.stamp = rospy.Time.now()
            # pose_stamped.pose = goal 
            
            # # Create a moveit ik request
            # ik_request = PositionIKRequest() 
            # ik_request.group_name = 'arm0' # Hard coded for now
            # ik_request.pose_stamped = pose_stamped
            # ik_request.timeout.secs = 0.1
            # ik_request.avoid_collisions = False 
            
            
            # request_value = self.compute_ik(ik_request)
            # # rospy.loginfo(request_value)

            # joints = request_value.solution.joint_state.position
            # rospy.loginfo(joints)
            # rospy.loginfo(request_value.error_code)
            # self._arm.set_joint_value_target(joints[0:5])
            # dt = rospy.Time.now().to_sec() - now
            # temp  = "IK time : {}".format(dt)
            # rospy.loginfo(temp)

            self._arm.set_pose_target(goal)
            now =  rospy.Time.now().to_sec()
            plan = self._arm.plan()

            dt = rospy.Time.now().to_sec() - now
            temp  = "plan time : {}".format(dt)
            rospy.loginfo(temp)
            
            now =  rospy.Time.now().to_sec()
            ret = self._arm.execute(plan)

            dt = rospy.Time.now().to_sec() - now
            temp  = "execute time : {}".format(dt)
            rospy.loginfo(temp)
            

        elif mode == "cartesian":
            waypoints = [goal]
            (plan, fraction) = self._arm.compute_cartesian_path(waypoints, 100, 0)
            ret = self._arm.execute(plan, wait=True)
            


        return ret



    def vertical_move(self, relative_m):
        now = self._arm.get_current_pose()
        target_pose = now.pose
        target_pose.position.z += relative_m
        self._arm.set_pose_target(target_pose)
        ret = self._arm.go()

    def biscoAction(self):
        self._biscos.calcTargetId()
        target_id = self._biscos.getTargetId() 
        print("target bisco: ", target_id)
        if target_id  is None:
            self.mannualAction()
            self._next_action = ActionState.FINISH
        else:
            if target_id <=20 :
                self._next_action = ActionState.FINISH
                return
            self._biscos.updateState(target_id, False)
            if not self._biscos.getTagetObj()["my_area"]:
                return

            if self.goAboveObj(self._biscos, 0.136 + 0.06, "safe"):
                self.hand(close=False)
                self.goAboveObj(self._biscos, 0.136 + 0.04, "safe")
                self.hand(close=True)
                self.goAboveObj(self._biscos, 0.136 *2+ 0.05, "safe")
                self._next_action = ActionState.SHOOT
            else:
                rospy.logerr("cannot make path")
    

    def shootAction(self):
        self._shoots.calcTargetId()
        target_id = self._shoots.getTargetId() 
        print("target shoot: ", target_id)
        if target_id  is None:
            self.mannualAction()
            self._next_action = ActionState.FINISH
        else:
            self._shoots.updateState(target_id, False)
            if self.goAboveObj(self._shoots, 0.136 *2+ 0.05, "safe"):
                self.goAboveObj(self._shoots,0.136 + 0.04, "safe")
                self.hand(close=False)
                self.goAboveObj(self._shoots,0.136 + 0.06, "safe")
                self._next_action = ActionState.BISCO
            else:
                rospy.logerr("cannot make path")
        

    def main(self):
        # self._arm.set_max_velocity_scaling_factor(1)

        self._next_action = ActionState.BISCO
        # rate = rospy.Rate(10)
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self._next_action == ActionState.BISCO:
                self.biscoAction()
                
            elif self._next_action == ActionState.SHOOT:
                self.shootAction()
            elif self._next_action == ActionState.FINISH:
                break
        
        end_time = rospy.Time.now().to_sec()
        print("time: ", end_time - start_time)
            # rate.sleep()
        
        
if __name__ == "__main__":
    game_manager = GameManager()
    game_manager.main()
    rospy.spin()



        



