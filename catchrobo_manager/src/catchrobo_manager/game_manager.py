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
        self.BISCO_SIZE = 0.086,0.029,0.136
        self.BISCO_NUM = 27
        self._color = rospy.get_param("/color")


        self._can_go_common = False
        #[TODO] load from the retry point
        self.readCsvs()

        rospy.init_node("moveit_command_sender")
        self._robot = moveit_commander.RobotCommander()
        rospy.loginfo(self._robot.get_group_names())
        self._arm = moveit_commander.MoveGroupCommander("arm0")
        self._gripper = moveit_commander.MoveGroupCommander("hand1")
        # rospy.loginfo(self._arm.get_current_pose())
        # self._arm.set_end_effector_link("gripper/grasping_frame1")
        # rospy.loginfo(self._arm.get_current_pose())
        # rospy.loginfo(self._robot.get_link_names("hand1"))

        # arm_link = self._robot.get_link_names("arm0")

        # hand1_attached_link_names = arm_link + self._robot.get_link_names("hand1")
        # rospy.loginfo(hand1_attached_link_names)
        # rospy.sleep(10)
        # rospy.loginfo(self.variable.get_current_joint_values())
        

        rospy.wait_for_service('compute_ik', timeout=10.0)  # Wait for 10 seconds and assumes we don't want IK

        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

        self._pose_stamped = PoseStamped()
        self._pose_stamped.header.frame_id ="world" # Hard coded for now
        
        # Create a moveit ik request
        self._ik_request = PositionIKRequest() 
        self._ik_request.group_name = 'arm0' # Hard coded for now
        # self._ik_request.ik_link_names = hand1_attached_link_names
        self._ik_request.timeout.secs = 0.1
        self._ik_request.avoid_collisions = True 
        self._ik_request.attempts= 100

        self._pick_quat = tf.transformations.quaternion_from_euler(np.pi, 0,0)

        self.AddBisco2Scene()

    
    def AddBisco2Scene(self):
        self._scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)

        for i in range(self.BISCO_NUM):
            if not self._biscos.isExist(i):
                continue
            p = PoseStamped()
            p.header.frame_id = self._robot.get_planning_frame()
            
            p.pose.position = self._biscos.getPosi(i)
            p.pose.position.z += self.BISCO_SIZE[2]/2 + 0.0005
            p.pose.orientation.w = 1.0
            size = self.BISCO_SIZE[0], self.BISCO_SIZE[1], self.BISCO_SIZE[2] - 0.001
            self._scene.add_box("bisco{}".format(i), p,size)


        
        

    def readCsvs(self):
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        pkg_path = rospack.get_path('catchrobo_manager')
        
        config_path = pkg_path + "/config/"


        bisco_csv = config_path + self._color + "_bisco.csv"
        self._biscos =  ObjectDatabase("bisco", bisco_csv, "exist")

        shoot_csv = config_path +self._color+ "_shoot.csv"
        self._shoots =  ObjectDatabase("shoot", shoot_csv, "open")
    
    def checkGoCommon(self):
        if not self._can_go_common:
            self._can_go_common = self._box[self._box["open"]]["priority"].max() <= - 3  #0,1,2が埋まればcommonに入ってよい
        return self._can_go_common

    #[TODO]
    def mannualAction(self):

        rospy.loginfo(self._arm.get_current_pose())
        pass

    def gripperMove(self, dist, wait):
        self._gripper.set_joint_value_target([dist, dist])
        self._gripper.go(wait)

    def goAboveObj(self, obj, above_val, mode):
        target_pose = Pose()
        target_pose.position = obj.getPosi(obj.getTargetId())
        # target_pose.position.x = 0.367048603838
        # target_pose.position.y -= 0.05
        target_pose.position.z += above_val
        
        target_pose.orientation = Quaternion(*self._pick_quat)
        rospy.loginfo(target_pose.position)
        ret = self.go(target_pose, mode)
        # self._arm.stop()
        
        return ret
    

    def go(self, goal, mode):

        now =  rospy.Time.now()
        self._pose_stamped.header.stamp = now
        self._pose_stamped.pose = goal 
        self._ik_request.pose_stamped = self._pose_stamped
        self._ik_request.robot_state = self._robot.get_current_state()
        
        request_value = self.compute_ik(self._ik_request)
        if request_value.error_code.val <0:
            return  False
        joints = request_value.solution.joint_state.position
        # rospy.loginfo(joints)
        
        self._arm.set_joint_value_target(joints[0:5])
        dt = rospy.Time.now().to_sec() - now.to_sec()
        rospy.loginfo("IK time : {}".format(dt))

        if mode == "safe":
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

    def goAboveBisco(self):
        ret =  self.goAboveObj(self._biscos, 0.136 + 0.04, self._go_mode)
        ret =  self.goAboveObj(self._biscos, 0.136 + 0.02, self._go_mode)
        return ret

    def goAboveShoot(self):
        ret =  self.goAboveObj(self._shoots, 0.136 + 0.04, self._go_mode)
        ret =  self.goAboveObj(self._shoots, 0.136 + 0.02, self._go_mode)
        return ret

    def graspBisco(self):
        #[TODO] change for servo
        touch_links = self._robot.get_link_names("hand")
        box_name = self._biscos.getTargeName()
        self._scene.attach_box(self._arm.get_end_effector_link(), box_name,touch_links=touch_links)
        self.gripperMove(0.005, True)
        ret =  self.goAboveObj(self._biscos, 0.136 + 0.04, self._go_mode)
        return ret


    def releaseBisco(self):
        box_name = self._biscos.getTargeName()
        self._scene.remove_attached_object(self._arm.get_end_effector_link(), name=box_name)
        self.gripperMove(0, True)
        rospy.sleep(0.1)
        self._scene.remove_world_object(box_name) ### it need some minitues from remove_attached_object
        

    def biscoAction(self):
        self._biscos.calcTargetId()
        target_id = self._biscos.getTargetId() 
        print("target bisco: ", target_id)
        if target_id  is None:
            self.mannualAction()
            self._next_action = ActionState.FINISH
        else:
            # if target_id <=20 :
            #     self._next_action = ActionState.FINISH
            #     return
            self._biscos.updateState(target_id, False)
            # if not self._biscos.getTagetObj()["my_area"]:
            #     return
            
            # self.gripperMove(0.0, False) # open
            if self.goAboveBisco():
                self.graspBisco()
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
            if self.goAboveShoot():
                self.releaseBisco()
                # self.goAboveObj(self._shoots,0.136 + 0.04, self._go_mode)
                self._next_action = ActionState.BISCO
            else:
                rospy.logerr("cannot make path")
        

    def main(self):
        # self._arm.set_max_velocity_scaling_factor(1)

        self._next_action = ActionState.BISCO
        self._go_mode = "safe"
        # rate = rospy.Rate(10)
        start_time = rospy.Time.now().to_sec()
        self.gripperMove(0, True)
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



        



