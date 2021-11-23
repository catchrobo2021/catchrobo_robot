#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from rospy.core import rospyinfo
import tf
import rospy


from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from sensor_msgs.msg import Joy

from catchrobo_manager.arm import Arm
from catchrobo_manager.gripper_manager import GripperManager,GripperID,GripWay

class XBoxButton:
    # Axes
    LX = 0
    LY = 1
    LT = 2
    RX = 3
    RY = 4
    RT = 5
    LEFTRIGHT = 6
    UPDOWN = 7
    # Buttons
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    START = 6
    BACK = 7
    XBOX = 8
    LSTICK = 9
    RSTICK = 10

    K_CON = 1


class MenuEnum:
    START = 1
    PAUSE = 2
    EMERGENCY_STOP = 3


class GamePad():
    def __init__(self,Button):

        self._state = Joy()
        self._state.buttons = [0] * 11
        self._state.axes = [0, 0, 1, 0, 0, 1, 0, 0]
        self.ButtonEnum = Button
        rospy.Subscriber("joy", Joy, self.joyCallback, queue_size=1)
        self._flag_LT = 1
        self._flag_RT = 1

    def joyCallback(self, joy_msg):
        self._state = joy_msg

    def getState(self):
        return self._state


class Manual():
    def __init__(self, Button):
        self._color = rospy.get_param("/color", default="blue")
        self._arm = Arm()
        self._gripper = GripperManager(self._color)
        self._gamepad = GamePad(Button)
        self.ButtonEnum = Button

        self._SCALING = 0.1
        self._pub_menu = rospy.Publisher("menu", Int8, queue_size=1)

    def delta_calc(self):
        _MAX_XBOX_AXES = 1.0
        _THRESHOLD_STICK = _MAX_XBOX_AXES * 0.2
        _THRESHOLD_LRT = -_MAX_XBOX_AXES * 0.8
        delta_x = 0
        delta_y = 0
        delta_z = 0
        delta_theta = 0

        if self._state.axes[self.ButtonEnum.LX] < -_THRESHOLD_STICK or _THRESHOLD_STICK < self._state.axes[self.ButtonEnum.LX]:
            delta_x = -self._state.axes[self.ButtonEnum.LX] * self._SCALING
        if self._state.axes[self.ButtonEnum.LY] < -_THRESHOLD_STICK or _THRESHOLD_STICK < self._state.axes[self.ButtonEnum.LY]:
            delta_y = self._state.axes[self.ButtonEnum.LY] * self._SCALING
        if self._state.axes[self.ButtonEnum.RY] < -_THRESHOLD_STICK or _THRESHOLD_STICK < self._state.axes[self.ButtonEnum.RY]:
            delta_z = self._state.axes[self.ButtonEnum.RY] * self._SCALING


        if self._state.axes[self.ButtonEnum.LT] < 0.5:
            delta_theta = (1 - self._state.axes[self.ButtonEnum.LT]) * 0.3
        if self._state.axes[self.ButtonEnum.RT] < 0.5:
            delta_theta = -(1 - self._state.axes[self.ButtonEnum.RT]) * 0.3

        return delta_x, delta_y, delta_z, delta_theta

    def arm_move(self):
        delta_x, delta_y, delta_z, delta_theta = self.delta_calc()
        target_pose = self.calc_target_pose(
            delta_x, delta_y, delta_z, delta_theta)
        self._arm.setTargetPose(target_pose)
        self._arm.go()

    def calc_target_pose(self, delta_x, delta_y, delta_z, delta_theta):
        target_pose = self._arm.getCurrentPose()

        # if _THRESHOLD_LRT < self._state.axes[self.ButtonEnum.LT] or -_THRESHOLD_LRT < self._state.axes[self.ButtonEnum.RT]:
        # 	delta_theta = (np.pi / 4) / 65534 * (self._state.axes[self.ButtonEnum.LT] - self._state.axes[self.ButtonEnum.RT])

        current_orientation = target_pose.orientation
        orientation_list = [current_orientation.x, current_orientation.y,
                            current_orientation.z, current_orientation.w]
        eular = tf.transformations.euler_from_quaternion(orientation_list)
        quat = tf.transformations.quaternion_from_euler(
            np.pi, 0, eular[2] + delta_theta)
        # target_pose.orientation = Quaternion(*quat)
        if self._color == "blue":
            target_pose.position.x += delta_y
            target_pose.position.y -= delta_x
        elif self._color == "red":
            target_pose.position.x -= delta_y
            target_pose.position.y += delta_x
        target_pose.position.z += delta_z
        return target_pose

    def buttonRiseUp(self,button_state,old_button_state,buttonID):
        return (button_state.buttons[buttonID] == 1 and old_button_state.buttons[buttonID] == 0)

    def main(self):
        rate = rospy.Rate(10)
        toggle_grispper_open = 1
        self._old_state = Joy()
        self._old_state.buttons = [0] * 11
        self._old_state.axes = [0, 0, 1, 0, 0, 1, 0, 0]
        is_manual_mode = 0
        while not rospy.is_shutdown():
            self._state = self._gamepad.getState()
            #rospy.loginfo(self._state)
            if self.buttonRiseUp(self._state,self._old_state,self.ButtonEnum.B):
                is_manual_mode = 0
                self._pub_menu.publish(MenuEnum.EMERGENCY_STOP)
            elif self.buttonRiseUp(self._state,self._old_state,self.ButtonEnum.Y):
                is_manual_mode = 0
                self._pub_menu.publish(MenuEnum.PAUSE)
            elif self.buttonRiseUp(self._state,self._old_state,self.ButtonEnum.X):
                is_manual_mode = 0
                self._pub_menu.publish(MenuEnum.START)
            elif self.buttonRiseUp(self._state,self._old_state,self.ButtonEnum.A):
                is_manual_mode = 0
                self._pub_menu.publish(MenuEnum.PAUSE)

            if is_manual_mode == 1:
                if self.buttonRiseUp(self._state,self._old_state,self.ButtonEnum.LB):
                    if toggle_grispper_open == 1:
                        self._gripper.graspBisco(GripperID.NEAR,GripWay.LONG_GRIP,True)
                        self._gripper.graspBisco(GripperID.FAR,GripWay.LONG_GRIP,True)
                        toggle_grispper_open = 0
                    elif toggle_grispper_open == 0:
                        self._gripper.releaseBisco(GripperID.NEAR)
                        self._gripper.releaseBisco(GripperID.FAR)
                        toggle_grispper_open = 1

                else:
                    self.arm_move()
                    pass
            self._old_state = self._state

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ManualManager", anonymous=True)
    rospy.loginfo("Manual Mode!!")
    manual = Manual(XBoxButton)
    manual.main()
