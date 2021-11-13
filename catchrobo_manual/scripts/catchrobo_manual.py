#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from catchrobo_manager.arm import Arm
from catchrobo_manager.gripper_manager import GripperManager
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf


class PS3Button():
	#Buttons
	SELECT = 0
	LSTICK = 1
	RSTICK = 2
	START = 3
	UP = 4
	RIGHT = 5
	DOWN = 6	
	LEFT = 7
	L2 = 8
	R2 = 9
	L1 = 10
	R1 = 11
	TRIANGLE = 12
	CIRCLE = 13
	CROSS = 14
	SQUARE = 15
	PS = 16
	#Axes
	LX = 0
	LY = 1
	RX = 2
	RY = 3

	K_CON = 1

class XBoxButton():
	#Buttons
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
	#Axes
	LX = 0
	LY = 1
	LT = 2
	RX = 3
	RY = 4
	RT = 5
	LEFTRIGHT = 6
	UPDOWN = 7

	K_CON = 1


class GamePad():
	def __init__(self):

		self._state = Joy()
		rospy.Subscriber("joy", Joy, self.joyCallback)

	def joyCallback(self, joy_msg):
		# self._button = joy_msg.button + joy_msg.axes
		self._state = joy_msg

	def getState(self):
		return self._state

class Manual():
	def __init__(self, Button):
		color = rospy.get_param("/color", default="blue")
		self._target_pose = PoseStamped()
		self._arm = Arm()
		self._gripper = GripperManager(color)
		self._game_pad = GamePad()
		self.ButtonEnum = Button

	def move(self):
		while not rospy.is_shutdown():
			self._state = self._game_pad.getState()
			rospy.loginfo(self._state)
			if self._state.buttons[self.ButtonEnum.A] == 1:

				pass
			elif self._state.buttons[self.ButtonEnum.B] == 1:
				pass
			elif self._state.buttons[self.ButtonEnum.X] == 1:
				pass
			elif self._state.buttons[self.ButtonEnum.Y] == 1:
				pass
			elif self._state.buttons[self.ButtonEnum.LB] == 1:
				pass
			elif self._state.buttons[self.ButtonEnum.RB] == 1:
				pass
			elif self._state.buttons[self.ButtonEnum.START] == 1:
				pass
			elif self._state.buttons[self.ButtonEnum.BACK] == 1:
				pass
			elif self._state.buttons[self.ButtonEnum.XBOX] == 1:
				pass
			elif self._state.buttons[self.ButtonEnum.LSTICK] == 1:
				pass
			elif self._state.buttons[self.ButtonEnum.RSTICK] == 1:
				pass
			else:
				self._target_pose = self._arm.getTargetPose()
				_MAX_XBOX_AXES = 32767
				_THRESHOLD_STICK = _MAX_XBOX_AXES * 0.2
				_THRESHOLD_LRT = -_MAX_XBOX_AXES *0.8
				delta_x = 0
				delta_y = 0
				delta_z = 0
				delta_theta = 0
				if self._state.axes[self.ButtonEnum.LX] < -_THRESHOLD_STICK and _THRESHOLD_STICK < self._state.axes[self.ButtonEnum.LX]:
					delta_x = self._state.axes[self.ButtonEnum.LX]
				if self._state.axes[self.ButtonEnum.LY] < -_THRESHOLD_STICK and _THRESHOLD_STICK < self._state.axes[self.ButtonEnum.LY]:
					delta_y = -self._state.axes[self.ButtonEnum.LY]
				if self._state.axes[self.ButtonEnum.RY] < -_THRESHOLD_STICK and _THRESHOLD_STICK < self._state.axes[self.ButtonEnum.RY]:
					delta_z = -self._state.axes[self.ButtonEnum.RY]
				if _THRESHOLD_LRT < self._state.axes[self.ButtonEnum.LT] or -_THRESHOLD_LRT < self._state.axes[self.ButtonEnum.RT]:
					delta_theta = (np.pi / 4) / 65534 * (self._state.axes[self.ButtonEnum.LT] - self._state.axes[self.ButtonEnum.RT])
				eular = tf.transformations.euler_from_quaternion(self._target_pose.pose.orientation)
				self._target_pose.pose.orientation += tf.transformations.quaternion_from_euler(np.pi, 0, delta_theta)
				self._target_pose.pose.position.x += delta_x
				self._target_pose.pose.position.y += delta_y
				self._target_pose.pose.position.z += delta_z
				self._arm.setTargetPose(self._target_pose)
				self._arm.go()
				rospy.spin()


if __name__ == "__main__":
	rospy.init_node("ManualManager", anonymous=True)
	rospy.loginfo("Manual Mode!!")
	manual = Manual(XBoxButton)
	manual.move()