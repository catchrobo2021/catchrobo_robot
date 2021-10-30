#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from catchrobo_manager.arm import Arm
from geometry_msgs.msg import PoseStamped


class PS3Button():
	SELSCT = 0
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
	LX = 0
	LY = 1
	RX = 2
	RY = 3

	K_CON = 1


class PS3():
	def __init__(self):

		self._state = Joy()
		rospy.Subscriber("joy", Joy, self.joyCallback)

	def joyCallback(self, joy_msg):
		# self._button = joy_msg.button + joy_msg.axes
		self._state = joy_msg

	def getState(self):
		return self._state

class Manual():
	def __init__(self):
		self._target_pose = PoseStamped()
		self._arm = Arm()
		self._ps3 = PS3()

	def move(self):
		while not rospy.is_shutdown():
			self._state = self.ps3.getState()
			if self._state.buttons[PS3Button.SELECT] == 1:
				rospy.loginfo("SELECT")
			elif self._state.buttons[PS3Button.LSTICK] == 1:
				rospy.loginfo("LSTICK")
			elif self._state.buttons[PS3Button.RSTICK] == 1:
				rospy.loginfo("RSTICK")
			elif self._state.buttons[PS3Button.START] == 1:
				rospy.loginfo("START")
			elif self._state.buttons[PS3Button.UP] == 1:
				rospy.loginfo("UP")
			elif self._state.buttons[PS3Button.RIGHT] == 1:
				rospy.loginfo("RIGHT")
			elif self._state.buttons[PS3Button.DOWN] == 1:
				rospy.loginfo("DOWN")
			elif self._state.buttons[PS3Button.LEFT] == 1:
				rospy.loginfo("LEFT")
			elif self._state.buttons[PS3Button.L2] == 1:
				rospy.loginfo("L2")
			elif self._state.buttons[PS3Button.R2] == 1:
				rospy.loginfo("R2")
			elif self._state.buttons[PS3Button.L1] == 1:
				rospy.loginfo("L1")
			elif self._state.buttons[PS3Button.R1] == 1:
				rospy.loginfo("R1")
			elif self._state.buttons[PS3Button.TRIANGLE] == 1:
				rospy.loginfo("TRIANGLE")
			elif self._state.buttons[PS3Button.CIRCLE] == 1:
				rospy.loginfo("CIRCLE")
			elif self._state.buttons[PS3Button.CROSS] == 1:
				rospy.loginfo("CROSS")
			elif self._state.buttons[PS3Button.SQUARE] == 1:
				rospy.loginfo("SQUARE")
			elif self._state.buttons[PS3Button.PS] == 1:
				rospy.loginfo("PS")
			else:
				self._target_pose = self._arm.getTargetPose()
				self._THRESHOLD_STICK = 0.2
				self.delta_x = 0
				self.delta_y = 0
				if self._state.axes[PS3Button.LX] < -self._THRESHOLD_STICK and self._THRESHOLD_STICK < self._state.axes[PS3Button.LX]:
					self.delta_x = self._state.axes[PS3Button.LX]
				if self._state.axes[PS3Button.LY] < -self._THRESHOLD_STICK and self._THRESHOLD_STICK < self._state.axes[PS3Button.LY]:
					self.delta_y = self._state.axes[PS3Button.LY]
				
				self._target_pose.pose.position.x+=self.delta_x
				self._target_pose.pose.position.y+=self.delta_y
				self._arm.move(self._target_pose)
				rospy.spin()


if __name__ == "__main__":
	rospy.init_node("ManualManager", anonymous=True)
	rospy.loginfo("Hello World!!")
	manual = Manual()
	manual.move()