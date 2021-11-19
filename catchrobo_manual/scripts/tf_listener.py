#!/usr/bin/env python
# -*- coding: utf-8 -*-
#import roslib
#roslib.load_manifest('learning_tf')
import rospy
#import math
import tf
#import geometry_msgs.msg
#import turtlesim.srv
 
if __name__ == '__main__':
	rospy.init_node('tf_turtle')
	listener = tf.TransformListener()
 
	rate = rospy.Rate(10.0)
	rospy.loginfo("Hello World!!")
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/world', '/arm/link_tip', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		rospy.loginfo(trans)
		rospy.loginfo(rot)

		#rospy.loginfo("trans:%f",trans)
		#rospy.loginfo("trans:%f",rot)
 
		rate.sleep()