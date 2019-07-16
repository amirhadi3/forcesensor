#! /usr/bin/env python

import rospy
from forcesensor.msg import SensorOutput
import time

count = 0
start = 0

def callback(msg):
	global count
	if count == 0:
		global start
		start = time.time()
	if count == 1500:
		stop = time.time()
	# if count < 1500:
	# 	rospy.loginfo(msg)
	count += 1


rospy.init_node('conti_data_sub')
sub = rospy.Subscriber('force_data', SensorOutput, callback)
rospy.spin()