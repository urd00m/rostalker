#!/usr/bin/env python
from __future__ import print_function

import rospy
import sys
import time 
from turtletest.msg import *
from turtletest.srv import * 
from geometry_msgs.msg import Twist, Vector3

def usage():
	print("Please type in one argument for the forward distance the turtle will travel, you submitted: %s arguments"%str(len(sys.argv)-1))

def forward_client(dist):
	rospy.wait_for_service("forward")
	try:
		f_service = rospy.ServiceProxy("forward", forward)
		resp = f_service(dist)
		if(resp.ack == 0):
			print("Acknowledgement error, error code: %s"%resp.ack)
			raise rospy.ServiceException
		else:
			return resp.ack
	except rospy.ServiceException as e:
		print("Service call failed: %s" % e) 


if __name__ == '__main__':
	if(len(sys.argv) != 2):
		usage()
		sys.exit(1)

	dist = float(sys.argv[1])
	print("Requesting forward service...")
	forward_client(dist)
	print("Service complete")
