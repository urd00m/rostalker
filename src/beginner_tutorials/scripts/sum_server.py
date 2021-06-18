#!/usr/bin/env python

from __future__ import print_function
import rospy
import sys 
from beginner_tutorials.srv import *

def handle_add_two_ints(req):
	print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
	return sumResponse(req.a + req.b) 

def add_two_ints_server():
	rospy.init_node('sum') 
	s = rospy.Service('sum', sum, handle_add_two_ints)
	print("Ready to add two ints.") 
	rospy.spin()

if __name__ == '__main__':
	try:
		add_two_ints_server()
	except rospy.ROSInterruptException:
		pass
