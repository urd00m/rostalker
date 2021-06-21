#!/usr/bin/env python
import sys
import rospy
import time
from geometry_msgs.msg import Twist, Vector3
from turtletest.srv import *
from turtletest.msg import *

global pub
global pub_ack


def handle_forward(req):
	rospy.loginfo("Moving turtle forward " + str(req.dist) + " units")
	try:
		f_msg = Twist(Vector3(req.dist, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)) #Twist mesage
		pub.publish(f_msg)
		ack_msg = ack("Request processed for foward movement of " + str(req.dist) + " units")
		pub_ack.publish(ack_msg)
		return forwardResponse(1) #All good
	except:
		rospy.loginfo("Failed to move turtle forward")
		return forwardResponse(0) #Error

def forward_server():
	global pub
	global pub_ack
	s1 = rospy.Service("forward", forward, handle_forward) # req - float64 dist, resp - int64 ack (1 is recieved and done, 0 error occured)
	pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=5)
	pub_ack = rospy.Publisher("acknowledge", ack, queue_size=10)
	rospy.wait_for_service("forward")
	rospy.loginfo("node initilized, s1 ready to handle requests")


if __name__ == '__main__':
	try:
		rospy.init_node("turtlesim_server_node", anonymous=True)
		rospy.loginfo("Node created sucessfully")
	except:
		rospy.loginfo("Node creation failed")
		sys.exit(1)

	try:
		forward_server()
	except rospy.ROSInterruptShutdown:
		rospy.loginfo("rospy shutdown")
		pass

	#TODO 

	rospy.spin()
