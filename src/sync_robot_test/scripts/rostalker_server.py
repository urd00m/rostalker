#!/usr/bin/env python
import rospy
import sys
import time
from sync_robot_test.srv import *
from sync_robot_test.msg import *
from threading import Thread, Lock
from jobs import _move_job, terminate_check, decrease_job, increase_job

###########################################################################
# The script to be run on the master machine. This is also R3 in the model
# R1-R2-R3, where services for this robot involve moving objects from robot1
# to robot2. This is basically a shared resource
# Creator: Alan Wang
#
#
# Service list:
# /rostalker_master/move - parameters: start, destination, num_objects - returns: status
# /rostalker_master/register - parameters: - returns: ID
# /rostalker_master/get_worker_info - parameters: - returns: info
###########################################################################

global move_lock
global register_lock
global worker_count
global worker_info

# This is done with a lock because we pretend that this is a robotic arm, once it starts moving nothing else should be allowed to mess with it
def handle_move(req):
	global move_lock
	start = req.location
	destination = req.destination
	num_objects = req.num_objects 
	rospy.loginfo("Service starting moving %s objects from robot %s to robot %s", str(num_objects), str(start), str(destination))
	move_lock.acquire()
	rospy.loginfo("%s acquired lock", str(start))
	try:
		_move_job(start, destination, num_objects) # Abstraction
		rospy.loginfo("%s job finished", str(start))
		return rostalker_moveReponse(0) # All Good 
	except rospy.ROSException:
		rospy.logerr("Job call error")
		return rostalker_moveReponse(1) # Error occured 
	finally:
		move_lock.release() #allow others to work

# This registers a worker node and gives it a unique ID
# If return ID is -1 then an error occured 
def handle_register(req):
	global register_lock
	global worker_count
	global worker_info
	register_lock.acquire()
	try:
		worker_count = worker_count + 1
		worker_info.append(worker_count) # Add it to list of workers (Multiple robot support)
		return registerResponse(worker_count)
	except rospy.ROSException:
		rospy.logerr("Registration failed, try again...")
		return registerReponse(-1)
	finally:
		register_lock.release()

# Handles worker info requests
# Returns a list of all the workers
def handle_get_worker_info(req):
	global worker_info
	return worker_infoResponse(worker_info)

def node_init():
	rospy.init_node("rostalker_master", anonymous=True)
	rospy.loginfo("rostalker_master node started")

def init():
	global move_lock
	global register_lock
	global worker_count
	global worker_info
	rospy.loginfo("Starting services")
	move_service = rospy.Service("/rostalker_master/move", rostalker_move, handle_move)
	register_worker_service = rospy.Service("/rostalker_master/register", register, handle_register)
	worker_info_service = rospy.Service("/rostalker_master/get_worker_info", worker_info, handle_get_worker_info)
	rospy.wait_for_service("/rostalker_master/move")
	rospy.wait_for_service("/rostalker_master/register")
	rospy.wait_for_service("/rostalker_master/get_worker_info")
	rospy.loginfo("Services initialized, intializing locks")
	move_lock = Lock()
	register_lock = Lock()
	worker_count = 0
	worker_info = []
	rospy.loginfo("Locks acquired, all rostalker_master items initialized")

if __name__ == '__main__':
	rospy.loginfo("Starting rostalker_master")
	try:
		node_init()
	except rospy.ROSException:
		rospy.logfatal("Failed to initialize rostalker_master node exiting...")
		sys.exit(1)
	try:
		init()
	except rospy.ROSException:
		rosyp.logfatal("Failed to initialize rostalker_master items")
		sys.exit(1)

	# Now wait for services
	rospy.loginfo("Starting spin, waiting for calls...")
	rospy.spin()
