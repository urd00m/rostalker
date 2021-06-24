#!/usr/bin/env python
import rospy
import time
import sys
from sync_robot_test.srv import *
from sync_robot_test.msg import *
from threading import Thread, Lock
from jobs import terminate_check, get_info, worker_job, work, move_job, get_workers, get_worker_info

###########################################################################
# The script to be run on the worker machines. This is also R3 in the model
# R1-R2-R3, where services for this robot involve moving objects from robot1
# to robot2. This is basically a shared resource
# Creator: Alan Wang
###########################################################################
global id
global size
global max_size
global shutdown # In case of error this can stop the entire robot

# Returns basic information on this robot
def handle_info(req):
	global id
	global size
	global max_size
	global shutdown
	return workerResponse(id, size, max_size, shutdown)

# Shuts down the robot
def shutdown(req):
	global shutdown
	shutdown = 1 #Shutdown
	return action_statusResponse(0)

# Restarts the robot
def restart(req):
	global shutdown
	shutdown = 0 #Restart
	return action_statusResponse(0)

def node_init():
        rospy.init_node("rostalker_worker", anonymous=True)
        rospy.loginfo("rostalker_worker node started")

# Registers the robot with master 
def registration():
	global id
	id = -1
	while(id == -1):
		terminate_check()
		try:
			rospy.loginfo("Waiting for service...")
			rospy.wait_for_service("/rostalker_master/register")
			registration_call = rospy.ServiceProxy("/rostalker_master/register", register)
			id = registration_call().ID
			if(id == -1):
				raise rospy.ServiceException
		except rospy.ServiceException as e:
			rospy.loginfo("Failed to register, retrying...")
	rospy.loginfo("Registration Sucess!")

# Worker initialization
def init():
	global id
	global size
	global max_size
	global shutdown
	rospy.loginfo("Waiting on master's services")
	rospy.wait_for_service("/rostalker_master/move")
        rospy.wait_for_service("/rostalker_master/register")
        rospy.wait_for_service("/rostalker_master/get_worker_info")
	rospy.loginfo("Service intialization")
	info_service = rospy.Service("/rostalker_worker"+str(id)+"/info", worker, handle_info)
	shutdown_service = rospy.Service("/rostalker_worker"+str(id)+"/shutdown", action_status, shutdown)
	restart_service = rospy.Service("/rostalker_worker"+str(id)+"/restart", action_status, restart)
	rospy.wait_for_service("/rostalker_worker"+str(id)+"/info")
	rospy.wait_for_service("/rostalker_worker"+str(id)+"/shutdown")
	rospy.wait_for_service("/rostalker_worker"+str(id)+"/restart")
	rospy.loginfo("Services ready, obtaining robot information")
	#obtaining robot information
	info = get_info()
	size = info[0]
	max_size = info[1]
	shutdown = 0
	rospy.loginfo("Retrieved robot information, initialization complete")

# Worker does work here
def start_working():
	global id
	worker_job(id)

if __name__ == '__main__':
        rospy.loginfo("Starting rostalker_worker")
	try:
		node_init()
	except rospy.ROSException:
		rospy.logfatal("Failed to initialize rostalker_master node exiting...")
                sys.exit(1)

	try:
		registration()
	except rospy.ROSException:
		rospy.logfatal("Failed to register node exiting...")
                sys.exit(1)

	try:
		init()
	except rospy.ROSException:
		rospy.logfatal("Failed to init services and data exiting...")
                sys.exit(1)

	start_working()

