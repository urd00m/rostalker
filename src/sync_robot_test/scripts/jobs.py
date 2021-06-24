#!/usr/bin/env python
import rospy
import sys
import time
from random import random
from sync_robot_test.srv import *
from sync_robot_test.msg import *

###########################################################################
#
# Helper Functions
#
###########################################################################

# Checks if process needs to be termianted 
def terminate_check():
        if(rospy.is_shutdown()):
                rospy.logfatal("Process terminating...")
                sys.exit(1)

# Helper function for readablility in move job
# If move service sucessful returns 0, else returns 1 (error)
def move_job(start, destination, num_objects):
	status = 0
	move_service = rospy.ServiceProxy("/rostalker_master/move", rostalker_move)
	status = move_service(start, destination, num_objects).status
	if(status == 1):
		rospy.logerr("/rostalker_master/move service failed")
	return status

# Helper function to get the worker info
def get_workers():
	workers = []
	get_workers_service = rospy.ServiceProxy("/rostalker_master/get_worker_info", worker_info)
	workers = get_workers_service().info
	return workers

# Helper function to get worker information
def get_worker_info(id):
	get_worker_info_service = rospy.ServiceProxy("/rostalker_worker"+str(id)+"/info", worker)
	info = get_worker_info_service()
	return info

###########################################################################
# Abstraction, code the user wrote that has the bare minimum amount of ROS.
# This is the only thing we want those looking at this abstraction to worry about.
###########################################################################
#TODO: update the jobs to factor in worker size

# In real life this would pick up objects from start
def decrease_job(start, end, num_objects):
	name = "/"+str(start)+"/decrease"
	time.sleep((num_objects/2.0)) #Removing objects from start 
	service_call = rospy.ServiceProxy(name, decrease)
	return service_call(num_objects).status

# In real life this would put down objects in end
def increase_job(start, end, num_objects):
	name = "/"+str(end)+"/increase"
	time.sleep((num_objects/2.0)) #moving objects to end
	service_call = rospy.ServiceProxy(name, increase)
	return service_call(num_objects).status

# What the worker will do
def work(id):
	time.sleep(random()*5)
	workers = get_workers()
	# Moving to that worker
	return move_job(id, workers[int(random()*len(workers))], int(random()*10)) #Select a random worker witha random number of item transfer 

# Get robot info
def get_info():
	return (7, 10) # size, max_size

###########################################################################
# This implements the abstraction provided in the server and to the above.
# The above doesn't need to worry about the exception handling which is done
# in this section. This allows for code that has as little ROS as needed.
###########################################################################

# Handles all the nasty things needed for the move job
def _move_job(start, end, num_objects):
	status = 1
	err_count = 0
	while(status == 1):
		terminate_check()
		try:
			status = decrease_job(start, end, num_objects)
			if(status == 1):
				raise rospy.ServiceException
		except rospy.ServiceException as e:
			rospy.logerr("Failed to use service, error: %s", e)
			err_count = err_count + 1
			if(err_count == 1):
				status = -1 # to exit out
				rospy.logerr("Too many tries, failing")
			else:
				rospy.loginfo("Retrying to connect to service...")
	if(err_count==1):
		raise rospy.ROSException #return back to the caller

	status = 1
	err_count = 0
	while(status == 1):
		terminate_check()
		try:
			status = increase_job(start, end, num_objects)
			if(status == 1):
				raise rospy.ServiceException
		except rospy.ServiceException as e:
			rospy.logerr("Failed to use service, error: %s", e)
			err_count = err_count + 1
                        if(err_count == 10):
				status = -1
                                rospy.logerr("Too many tries, failing")
                        else:
                                rospy.loginfo("Retrying to connect to service...")
	if(err_count==10):
		raise rospy.ROSException #return back to the caller

	rospy.loginfo("Job done moved %s objects from %s to %s", str(num_objects), str(start), str(destination))

# Handles all the nasty things needed for a worker to do its job
def worker_job(id):
	rospy.loginfo("Worker id: %s starting", str(id))
	while not rospy.is_shutdown():
		try: 
			status = work(id)
			if(status == 1):
				rospy.logerr("move_job error")
				raise rospy.ROSException
		except rospy.ROSException:
			shutdown_service = rospy.ServiceProxy("/rostalker_worker"+str(id)+"/shutdown", action_status)
			status = shutdown_service().status
			while(get_worker_info(id).shutdown == 1):
				rospy.logerr("Error occured waiting on human input before continuing, please call service /rostalker_worker%s/restart", str(id))
				time.sleep(3)
