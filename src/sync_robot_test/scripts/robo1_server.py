#!/usr/bin/env python


#Robo 1 node starts the data publishing stops and transfers to robo 2 repeat 

import rospy
import sys
import time
from sync_robot_test.msg import *
from sync_robot_test.srv import *
from random import random
global data_pub # Data is published here 
global ack_pub # Acknowledgements back to robo1 sent here 
global lock #Lock = 0 means locked, lock = 1 means running     (initially unlocked at runtime)

#Simple non-atomic spin lock
def spin():
	global lock 
	while(lock != 1 and rospy.is_shutdown() == False):
		time.sleep(.5) #so the cpu isn't completely burned out  

#Checks if process needs to be termianted 
def terminate_check():
	if(rospy.is_shutdown()):
		rospy.loginfo("Process terminating...")
		sys.exit(1)


#Service to transfer back control to robo1
def handle_robo1_start(req):
	global lock
	rospy.loginfo("Recieved robo1 control transfer, transferring...")
	if(lock == 1):
		rospy.loginfo("Warning! Unlocking and already unlocked robo1, DEADLOCK POTENTIAL")
		return robo1_startResponse(0) #Force it to retry/acknowledgement error
	lock = 1
	return robo1_startResponse(1)

#brings up services and publishers 
def init():
	global lock
	global data_pub
	global ack_pub
	lock = 1 #robo1 starts off unlocked
	start_service = rospy.Service("/robo1/start", robo1_start, handle_robo1_start)
	data_pub = rospy.Publisher("/robo1/data", data, queue_size=10) 
	ack_pub = rospy.Publisher("/robo1/ack", ack, queue_size=10) 
	rospy.wait_for_service("/robo1/start")
	time.sleep(3) #3 seconds 
	rospy.loginfo("Publishers and services initialized, ready to handle requests")

#Brings up nodes 
def server_start():
	rospy.init_node("robo1", anonymous=True)
	rospy.loginfo("robo1 node created")

#Function to publish 10 pieces of "scientific" data
def data_publish():
	rate = rospy.Rate(10) #10 hz
	for i in range(0, 10):
		a = random()*100
		b = random()*100
		c = random()*100
		d = random()*100
		data_msg = data(a, b, c, d)
		data_pub.publish(data_msg)
		rate.sleep()
	#TODO: ack checks to see if all 10 were sent and recieved 

#calls robo2
def robo1_to_robo2():
	global lock
	resp = robo2_startResponse(-1) #None 
	while resp.ack == -1: #will try till sucessful 
		try:
			robo2_key = rospy.ServiceProxy("/robo2/start", robo2_start)
			resp = robo2_key(1) #1 means start
			if(resp.ack == 0): #0 means error 
				raise rospy.ServiceException 
			lock = 0 #Relocking so that we don't interfere with robo2 
		except rospy.ServiceException as e:
			terminate_check()
			if(resp.ack == 0):
				rospy.loginfo("Acknowledgement failed")
				resp.ack = -1
			rospy.loginfo("Error: %s\tretrying..."%e)
			time.sleep(2) #2 second intervals
			pass

#robo1 starts doing work 
def start():
	global lock
	rospy.loginfo("Waiting on robo2...")
	rospy.wait_for_service("/robo2/start")
	rospy.loginfo("Both robo1 and robo2 are online and servers are ready to recieve") 
	try:
		input("Press <ENTER> key to start this process")
	except:
		pass
	spin() #simple spin lock 
	while not rospy.is_shutdown() and lock == 1: 
		rospy.loginfo("robo1 gather info")
		#time.sleep(5) #5 seconds
		rospy.loginfo("robo1 publishing info")
		data_publish()
		#time.sleep(5) #5 seconds
		rospy.loginfo("transfering control to robo2")
		robo1_to_robo2()
		rospy.loginfo("control transfered waiting to restart...")
		spin() #Simple spinlock 

if __name__ == '__main__':
	try:
		server_start()
	except rospy.ROSException:
		rospy.loginfo("Failed bring up robo1 node")
		sys.exit(1)

	try:
		init()
	except rospy.ROSException:
		rospy.loginfo("Error initializing publishers and/or services")
		sys.exit(1)

	start()
