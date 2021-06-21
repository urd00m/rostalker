#!/usr/bin/env python
import rospy 
from sync_robot_test.srv import *
from sync_robot_test.msg import *


#calls robo2
def robo2_start2():
	resp = robo2_startResponse(-1) #None 
	while resp.ack == -1: #will try till sucessful 
		try:
			robo2_key = rospy.ServiceProxy("robo2_start", robo2_start)
			resp = robo2_key(1) #1 means start
			if(resp.ack == 0): #0 means error 
				raise rospy.ServiceException 
		except rospy.ServiceException as e:
			if(resp.ack == 0):
				rospy.loginfo("Acknowledgement failed")
				resp.ack = -1
			rospy.loginfo("Error: %s\tretrying..."%e)
			pass
		time.sleep(2) #2 second intervals

#calls robo2
def tests():
	try:
		robo2_key = rospy.ServiceProxy("robo2_start", robo2_start)
		resp = robo2_key(1) #1 means start
		if(resp.ack == 0): #0 means error 
			raise rospy.ServiceException 
	except rospy.ServiceException as e:
		if(resp.ack == 0):
			rospy.loginfo("Acknowledgement failed")
			resp.ack = -1
		rospy.loginfo("Error: %s\tretrying..."%e)
		pass
	time.sleep(2) #2 second intervals

def tests2():
	robo2_key = rospy.ServiceProxy("robo2_start", robo2_start)
	resp = robo2_key(1) #1 means start

if __name__ == '__main__':
	#robo2_start()
	#tests2()
	#test = rospy.ServiceProxy("robo2_start", robo2_start)
	#resp = test(1) #1 means start
	test = rospy.ServiceProxy("robo2_start", robo2_start)
	resp = test(1)
	print("test")
