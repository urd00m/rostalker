#!/usr/bin/env python

import rospy
from beginner_tutorials.msg import * 

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " Numbers are: " + str(data.a) + " " + str(data.b) + " " + str(data.c))

def listener():
	rospy.init_node("numberlistener", anonymous=True)

	rospy.Subscriber("numbers", Num, callback) 
	
	rospy.spin() 

if __name__ == '__main__':
	try: 
		listener()
	except rospy.ROSInterruptException:
		pass 
	 
	
