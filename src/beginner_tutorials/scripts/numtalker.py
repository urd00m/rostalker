#!/usr/bin/env python

from beginner_tutorials.msg import *
import rospy
from random import randint 

def talker():
	pub = rospy.Publisher("numbers", Num, queue_size=10)
	rospy.init_node("numbertalker", anonymous=True) 
	rate = rospy.Rate(1) # 1 hz 
	while not rospy.is_shutdown():
		number_msg = Num(randint(0, 100), randint(0, 100), randint(0, 100))
		rospy.loginfo(str(number_msg)) 
		pub.publish(number_msg)
		rate.sleep() 

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass  
