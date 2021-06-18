#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist, Vector3
import time 
import sys 
from random import *

def usage():
	rospy.loginfo("1 required parameter (1 for random, 0 for circle), you submitted %s items" % str(len(sys.argv)-1))

def talker(is_rand):
	rospy.init_node("movement_talker", anonymous=True)
	rate = rospy.Rate(.5) # .5 hz, every 2 seconds 
	pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)
        #Allow subscriber time to connect 
        time.sleep(3) #3 seconds
	while not rospy.is_shutdown():
		movement_msg = Twist() 
		if(is_rand == '0'):
			movement_msg = Twist(Vector3(1.0, 1.0, 1.0), Vector3(0.0, 0.0, 1.8))
		else:
			movement_msg = Twist(Vector3(random()*5.0, random()*5.0, random()*5.0), Vector3(random()*3.0, random()*3.0, random()*3.0))
		rospy.loginfo(str(movement_msg.linear) + " " + str(movement_msg.angular))
		pub.publish(movement_msg)	
		rate.sleep()

if __name__ == '__main__':
	if len(sys.argv) != 2:
		print usage()
		sys.exit(1)
	try:
		talker(sys.argv[1])
	except rospy.ROSInterruptException:
		pass
