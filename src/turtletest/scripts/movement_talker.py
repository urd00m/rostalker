#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist, Vector3
import time 

def talker():
	rospy.init_node("movement_talker", anonymous=True)
	rate = rospy.Rate(.5) # .5 hz, every 2 seconds 
	pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)
        #Allow subscriber time to connect 
        time.sleep(3) #3 seconds
	while not rospy.is_shutdown():
		movement_msg = Twist(Vector3(1.0, 1.0, 1.0), Vector3(0.0, 0.0, 1.8))
		rospy.loginfo(str(movement_msg.linear) + " " + str(movement_msg.angular))
		pub.publish(movement_msg)	
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
