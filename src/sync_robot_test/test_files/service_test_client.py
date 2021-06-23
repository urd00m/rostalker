#!/usr/bin/env python
import rospy
import time
import sys
from sync_robot_test.srv import *




if __name__ == '__main__':
	rospy.init_node("service_test_client", anonymous=True)
	test = rospy.ServiceProxy("/test", service_test)
	resp = test(1)
	rospy.loginfo("done")
