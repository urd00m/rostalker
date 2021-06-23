#!/usr/bin/env python
import rospy 
import time
import sys
from sync_robot_test.srv import *
from threading import Thread, Lock

global test
global lock

def handle_service(req):
	global test
	global lock
	rospy.loginfo("Handling service from %s"%str(req.inc))
	for i in range(0, 1000000):
		lock.acquire()
		try:
			test = test+1
		finally:
			lock.release()
	rospy.loginfo("Done %s"%str(test))
	return service_testResponse(1)


if __name__ == '__main__':
	# Just spin in a loop forever
	global test
	global lock
	rospy.init_node("service_test_server", anonymous=True)
	s1 = rospy.Service("test", service_test, handle_service)
	rospy.wait_for_service("test")
	rospy.loginfo("All set, starting spin")
	count = 0
	test = 0
	lock = Lock()
	while True:
		rospy.loginfo(str(count))
		count = count + 1 
		time.sleep(1)
