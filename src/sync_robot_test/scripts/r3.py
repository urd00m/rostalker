#!/usr/env/bin python

##############################################################################################################
# This is a resource robot, it moves objects between r1 and r2. No ciruclar wait but mutual exclusion is needed
# atomic writes and reads may also be needed as memory acess isn't protected.
# This is part of a newer model R1, R2, R3 which better simulates what might happen with the OT-2s.
# Creator: Alan Wang
##############################################################################################################

import rospy
import sys
import time
