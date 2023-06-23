#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from motor_controller.srv import *

def add_PID_client(p, i, d, setpoint):
    rospy.wait_for_service('add_data_PID')
    try:
        add_PID = rospy.ServiceProxy('add_data_PID', PID_SRV)
        resp1 = add_PID(p, i, d, setpoint)
        return resp1.Output
    except rospy.ServiceException as e:
        print("Service Call Failed: %s" %e)

def usage():
    return "%s [p i d setpoint]" % sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:
        p = float(sys.argv[1])
        i = float(sys.argv[2])
        d = float(sys.argv[3])
        setpoint = float(sys.argv[4])
    else:
        print(usage())
        sys.exit(1)

    print("Requesting P: %.2f, I: %.2f, D: %.2f, Set Point: %.2f" %(p, i, d, setpoint))
    print("Received Output: %.2f"%(add_PID_client(p, i, d, setpoint)))
