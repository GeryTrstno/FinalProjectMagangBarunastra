#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from motor_controller.srv import *

def add_PID_client(p, i, d, set_point):
    rospy.wait_for_service('add_PID')
    try:
        add_PID = rospy.ServiceProxy('add_PID', PID)
        resp1 = add_PID(p, i, d, set_point)
        return resp1.Output
    except rospy.ServiceException as e:
        print("Service Call Failed: %s" %e)

def usage():
    return "%s [p i d set_point]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:
        p = float(sys.argv[1])
        i = float(sys.argv[2])
        d = float(sys.argv[3])
        set_point = float(sys.argv[4])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting P:%s, I:%s, D:%s, Set Point:%s" %(p, i, d, set_point))
    print("")

