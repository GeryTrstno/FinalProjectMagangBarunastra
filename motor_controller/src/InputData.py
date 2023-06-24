#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from motor_controller.srv import PID_Initiate, PID_Setter

def Initialize_PID_client(p, i, d, setpoint):
    rospy.wait_for_service('Initialize_data_PID')
    try:
        add_PID = rospy.ServiceProxy('Initialize_data_PID', PID_Initiate)
        resp1 = add_PID(p, i, d, setpoint)
        return resp1.Output
    except rospy.ServiceException as e:
        print("Service Call Failed: %s" %e)

def Set_PID_client(p, i, d):
    rospy.wait_for_service('Set_data_PID')
    try:
        set_PID = rospy.ServiceProxy('Set_data_PID', PID_Setter)
        resp1 = set_PID(p, i, d)
        return resp1.Output
    except rospy.ServiceException as e:
        print("Service Call Failed: %s" %e)

def usage():
    return "%s [P I D Set_Point] to initialize PID, or [P I D] to set P I D" % sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:
        p = float(sys.argv[1])
        i = float(sys.argv[2])
        d = float(sys.argv[3])
        setpoint = float(sys.argv[4])
        
        print("Initializing P: %.2f, I: %.2f, D: %.2f, Set Point: %.2f" %(p, i, d, setpoint))
        print("Received Output: %.2f"%(Initialize_PID_client(p, i, d, setpoint)))

    elif len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])

        print("Setting P: %.2f, I: %.2f, D: %.2f" %(x, y, z))
        print("Received Output: %.2f"%(Set_PID_client(x, y, z)))    

    else:
        print(usage())
        sys.exit(1)
