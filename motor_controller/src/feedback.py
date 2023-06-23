#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from motor_controller.srv import PID_Feedback

def initiate_pid(feedback):
    rospy.wait_for_service('Set_Feedback')
    try:
        pid_req = rospy.ServiceProxy('Set_Feedback', PID_Feedback)
        response = pid_req(feedback)
        return response.newResult
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def odometry_callback(msg):
    # Extract the feedback data from the received odometry message
    feedback = msg.pose.pose.position.x

    # Call the service to initiate PID with the feedback data
    initiate_pid(feedback)

def twist_callback(msg):
    # Extract the feedback data from the received twistStamped message
    feedback = msg.twist.linear.x

    # Call the service to initiate PID with the feedback data
    initiate_pid(feedback)

if __name__ == "__main__":
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber('mavros/global_position/local', Odometry, odometry_callback)
        rospy.Subscriber('mavros/global_position/velocity', TwistStamped, twist_callback)
        rospy.spin()