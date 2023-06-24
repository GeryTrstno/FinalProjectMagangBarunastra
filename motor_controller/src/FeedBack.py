#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from motor_controller.srv import PID_Feedback

def odometry_callback(msg):
    # Process the odometry data here
    # Example: Print the position and orientation
    print("Position: x={}, y={}, z={}".format(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
    print("Orientation: x={}, y={}, z={}, w={}".format(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

rospy.init_node('odometry_subscriber')
rospy.Subscriber('/mavros/local_position/odom', Odometry, odometry_callback)
rospy.spin()
    

def main():
    rospy.init_node('pid_feedback_controller')
    controller = PIDFeedbackController()
    controller.set_target_position(1.0)  # Set the target position

    rospy.Subscriber('mavros/local_position/odom', Odometry, controller.handle_odometry)
    rospy.spin()

if __name__ == "__main__":
    main()
