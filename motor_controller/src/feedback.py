#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from motor_controller.srv import PID_Feedback

class PIDFeedbackController:
    def __init__(self):
        self.pid_service = rospy.ServiceProxy('Set_Feedback', PID_Feedback)
        self.target_position = 0.0

    def set_target_position(self, position):
        self.target_position = position

    def handle_odometry(self, msg):
        current_position = msg.pose.pose.position.x

        # Calculate error
        error = self.target_position - current_position

        # Call PID service with the error as feedback
        try:
            response = self.pid_service(error)
            # Process the response if needed
            print("PID Output:", response.output)
        except rospy.ServiceException as e:
            print("Service call failed:", str(e))

def main():
    rospy.init_node('pid_feedback_controller')
    controller = PIDFeedbackController()
    controller.set_target_position(1.0)  # Set the target position

    rospy.Subscriber('mavros/global_position/local', Odometry, controller.handle_odometry)
    rospy.spin()

if __name__ == "__main__":
    main()
