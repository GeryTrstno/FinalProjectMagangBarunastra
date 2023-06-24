#!/usr/bin/env python

from __future__ import print_function

from motor_controller.srv import PID_Initiate, PID_InitiateResponse
from motor_controller.srv import PID_Setter, PID_SetterResponse
from motor_controller.srv import PID_Feedback, PID_FeedbackResponse
from motor_controller.msg import Motor

import rospy

class PID:
    def __init__(self, p, i, d, set_point):
        self._p = p
        self._i = i
        self._d = d
        self._set_point = set_point

        self.last_error = 0
        self.last_time = 0
        self.integral_cumulation = 0
        self.current_feedback = 0
        self.current_error = 0
        self.cycle_derivative = 0

        self.use_time = False
        self.use_output_bound = False
        self.use_max_i_cum = False

    def calculation(self):
        self.current_error = self._set_point - self.current_feedback
        if self.use_time:
            delta_time = self.current_time - self.last_time
            cycle_integral = (self.last_error + self.current_error / 2) * delta_time
            self.integral_cumulation += cycle_integral
            self.cycle_derivative = (self.current_error - self.last_error) / delta_time
            self.last_time = self.current_time
        else:
            self.integral_cumulation += self.current_error
            self.cycle_derivative = self.current_error - self.last_error

        if self.use_max_i_cum:
            if self.integral_cumulation > self.max_integral_comulation:
                self.integral_cumulation = self.max_integral_comulation
            elif self.integral_cumulation < -self.max_integral_comulation:
                self.integral_cumulation = -self.max_integral_comulation

        output = (self.current_error * self._p) + (self.integral_cumulation * self._i) + (self.cycle_derivative * self._d)
        if self.use_output_bound:
            if output > self.output_upper_bound:
                output = self.output_upper_bound
            elif output < self.output_lower_bound:
                output = self.output_lower_bound

        self.last_error = self.current_error
        return output

    def getP(self):
        return self._p

    def getI(self):
        return self._i

    def getD(self):
        return self._d

    def getPropotional(self):
        return self.current_error * self._p

    def getIntegral(self):
        return self.integral_cumulation * self._i

    def getDerivative(self):
        return self.cycle_derivative * self._d

    def getIntegralCum(self):
        return self.integral_cumulation
    
    def getFeedback(self):
        return self.current_feedback

    def setP(self, p):
        self._p = p

    def setI(self, i):
        self._i = i

    def setD(self, d):
        self._d = d

    def setTarget(self, set_point):
        self._set_point = set_point
    
    def setPID(self, p, i, d):
        self._p = p
        self._i = i
        self._d = d

    def InitializePID(self, p, i, d, set_point):
        self._p = p
        self._i = i
        self._d = d
        self._set_point = set_point

    def setFeedback(self, feedback):
        self.current_feedback = feedback

    def setTime(self, time):
        self.current_time = time

    def setMaxIntegralCum(self, max_integral_cum):
        self.max_integral_comulation = max_integral_cum

    def setOutputBound(self, lower_bound, upper_bound):
        self.output_lower_bound = lower_bound
        self.output_upper_bound = upper_bound


def handle_add_data_PID(req):
    PID_Object.InitializePID(req.P, req.I, req.D, req.SetPoint)
    print("Initialize P: %.2f, I: %.2f, D: %.2f, Set Point: %.2f" % (req.P, req.I, req.D, req.SetPoint))
    output = Motor()
    output.Vel = PID_Object.calculation()

    print("Output: %.2f" %output.Vel)

    hasil = PID_InitiateResponse()
    hasil.Output = output.Vel
    return hasil

def handle_set_data_PID(req):
    PID_Object.setPID(req.Kp, req.Ki, req.Kd)
    print("Setting P: %.2f, I: %.2f, D: %.2f" % (req.Kp, req.Ki, req.Kd))
    output = Motor()
    output.Vel = PID_Object.calculation()

    print("Output: %.2f" %output.Vel)

    hasil = PID_SetterResponse()
    hasil.Output = output.Vel
    return hasil

def handle_set_data_Feedback(req):
    PID_Object.setFeedback(req.Feedback)

    output = Motor()
    output.Vel = PID_Object.calculation()
    print("PID Feedback: %.2f" %PID_Object.getFeedback())
    print("Output Vel: %.2f" %output.Vel)

    hasil = PID_FeedbackResponse()
    hasil.Output = output.Vel
    hasil.Curr = PID_Object.getPropotional()
    hasil.Deriv = PID_Object.getDerivative()
    hasil.Integral = PID_Object.getIntegral()
    return hasil
    


def add_PID_server():
    rospy.init_node('PID_server')
    rospy.Service('Initialize_data_PID', PID_Initiate, handle_add_data_PID)
    rospy.Service('Set_data_PID', PID_Setter, handle_set_data_PID)
    rospy.Service('Set_Data_Feedback', PID_Feedback, handle_set_data_Feedback)
    print("Ready to Configure PID")
    rospy.spin()

if __name__ == "__main__":
    PID_Object = PID(0, 0, 0, 0)
    add_PID_server()
