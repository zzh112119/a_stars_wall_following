#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np
import math
import csv

class pid_controller:


	def __init__(self, P=0.0, I=0.0, D=0.0):

		self.Kp = P
		self.Ki = I
		self.Kd = D

		self.sample_time = 0.00
		self.current_time = rospy.get_time()
		self.last_time = self.current_time
		self.SetPoint = 0.0

		self.clear()    

	def clear(self):
		"""Clears PID computations and coefficients"""
		self.SetPoint = 0.0

		self.PTerm = 0.0
		self.ITerm = 0.0
		self.DTerm = 0.0
		self.last_error = 0.0

		# Windup Guard
		self.int_error = 0.0
		self.windup_guard = 20.0

		self.output = 0.0

	def update(self, feedback_value):
		"""Calculates PID value for given reference feedback
		.. math::
			u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
		.. figure:: images/pid_1.png
		   :align:   center
		   Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
		"""
		error = feedback_value - self.SetPoint 
		# print(self.SetPoint)
		# //print("ERROR:",error)
		self.current_time = rospy.get_time()
		delta_time = self.current_time - self.last_time
		#delta_time = 1.0 / self.frequency
		delta_error = error - self.last_error

		if (delta_time >= self.sample_time):
			self.PTerm = self.Kp * error

			if(abs(error)<0.005):
				self.ITerm=0
			self.ITerm += error * delta_time

			# if (self.ITerm < -self.windup_guard):
			#     self.ITerm = -self.windup_guard
			# elif (self.ITerm > self.windup_guard):
			#     self.ITerm = self.windup_guard

			# self.DTerm = 0.0
			if delta_time > 0:
				self.DTerm = delta_error / delta_time

			# Remember last time and last error for next calculation
			self.last_time = self.current_time
			self.last_error = error

			self.output = - self.PTerm + (self.Ki * self.ITerm) - (self.Kd * self.DTerm)

		return self.output
		# rospy.loginfo(self.output)

	def setKp(self, proportional_gain):
		"""Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
		self.Kp = proportional_gain

	def setKi(self, integral_gain):
		"""Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
		self.Ki = integral_gain

	def setKd(self, derivative_gain):
		"""Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
		self.Kd = derivative_gain

	def setWindup(self, windup):
		"""Integral windup, also known as integrator windup or reset windup,
		refers to the situation in a PID feedback controller where
		a large change in setpoint occurs (say a positive change)
		and the integral terms accumulates a significant error
		during the rise (windup), thus overshooting and continuing
		to increase as this accumulated error is unwound
		(offset by errors in the other direction).
		The specific problem is the excess overshooting.
		"""
		self.windup_guard = windup

	def setSampleTime(self, sample_time):
		"""PID that should be updated at a regular interval.
		Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
		"""
		self.sample_time = sample_time

class control_loop:

	

	def __init__(self):
		# TODO: modify these constants to make the car follow walls smoothly.
		self.KP = 3
		self.KI = 0.001
		self.KD = 0.8
		self.last_angle = 0.0
		self.last_vel = 0.5
		self.pid_last = 0.0
		self.msg = drive_param()

		self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
		self.pid = pid_controller()	

		self.max_rad_angle = 30*math.pi/180
		rospy.Subscriber("pid_error", Float64, self.control_callback)


	def calculate_angle(self,data,vel):
		self.pid.setKp(self.KP)
		self.pid.setKi(self.KI)
		self.pid.setKd(self.KD)
		self.pid.SetPoint = 0.0
		self.pid.setSampleTime = 1.0 / 40.0
		output_angle = self.pid.update(data.data)	
		output_angle = np.clip(output_angle,-self.max_rad_angle,self.max_rad_angle)
		# pid_now, delta_time= self.pid.update(data.data)	
		# print(pid_now,self.pid_last,delta_time,vel)
		# output_angle = math.acos((pid_now - self.pid_last)/delta_time/vel)

		# self.pid_last = pid_now
		return output_angle

	def calculate_vel(self,angle):
		vel = 0.0
		angle_d = abs(angle * 180 / math.pi)
		if (angle_d>=0) and (angle_d<10):
			vel = 1.5
		elif (angle_d>=10) and (angle_d<20):
			vel = 1.0
		else:
			vel = 0.5
		return vel


	# Callback for receiving PID error data on the /pid_error topic
	# data: the PID error from pid_error_node, published as a Float64
	def control_callback(self,data):
	  # TODO: Based on the error (data.data), determine the car's required velocity
	  # amd steering angle.
		# self.pid.setKp(self.KP)
		# self.pid.setKd(self.KD)

		self.msg.angle = self.calculate_angle(data,self.last_vel)   # TODO: implement PID for steering angle
		self.msg.velocity = self.calculate_vel(self.msg.angle)  # TODO: implement PID for velocity
	 	self.last_vel = self.msg.velocity
		self.last_angle = self.msg.angle

	def send_command(self):

		self.pub.publish(self.msg)
		#print("SENT")

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_controller_node', anonymous=True)
	C = control_loop()	
	r = rospy.Rate(40)

	while not rospy.is_shutdown():
		C.send_command()
		r.sleep()


