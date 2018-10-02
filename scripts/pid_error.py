#!/usr/bin/env python

import rospy
from math import cos, sin, atan, pi
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import pdb

pub = rospy.Publisher('pid_error', Float64, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0
a = 0.0
b = 0.0

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle, old_range):

  ranges = np.asarray(data.ranges)
  angle_index = (angle + 45) * 4
  if ranges[angle_index] == np.nan or ranges[angle_index] == np.inf:
    return old_range
  else:
    output_range = ranges[angle_index]
    return output_range

# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):
  global a,b
  L = 0.025
  desired_distance = desired_distance
  a = getRange(data, 135, a)
  b = getRange(data, 180, b)
  theta = 45 * pi / 180
  alpha = atan((a*cos(theta)-b)/(a*sin(theta)))
  current_dist  = b * cos(alpha)
  next_dist = current_dist + L * sin(alpha)
  error_t = desired_distance - current_dist

  # pass the error_t term into some function and output the next_angle and velocity
  return error_t

# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
  global a,b
  L = 0.025
  desired_distance = desired_distance
  a = getRange(data, 45, a)
  b = getRange(data, 0, b)
  theta = 45 * pi / 180
  alpha = atan((a*cos(theta)-b)/(a*sin(theta)))
  current_dist  = b * cos(alpha)
  next_dist = current_dist + L * sin(alpha)
  error_t = desired_distance - current_dist

  # TODO: implement
  return -error_t

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
  left_error = followLeft(data,1)
  right_error = followRight(data,1)
  center_error = (left_error + right_error)/2

  return center_error

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):
  desired_distance = 0.5
  error = followLeft(data,desired_distance) 
  # error = followRight(data,desired_distance) 
  # error = followCenter(data) 

  msg = Float64()
  msg.data = error
  pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
  rospy.init_node('pid_error_node', anonymous = True)
  rospy.Subscriber("scan", LaserScan, scan_callback)
  rospy.spin()
