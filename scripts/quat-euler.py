#!/usr/bin/env python

#quarternions to euler angle conversion 
import rospy
import math 
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from std_msgs.msg import Float64
import pdb
import csv
from tf.msg import tfMessage
import tf


# You can define constants in Python as uppercase global names like these.
x=0
y=0
x_old=0
y_old=0
quaternion=[]
def Position_change(data):
    qx=data.pose.pose.orientation.x
    qy=data.pose.pose.orientation.y
    qz=data.pose.pose.orientation.z
    qw=data.pose.pose.orientation.w
    quaternion = (qx,qy,qw,qz)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = 180*euler[2]/math.pi
    #head=180*head/math.pi
    print(yaw)
    #euler = data.tf.transformations.euler_from_quaternion(x_q,y_q,z_q,w_q)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]
    # # global x
	# global y
	# global i
	# global x_old
	# global y_oldn
	# x=data.pose.pose.position.x 
	# y=data.pose.pose.position.y
	# print(y)

	# if(i==10):
	# 	i=0
	# 	x_old=x
	# 	y_old=y

	# i=i+1

if __name__ == '__main__':
  rospy.init_node('pid_error_node', anonymous = True)
  rospy.Subscriber('/vesc/odom',Odometry,Position_change)
  rospy.spin()
