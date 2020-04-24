#!/usr/bin/env python

#Author: Sanha Lee, Will Pivik
#Date: April 23 2020

from numpy import *
import rospy 
import sys
import roslib
roslib.load_manifest('openDog_description')

import tf.transformations
import tf_conversions
import tf2_ros

# Imports message types and services from several libraries
from std_msgs.msg import Float64  #,Float64Stamped, Int32Stamped
from geometry_msgs.msg import TwistStamped, TransformStamped, Pose, PoseStamped
import std_srvs.srv

import time
from numpy import *
import traceback
import Queue   # might not be needed
# from matplotlib.pyplot import *

# -----------------------
# INVERSE KINEMATICS: 3-D
# -----------------------

class inverse_kinematics():

	def __init__(self):

		# robot dimensions

		self.lf = 14.125 # femur, inches
		self.lt = 13.25 # tibia, inches
		self.ls = 1.40 # shoulder offset, inches

		self.find_xyz('stand', 'left', 0)
        
        self.theta_f = rospy.get_param('~femur_angle', "/theta_f")
		self.theta_t = rospy.get_param('~tibia_angle', "/theta_t")
		self.theta_h = rospy.get_param('~hip_angle',"/theta_h")
		self.foot_position = rospy.get_param('~foot_position', "/footPosition_1")

        self.sub = rospy.Subscriber(self.foot_position, PoseStamped, self.pos_callback)

        self.theta_f = rospy.get_param('~femur_angle', "/theta_f")
		self.theta_t = rospy.get_param('~tibia_angle', "/theta_t")
		self.theta_h = rospy.get_param('~hip_angle',"/theta_h")
		self.foot_position = rospy.get_param('~foot_position', "/footPosition_1")


        self.femur = rospy.Publisher(self.theta_f, Float64, queue_size = 1)
		self.tibia = rospy.Publisher(self.theta_t, Float64, queue_size = 1)
		self.hip = rospy.Publisher(self.theta_h, Float64, queue_size = 1)

        self.theta_f = None    # angle of femur, from femur ball screw to hip constraint
		self.theta_t = None    # angle of tibia, from tibia ball screw to knee constraint
		self.theta_h = None	   # angle of hip, from

    def pos_callback(self, data):

        self.d = sqrt(data.pose.position.x**2 + data.pose.position.y**2)
		self.theta_P = pi - arctan2(data.pose.position.y, data.pose.position.x)
		print(self.theta_P)

		# Check to see if the point and angle are within the allowed range of motion
		if(self.d > self.R):
			print("Too far, can't reach that!")
			rospy.logwarn("Too far, can't reach that!")
		elif(self.d < self.r):
			print("Too close, can't reach that either")
			rospy.logwarn("Too close, can't reach that either")
		# elif(self.theta_P > self.theta_max_E):
		# 	print("Angle is too large")
		# elif(self.theta_P < self.theta_min_C):
		# 	print("Angle is too small")
		else:
	

            if (y<0):
                Adxy = arctan(z/y)
            else:
                Adxy = pi + arctan(z/y)

            dxy = sqrt(y**2 + z**2)
            As = Adxy - arccos(self.ls/dxy)

                if (x<0):
                    Ad = pi + arctan((z+self.ls*sin(As))/x)
                else:
                    Ad = arctan((z+self.ls*sin(As))/x)

                d = sqrt(x**2 + (z+self.ls*sin(As))**2)
                Af = Ad - arccos((self.lf**2 + d**2 - self.lt**2)/(2*self.lf*d))
                At = pi - arccos((self.lf**2 + self.lt**2 - d**2)/(2*self.lf*self.lt))

                Af = pi-Af
                At = -At

		return As,Af,At #set up publisher
