#!/usr/bin/env python

# Author: Sanha Lee, May 2020
# File: 3Dgazebo_joint.py
# This node subscribes to 3D inverse kinematics and publishes joint topics to the openDog Gazebo file


import rospy
import sys
import roslib
import tf.transformations
import tf_conversions
import tf2_ros


from std_msgs.msg import  Float64   
from geometry_msgs.msg import TwistStamped, TransformStamped, Pose
import std_srvs.srv
import time
from numpy import *
import traceback
import Queue

roslib.load_manifest('openDog_description')

class GazeboJoint3D:

	def __init__(self):

		self.theta_f = rospy.get_param('~femur_angle', "/theta_f")
		self.theta_t = rospy.get_param('~tibia_angle', "/theta_t")
        self.theta_h = rospy.get_param('~hip_angle', '/theta_h_1')
		self.joint1_position_controller = rospy.get_param('~tibia_controller', "/joint1_position_controller/command")
		self.joint5_position_controller = rospy.get_param('~femur_controller', "/joint5_position_controller/command")
        self.joint9_position_controller = rospy.get_param('~hip_controller', "/joint9_position_controller/command")
		
		self.sub_F = rospy.Subscriber(self.theta_f, Float64, self.femur_joint_callback)
		self.sub_T = rospy.Subscriber(self.theta_t, Float64, self.tibia_joint_callback)
        self.sub_F = rospy.Subscriber(self.theta_h, Float64, self.hip_joint_callback)

		self.tibia_controller = rospy.Publisher(self.joint5_position_controller, Float64, queue_size = 1)
		self.femur_controller = rospy.Publisher(self.joint1_position_controller, Float64, queue_size = 1)
        self.hip_controller = rospy.Publisher(self.joint9_position_controller, Float64, queue_size = 1)


	def tibia_joint_callback(self, data):
		#rospy.logwarn('tibia callback triggered')
		self.theta_t = data.data
		self.joint5_position_controller = Float64()
		self.joint5_position_controller = self.theta_t
		#self.joint5.publish = (self.joint5_position_controller)
        #rospy.logwarn(str(self.theta_f))
		self.tibia_controller.publish(self.joint5_position_controller)
        #rospy.logwarn(str(self.joint5_position_controller))

	def femur_joint_callback(self, data):
		#rospy.logwarn('femur callback triggered')
		self.theta_f = data.data
		self.joint1_position_controller = Float64()
		self.joint1_position_controller = self.theta_f
		#self.joint1.publish = (self.joint1_position_controller)
        #rospy.logwarn(str(self.joint1_position_controller))
		self.femur_controller.publish(self.joint1_position_controller)

    def hip_joint_callback(self, data):
        self.theta_h = data.data
        self.joint9_position_controller = Float64()
        self.joint9_position_controller = self.theta_h
        self.hip_controller.publish(self.joint9_position_controller)
        


def main(args):
	rospy.init_node('gazebo_joint',anonymous=True)
	GJ = GazeboJoint3D()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)