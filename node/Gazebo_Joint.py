#!/usr/bin/env python

# Author: Sanha Lee, April 2020
# File: gazebo_joint.py
# This node subscribes to inverse kinematics and publishes joint topic to openDog Gazebo file


#importing packages
import rospy
import sys
import roslib
import tf.transformations
import tf_conversions
import tf2_ros

# Imports message types and services from several libraries
from std_msgs.msg import  Float64, Int32, Bool   #,Float64Stamped, Int32Stamped,
from geometry_msgs.msg import TwistStamped, TransformStamped, Pose #PoseStamped
import std_srvs.srv

roslib.load_manifest('openDog_description')

class GazeboJoint:

	def __init__(self):

		self.sub_F = rospy.Subscriber(self.theta_f, Float64, self.femur_joint_callback)
		self.sub_T = rospy.Subscriber(self.theta_t, Float64, self.tibia_joint_callback)



		self.femur_joint_callback(self, data)

			self.theta_f = data.data


		self. tibia_joint_callback(self, data)

			self.theta_t = data.data

			#self.joint1_position_controller/command = 


def main(args):
	rospy.init_node('gazebo_joint',anonymous1=True)
	GJ = GazeboJoint()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "shutting down"

if __name__=='__main__':
	main(sys.argv)