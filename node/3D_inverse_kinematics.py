#!/usr/bin/env python
#May 2020
#Developer: Ryan Fainor 

# 3D IK NODE

import roslib
import rospy
roslib.load_manifest('openDog_description')
from std_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from numpy import *
import time

import IK_equations_3D


class angle_publisher():

  def __init__(self):

    self.dT = 0.05
    self.timenow = time.time()
    self.oldtime = self.timenow

    self.IK = IK_equations_3D.inverse_kinematics()

    # publish angles
    self.theta_f = rospy.get_param('~femur_angle', "/theta_f")
    self.theta_t = rospy.get_param('~tibia_angle', "/theta_t")
    self.theta_h = rospy.get_param('~hip_angle', "/theta_h")
    self.foot_position = rospy.get_param('~foot_position', "/footPosition")

    self.phase_shift = rospy.get_param('~phase_shift', 0.0)
    self.x_center = rospy.get_param('~x_center', 0.0)
    self.x_stride = rospy.get_param('~x_stride', 0.0)
    self.y_center = rospy.get_param('~y_center', 0.0)
    self.y_offset = rospy.get_param('~y_offset', 0.0)
    self.z_center = rospy.get_param('~z_center', -25.0)
    self.z_lift = rospy.get_param('~z_lift', 3.0)
    self.leg_pace = rospy.get_param('~leg_pace', 0.0) # pace of gait
    
    self.foot_pub = rospy.Publisher(self.foot_position, PoseStamped, queue_size = 1)
    self.femur = rospy.Publisher(self.theta_f, Float64, queue_size = 1)
    self.tibia = rospy.Publisher(self.theta_t, Float64, queue_size = 1)
    self.hip = rospy.Publisher(self.theta_h, Float64, queue_size = 1)

    self.leg = rospy.get_param('~leg', 1)

    # create loop
    rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)


  def loop(self, event):

    self.timenow = time.time()
    self.oldtime = self.timenow

    myAngles, foot_xyz = self.IK.JointAng(self.timenow, self.leg, self.phase_shift, self.x_center, self.x_stride, \
                                self.y_center, self.y_offset, self.z_center, self.z_lift, self.leg_pace)
    
    # publish foot path
    footPosition = PoseStamped()
    footPosition.header.stamp = rospy.Time.now()
    footPosition.pose.position.x = foot_xyz[0]
    footPosition.pose.position.y = foot_xyz[1]
    footPosition.pose.position.z = foot_xyz[2]
    footPosition.pose.orientation.x = 0.0
    footPosition.pose.orientation.y = 0.0
    footPosition.pose.orientation.z = 0.0
    footPosition.pose.orientation.w = 0.0
    self.foot_pub.publish(footPosition)

    # publish joint angles
    self.hip.publish(myAngles[0])
    self.femur.publish(myAngles[1])
    self.tibia.publish(myAngles[2])


# main function

def main(args):
  rospy.init_node('inverse_kinematics', anonymous=True)
  myNode = angle_publisher()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
  main(sys.argv)
