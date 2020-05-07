#!/usr/bin/env python
#May 2020
#Developer: Ryan Fainor 

# 3D IK NODE

import roslib
import rospy
roslib.load_manifest('openDog_description')
from std_msgs.msg import *
from numpy import *
import time


import IK_equations_3D


class servoPublisher():

  def __init__(self):

    self.dT = 0.005
    self.timenow = time.time()
    self.oldtime = self.timenow

    self.IK = IK_equations_3D.inverse_kinematics()

    # publish angles
    self.theta_f = rospy.get_param('~femur_angle', "/theta_f")
    self.theta_t = rospy.get_param('~tibia_angle', "/theta_t")
    self.theta_h = rospy.get_param('~hip_angle',"/theta_h")
    
    self.femur = rospy.Publisher(self.theta_f, Float64, queue_size = 1)
    self.tibia = rospy.Publisher(self.theta_t, Float64, queue_size = 1)
    self.hip = rospy.Publisher(self.theta_h, Float64, queue_size = 1)

    self.leg = rospy.get_param('~leg', "1")

    # create loop
    rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)



  def loop(self, event):

    self.timenow = time.time()
    self.oldtime = self.timenow

    myAngles = self.IK.JointAng(self.timenow, int(self.leg))
    
    self.hip.publish(myAngles[0])
    self.femur.publish(myAngles[1])
    self.tibia.publish(myAngles[2])


# main function

def main(args):
  rospy.init_node('servo_control_node', anonymous=True)
  myNode = servoPublisher()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
  main(sys.argv)
