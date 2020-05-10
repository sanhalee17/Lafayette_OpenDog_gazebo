  
#!/usr/bin/env python

# Author: Sanha Lee Will Pivik, May 2020
# File: marker.py
# This node subscribes to inverse kinematics and publishes joint topic to openDog Gazebo file

#importing packages
import rospy
import sys
import roslib
import tf.transformations
import tf_conversions
import tf2_ros
roslib.load_manifest('openDog_description')
# Imports message types and services from several libraries
from std_msgs.msg import  Float64   
from geometry_msgs.msg import TwistStamped, TransformStamped, Pose
import std_srvs.srv
import time
from numpy import *
import traceback
import Queue
#import Marker
from visualization_msgs.msg import MarkerArray, Marker


print("initialized")

class MarkerClass:

    def __init__(self):
        

        print("init line 0")
        self.theta_f = rospy.get_param('~femur_angle', "/theta_f_1")
        print("init line 1")
        self.theta_t = rospy.get_param('~tibia_angle', "/theta_t_1")
        print("init line 2")
        self.sub_T = rospy.Subscriber(self.theta_t, Float64, self.thetat)
        self.sub_F = rospy.Subscriber(self.theta_f, Float64, self.thetaf)
        
        print("init line 3")
        
        
        #print("init line 4")
        self.markerArray = MarkerArray()
        self.markerArray.markers.append(Marker())
        #print("init line 4.5")
        self.pub_array = rospy.Publisher('visualizationMarkerArray', MarkerArray)
        #print("init line 5")
        #print("went through all init")


    def thetat(self,data):
        self.theta_t = Float64()
        self.theta_t = data.data
        print("tibia callback triggered")

    def thetaf(self,data):
        self.theta_f = Float64()
        self.theta_f = data.data


        rospy.logwarn("femur callback triggered")
            
        #print("tibia: "+  str(self.theta_t) + "," + "femur: " + str(self.theta_f))
        self.y_foot = .0254*(19.5*sin(self.theta_f)+16*sin((pi-self.theta_t)-self.theta_f))
        self.x_foot = .0254*(19.5*cos(self.theta_f)-16*cos((pi-self.theta_t)-self.theta_f))
    

        #rospy.init_node('marker')

            

        # while not rospy.is_shutdown():

        # ... here I get the data I want to plot into a vector called trans
            
        marker = Marker()
        marker.header.frame_id = "/openDog_body"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.x_foot
        marker.pose.position.y = self.y_foot
        marker.pose.position.z = 0
            
    # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
    # if(self.count > MARKERS_MAX):
        # self.markerArray.markers.push(0)
    # else:
    #        self.count += 1
        self.markerArray.markers.pop(0)
        self.markerArray.markers.append(marker)


    # Publish the MarkerArray
        rospy.logwarn("about to publish!!!")
        self.pub_array.publish(self.markerArray) 

def main(args):
    rospy.init_node('Marker',anonymous=True)
    print('asdf')  
    MK = MarkerClass()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"

if __name__=='__main__':
    main(sys.argv)
