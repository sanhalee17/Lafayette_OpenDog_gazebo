#!/usr/bin/env python
#May 2020
#Developer: Ryan Fainor

from numpy import *

# -----------------------
# INVERSE KINEMATICS: 3-D
# -----------------------

class inverse_kinematics():

    def __init__(self):

		# robot dimensions

        self.lf = 0.39 # femur, inches
        self.lt = 0.31 # tibia, inches
        self.ls = 0.165 # shoulder offset, inches
        
        foot_xyz = self.find_xyz(0,1,0,0,0,0,0,0,0,1)

    def find_xyz(self, time, leg, phase_shift, x_center, x_stride, y_center, y_offset, z_center, z_lift, leg_pace):

        self.t = time
        self.leg = leg

        if self.leg == 1:
            leg1_offset = 0			# front left
            self.x = x_center + x_stride*sin(leg_pace*self.t - pi/2 - leg1_offset)
            self.y = y_center + y_offset*sin(leg_pace*self.t - pi - leg1_offset)
            self.z = z_center + z_lift*sin(leg_pace*self.t - leg1_offset)

        if self.leg == 2:
            leg2_offset = pi		# front right
            self.x = x_center + x_stride*sin(leg_pace*self.t - pi/2 - leg2_offset)
            self.y = y_center + y_offset*sin(leg_pace*self.t - pi - leg2_offset)
            self.z = z_center + z_lift*sin(leg_pace*self.t - leg2_offset)
 
        if self.leg == 3:
            leg3_offset = pi		# back left
            self.x = x_center + x_stride*sin(leg_pace*self.t - pi/2 - leg3_offset)
            self.y = y_center + y_offset*sin(leg_pace*self.t - pi - leg3_offset)
            self.z = z_center + z_lift*sin(leg_pace*self.t - leg3_offset)
            
        if self.leg == 4:
            leg4_offset = 0 		# back right
            self.x = x_center + x_stride*sin(leg_pace*self.t - pi/2 - leg4_offset)
            self.y = y_center + y_offset*sin(leg_pace*self.t - pi - leg4_offset)
            self.z = z_center + z_lift*sin(leg_pace*self.t - leg4_offset)

        if (self.z) < z_center: self.z = z_center
 

    def JointAng(self, time, leg, phase_shift, x_center, x_stride, y_center, y_offset, z_center, z_lift, leg_pace):

        self.find_xyz(time, leg, phase_shift, x_center, x_stride, y_center, y_offset, z_center, z_lift, leg_pace)

        self.angs, self.angf, self.angt = self.getJointAng(self.x, self.y, self.z, leg)

        myAngles = [self.angs, self.angf, self.angt]
        foot_xyz = [self.x, self.y, self.z]

        return myAngles, foot_xyz


    def getJointAng(self, x, y, z, leg):

        if (y<0):
            Adxy = arctan(z/y)
        elif (y==0):
            Adxy = pi/2
        else:
            Adxy = pi + arctan(z/y)

        dxy = sqrt(y**2 + z**2)
        As = Adxy - arccos(self.ls/dxy)

        

        if (leg == 1 or leg == 3):
            As = As

            if (x<0):
                Ad = pi + arctan((z+self.ls*sin(As))/x)
            elif (x==0):
                Ad = pi+pi/2
            else:
                Ad = arctan((z+self.ls*sin(As))/x)

            d = sqrt(x**2 + (z+self.ls*sin(As))**2)
            Af = Ad - arccos((self.lf**2 + d**2 - self.lt**2)/(2*self.lf*d))
            At = pi - arccos((self.lf**2 + self.lt**2 - d**2)/(2*self.lf*self.lt))

            Af = pi + Af
            At = At

        else:
            if (x<0):
                Ad = arctan((z+self.ls*sin(As))/x)
            elif (x==0):
                Ad = pi/2
            else:
                Ad = pi + arctan((z+self.ls*sin(As))/x)

            d = sqrt(x**2 + (z+self.ls*sin(As))**2)
            Af = Ad - arccos((self.lf**2 + d**2 - self.lt**2)/(2*self.lf*d))
            At = pi - arccos((self.lf**2 + self.lt**2 - d**2)/(2*self.lf*self.lt))


        return As,Af,At

