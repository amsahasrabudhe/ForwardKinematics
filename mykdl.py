# Created on Sat Feb 10 14:51:38 2018
# Authors: Chaitanya Perugu, Ashwin Sahasrabudhe

import numpy as np
from mathfunctions import *

class FKSolver:
    def __init__(self, robot):
        self.robot = robot
        
    def DH2T(self, d, theta, a, alpha):
        theta = deg2rad(theta)
        alpha = deg2rad(alpha)
        
        RotZ = np.matrix([[np.cos(theta), -np.sin(theta), 0, 0], 
                          [np.sin(theta), np.cos(theta), 0, 0], 
                          [0, 0, 1, 0], 
                          [0, 0, 0, 1]])
                         
        TransZ = np.matrix([[1, 0, 0, 0], 
                            [0, 1, 0, 0], 
                            [0, 0, 1, d], 
                            [0, 0, 0, 1]])
                           
        TransX = np.matrix([[1, 0, 0, a], 
                            [0, 1, 0, 0], 
                            [0, 0, 1, 0], 
                            [0, 0, 0, 1]])
                           
        RotX = np.matrix([[1, 0, 0, 0], 
                          [0, np.cos(alpha), -np.sin(alpha), 0], 
                          [0, np.sin(alpha), np.cos(alpha), 0], 
                          [0, 0, 0, 1]])
        
        T = RotZ*TransZ*TransX*RotX
        return T
        
    def solveFK(self, joint_angles):
        DH = self.robot.DHref
        T = np.asmatrix(np.identity(4))
        
        if self.check_joint_limits(joint_angles):         
            for joint in self.robot.joint_keys:
                T = T*self.DH2T(DH[joint][0], DH[joint][1] + joint_angles[joint], 
                                DH[joint][2], DH[joint][3])
            return np.around(T, 5)
        else:
            print "Invalid FK Request!"
            return 0
        
    def check_joint_limits(self, joint_angles):
        for joint in joint_angles.keys():
            lower_bound_check = joint_angles[joint] >= self.robot.angle_limits[joint][0]
            upper_bound_check = joint_angles[joint] <= self.robot.angle_limits[joint][1]
            if not (lower_bound_check and upper_bound_check):
                print "{} joint limits exceeded!".format(joint)
                return False
            
        return True