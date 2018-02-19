# Created on Fri Feb 16 20:13:27 2018
# Authors: Chaitanya Perugu, Ashwin Sahasrabudhe

import mykdl as mk

baxter = mk.Robot()
baxfk = mk.FKSolver(baxter)

# Forward Pose Kinematics
print "Pose 1: Transformation matrix"
joint_pose = {'s0':0, 's1':0, 'e0':0, 'e1':0, 'w0':0, 'w1':0, 'w2':0}
T = baxfk.solveFK(joint_pose)
print T

print "Pose 2: Transformation matrix"
joint_pose = {'s0':60, 's1':20, 'e0':10, 'e1':0, 'w0':-10, 'w1':-20, 'w2':-30}
T = baxfk.solveFK(joint_pose)
print T

# Quaternion Euler Conversion
quat = mk.Quaternion((0, 1, 0, 0))
euler = quat.q2e()

print "Quaternion = {}".format(quat.value())
print "Euler angles = "
euler.display()