# Recursive Newton-Euler algorithm 
 MatLab implementation of recursive Newton-Euler algorithm for robot dynamics. This script contains the implementation alongside with three examples how to use these functions. 
 ## Implementation details
 In order to use the functions correctly, one needs to define a data structure that describes the physical and geometrical parameters of the robot. 
 
 _**robot**_ struct contatins following parameters:
 - njoints - number of joints
 - type - array of 'R' or 'P' values describing the type of a joint, starting from the first one
 - mass - array of masses of each link, starting from the first one
 - length - array of lengthes of each link, starting from the first one
 - g - three-dimensional array describing the axis where the gravity acts 
 - R - cell array of rotation matrices from frame **{i-1}** to **{i}** starting from $R^0_1$
 - inertia - cell array of 3x3 inertia matrices of each link, starting from the first one.

_**RNE**_ function takes as arguments following values:
 - q - array of joint values
 - qdot - array of joint velocities
 - q2dot - array of joint accelerations
 - axis - 3x1 unit vector that describes alongside what axis the position vector from origin of {i} to origin of {i-1}
 
 ## First example
 A planar 2R robot is given with the following parameters:
 - q = [90, 45]
 - qdot= [-0.8, 0.35]
 - q2dot = [-0.4, 0.1]
 
 <img src="images/2R_ex1.jpg" alt="2R" width="400" height = "400" allign="center"/>*2R robot with frames attached*


 

