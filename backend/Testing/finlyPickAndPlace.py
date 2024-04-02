import sys, time, math

import numpy as np 
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
# from backend.Testing import finlyViaPoints as via

# Edit to declare if you are testing the sim or the real robot
is_real = False

robot = Robot(is_real)

print("Setup Complete")

################################################################
"""
FOR DESIGN TEAM

This file is to log the via points of the trajectories for the demo in effort to clean up the main code.
Angles are in radians and can be calculated using the MATLAB scripts found in backend>KoalbyHumanoid>MATLAB Scripts.

Format of each list goes as follows:
    [0]: list of t_f for each via point
    [1]: list of joint angles (rad)
    [2]: list of joint velocities through each via_point (v_f)
    [3]: list of joint accelerations through each via point (a_f)
"""

##first movement to grab the candy
leftArmTraj1 = [
    [[0,0,0,0,0], [3,3,3,3,3], [6,6,6,6,6]],
    [[0.349066, -1.570796, 0.000000, -1.919862, 0.000000], ##starting position
     [0.550746, -0.650519, 0.457600, -1.822806, -0.996616], ## X125, Y50, Z5
     [0.364398, -0.801358, 0.274735, -1.428318, -0.804733]], 
     [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]],
     [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]]

##second movement to drop the candy
leftArmTraj2 = [
    [[0,0,0,0,0],[3,3,3,3,3],[6,6,6,6,6]],
     [[0.317468, -0.741816, 0.250788, -1.431064, -0.855117],
     [0.296623, -1.474594, 0.029807, -1.803107, -0.101194],
     [0.349066, -1.570796, 0.000000, -1.919862, 0.000000]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]]
###############################


#Starting Agnles
robot.motors[0].target = (math.radians(-20), 'P')
robot.motors[1].target = (math.radians(90), 'P')
robot.motors[3].target = (math.radians(110), 'P')

robot.motors[5].target = (math.radians(20), 'P')
robot.motors[6].target = (math.radians(-90), 'P')
robot.motors[8].target = (math.radians(-110), 'P')

robot.motors[18].target = (math.radians(-90), 'P')
robot.motors[17].target = (math.radians(90), 'P')
robot.motors[23].target = (math.radians(90), 'P')
robot.motors[22].target = (math.radians(-90), 'P')

robot.motors[5].target = (0.349066, 'P')
robot.motors[6].target = (-1.570796, 'P')
robot.motors[7].target = (0.000000, 'P')
robot.motors[8].target = (-1.919862, 'P')
robot.motors[9].target = (0.000000, 'P')

prevTime = time.time()
simStartTime = time.time()

while time.time() - simStartTime < 5:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()

## EVEN TO RIGHT FOOT FORWARD
# rArm_tj = TrajPlannerTime(via.ra_grabCart[0], via.ra_grabCart[1], via.ra_grabCart[2], via.ra_grabCart[3])
lArm_tj = TrajPlannerTime(leftArmTraj1[0], leftArmTraj1[1], leftArmTraj1[2], leftArmTraj1[3])

state = 0
startTime = time.time()

print("State = 0")

## Grabbing cart
while time.time() - startTime < 6:
    l_points = lArm_tj.getQuinticPositions(time.time() - startTime)
    
    robot.motors[5].target = (l_points[0], 'P')
    robot.motors[6].target = (l_points[1], 'P')
    robot.motors[7].target = (l_points[2], 'P')
    robot.motors[8].target = (l_points[3], 'P')
    robot.motors[9].target = (l_points[4], 'P')

    robot.IMUBalance(0, 0)
    robot.moveAllToTarget()

## grab object

lArm_tj = TrajPlannerTime(leftArmTraj2[0], leftArmTraj2[1], leftArmTraj2[2], leftArmTraj2[3])
startTime = time.time()
print("State = 1")

while time.time() - startTime < 6:
    l_points = lArm_tj.getQuinticPositions(time.time() - startTime)
    
    robot.motors[5].target = (l_points[0], 'P')
    robot.motors[6].target = (l_points[1], 'P')
    robot.motors[7].target = (l_points[2], 'P')
    robot.motors[8].target = (l_points[3], 'P')
    robot.motors[9].target = (l_points[4], 'P')

    robot.IMUBalance(0, 0)
    robot.moveAllToTarget()
##while loop for traj
