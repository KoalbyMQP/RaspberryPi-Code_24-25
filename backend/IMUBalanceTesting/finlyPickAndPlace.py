import sys, time, math

import numpy as np 
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
from backend.Testing import finlyViaPoints as via

# Edit to declare if you are testing the sim or the real robot
is_real = False

robot = Robot(is_real)

print("Setup Complete")

#Starting Agnles
robot.motors[0].target = (math.radians(-20), 'P')
robot.motors[1].target = (math.radians(90), 'P')
robot.motors[3].target = (math.radians(110), 'P')

robot.motors[5].target = (math.radians(20), 'P')
robot.motors[6].target = (math.radians(-90), 'P')
robot.motors[8].target = (math.radians(-110), 'P')

prevTime = time.time()
simStartTime = time.time()

while time.time() - simStartTime < 2:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()

## EVEN TO RIGHT FOOT FORWARD
# rArm_tj = TrajPlannerTime(via.ra_grabCart[0], via.ra_grabCart[1], via.ra_grabCart[2], via.ra_grabCart[3])
lArm_tj = TrajPlannerTime(via.leftArmTraj[0], via.leftArmTraj[1], via.leftArmTraj[2], via.leftArmTraj[3])

state = 0
startTime = time.time()

print("State = 0")

## Grabbing cart
while time.time() - startTime < 8:
    l_points = lArm_tj.getQuinticPositions(time.time() - startTime)
    
    robot.motors[5].target = (l_points[0], 'P')
    robot.motors[6].target = (l_points[1], 'P')
    robot.motors[7].target = (l_points[2], 'P')
    robot.motors[8].target = (l_points[3], 'P')
    robot.motors[9].target = (l_points[4], 'P')

    robot.IMUBalance(0, 0)
    robot.moveAllToTarget()
