import sys, time, math

import numpy as np 
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
from backend.Testing import assistWalkViaPoints as via
from coppeliasim_zmqremoteapi_client import RemoteAPIClient as sim

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

robot.motors[17].target = (math.radians(20), 'P')
robot.motors[18].target = (math.radians(-40), 'P')
robot.motors[19].target = (math.radians(-20), 'P')
robot.motors[22].target = (math.radians(-20), 'P')
robot.motors[23].target = (math.radians(40), 'P')
robot.motors[24].target = (math.radians(20), 'P')

simStartTime = time.time()

while time.time() - simStartTime < 4:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()

## EVEN TO RIGHT FOOT FORWARD
rLeg_tj = TrajPlannerTime(via.rf_Even2Right[0], via.rf_Even2Right[1], via.rf_Even2Right[2], via.rf_Even2Right[3])
lLeg_tj = TrajPlannerTime(via.lf_Even2Right[0], via.lf_Even2Right[1], via.lf_Even2Right[2], via.lf_Even2Right[3])
rArm_tj = TrajPlannerTime(via.ra_grabCart[0], via.ra_grabCart[1], via.ra_grabCart[2], via.ra_grabCart[3])
lArm_tj = TrajPlannerTime(via.la_grabCart[0], via.la_grabCart[1], via.la_grabCart[2], via.la_grabCart[3])

startTime = time.time()
## Grabbing cart
while time.time() - startTime < 3:
    r_points = rArm_tj.getQuinticPositions(time.time() - startTime)
    l_points = lArm_tj.getQuinticPositions(time.time() - startTime)

    robot.motors[0].target = (r_points[0], 'P')
    robot.motors[1].target = (r_points[1], 'P')
    robot.motors[2].target = (r_points[2], 'P')
    robot.motors[3].target = (r_points[3], 'P')
    robot.motors[4].target = (r_points[4], 'P')
    
    robot.motors[5].target = (l_points[0], 'P')
    robot.motors[6].target = (l_points[1], 'P')
    robot.motors[7].target = (l_points[2], 'P')
    robot.motors[8].target = (l_points[3], 'P')
    robot.motors[9].target = (l_points[4], 'P')

    robot.IMUBalance(0, 0)
    robot.moveAllToTarget()

cart = robot.sim.getObject('/Cart')
forceSensor = robot.sim.getObject('/ForceSensor')
robot.sim.setObjectParent(cart, forceSensor, True)
time.sleep(2)

##  Walking
startTime = time.time()
state = 0
while True:
    right_points = rLeg_tj.getQuinticPositions(time.time() - startTime)
    left_points = lLeg_tj.getQuinticPositions(time.time() - startTime)

    robot.motors[15].target = (right_points[0], 'P')
    robot.motors[16].target = (right_points[1], 'P')
    robot.motors[17].target = (right_points[2], 'P')
    robot.motors[18].target = (right_points[3], 'P')
    robot.motors[19].target = (right_points[4], 'P')

    robot.motors[20].target = (left_points[0], 'P')
    robot.motors[21].target = (left_points[1], 'P')
    robot.motors[22].target = (left_points[2], 'P')
    robot.motors[23].target = (left_points[3], 'P')
    robot.motors[24].target = (left_points[4], 'P')
    robot.IMUBalance(0, 0)
    robot.moveAllToTarget()
    match(state):
        case 0: 
            if(time.time() - startTime >= 3):
                ##ANGLES TO GO RIGHT TO LEFT
                rLeg_tj = TrajPlannerTime(via.rf_Right2Left[0], via.rf_Right2Left[1], via.rf_Right2Left[2], via.rf_Right2Left[3])
                lLeg_tj = TrajPlannerTime(via.lf_Right2Left[0], via.lf_Right2Left[1], via.lf_Right2Left[2], via.lf_Right2Left[3])
                startTime = time.time()
                print("Right To Left")
                state = 1
            
        case 1: ##Right to Left
            if(time.time() - startTime >= 3):
                ##ANGLES TO GO LEFT TO RIGHT
                rLeg_tj = TrajPlannerTime(via.rf_Left2Right[0], via.rf_Left2Right[1], via.rf_Left2Right[2], via.rf_Left2Right[3])
                lLeg_tj = TrajPlannerTime(via.lf_Left2Right[0], via.lf_Left2Right[1], via.lf_Left2Right[2], via.lf_Left2Right[3])
                print("Left To Right")
                startTime = time.time()
                state = 0