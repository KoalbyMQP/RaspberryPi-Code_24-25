import sys, time, math

import numpy as np 
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
from backend.DemoScripts import assistWalkViaPoints as via
from coppeliasim_zmqremoteapi_client import RemoteAPIClient as sim
from backend.KoalbyHumanoid.Config import Joints
import copy

WAIST_OFFSET = 0 
ANKLE_OFFSET = -0 - WAIST_OFFSET
RIGHT_ROTATOR_OFFET = 5
LEFT_ROTATOR_OFFET = -5
LEFT_ABD_OFFSET = -3
RIGHT_ABD_OFFSET = 3

# Applies the given offset as a vector to the given trajectory
def applyOffsets(trajectory, offset):
    trajectory = copy.deepcopy(trajectory)

    for i in range(len(trajectory[1])):
        for j in range(len(trajectory[1][i])):
            trajectory[1][i][j] += math.radians(offset[j])

    return trajectory
    
# Creates and returns the trajectories for each of the joints
def createTrajectories(rightLeg, leftLeg, rightArm, leftArm):

    # Applies static offsets to joint poisitions (used for tweaking trajectories)
    rightOffset = [RIGHT_ABD_OFFSET, RIGHT_ROTATOR_OFFET, WAIST_OFFSET, 0, -ANKLE_OFFSET]
    leftOffset = [LEFT_ABD_OFFSET, LEFT_ROTATOR_OFFET, -WAIST_OFFSET, 0, ANKLE_OFFSET]
    rightArmOffset = [0, 0, 0, -5, -5]
    rightLeg = applyOffsets(rightLeg, rightOffset)
    leftLeg = applyOffsets(leftLeg, leftOffset)
    rightArm = applyOffsets(rightArm, rightArmOffset)

    # Generates trajectories
    rightLeg_tj = TrajPlannerTime(rightLeg[0], rightLeg[1], rightLeg[2], rightLeg[3])
    leftLeg_tj  = TrajPlannerTime(leftLeg[0],  leftLeg[1],  leftLeg[2],  leftLeg[3])
    rightArm_tj = TrajPlannerTime(rightArm[0], rightArm[1], rightArm[2], rightArm[3])
    leftArm_tj  = TrajPlannerTime(leftArm[0],  leftArm[1],  leftArm[2],  leftArm[3])
    return rightLeg_tj, leftLeg_tj, rightArm_tj, leftArm_tj

def main():
    # Edit to declare if you are testing the sim or the real robot
    is_real = False

    robot = Robot(is_real)
    print("Setup Complete") 


    robot.motors[17].target = (math.radians(20), 'P') 
    robot.motors[18].target = (math.radians(20), 'P') 
    robot.motors[19].target = (math.radians(-20), 'P') 

    robot.motors[22].target = (math.radians(-20), 'P') 
    robot.motors[23].target = (math.radians(-20), 'P') 
    robot.motors[24].target = (math.radians(20), 'P') 

    robot.motors[25].target = (math.radians(20), 'P') # Head

    simStartTime = time.time()

    [rLeg_tj, lLeg_tj, rArm_tj, lArm_tj] = createTrajectories(
        via.rf_Even2Right, 
        via.lf_Even2Right, 
        via.ra_leftDown, 
        via.la_rightDown )
    
    initialPositioning = True

    # input()

    ##  Walking
    startTime = time.time()
    state = 0

    while True:
        currentTime = time.time() - startTime
        right_points = rLeg_tj.getQuinticPositions(currentTime)
        left_points = lLeg_tj.getQuinticPositions(currentTime)

        rArm_points = rArm_tj.getQuinticPositions(currentTime)
        lArm_points = lArm_tj.getQuinticPositions(currentTime)
        
        # Right arm
        robot.motors[0].target = (math.radians(-20), 'P')
        robot.motors[1].target = (math.radians(-90), 'P') 
        robot. motors[3].target = (math.radians(90), 'P') #pos
        robot.motors[4].target = (math.radians(-15), 'P')

        # Left Arm
        robot.motors[5].target = (math.radians(20), 'P')
        robot.motors[6].target = (math.radians(90), 'P')
        robot.motors[8].target = (math.radians(-90), 'P')#neg
        robot.motors[9].target = (math.radians(15), 'P') 

        # Right Leg
        robot.motors[15].target = (right_points[0], 'P')
        robot.motors[16].target = (right_points[1], 'P')
        robot.motors[17].target = (right_points[2], 'P')
        robot.motors[18].target = (right_points[3], 'P')
        robot.motors[19].target = (right_points[4], 'P')

        # Left Leg
        robot.motors[20].target = (left_points[0], 'P')
        robot.motors[21].target = (left_points[1], 'P')
        robot.motors[22].target = (left_points[2], 'P')
        robot.motors[23].target = (left_points[3], 'P')
        robot.motors[24].target = (left_points[4], 'P') 

        # robot.IMUBalance(0, 0)
        robot.moveAllToTarget()
        robot.checkMotorsAtInterval(2)

        if(initialPositioning):
            input()
            initialPositioning = False
            startTime = time.time()

        if(time.time() - startTime >= 3):
            match(state):
                case 0: 
                    [rLeg_tj, lLeg_tj, rArm_tj, lArm_tj] = createTrajectories(
                        via.rf_Right2Left, 
                        via.lf_Right2Left, 
                        via.ra_leftDown, 
                        via.la_leftDown )

                    startTime = time.time()
                    # print("Right To Left")
                    state = 1
                    
                case 1: ##Right to Left
                    [rLeg_tj, lLeg_tj, rArm_tj, lArm_tj] = createTrajectories(
                        via.rf_Left2Right, 
                        via.lf_Left2Right, 
                        via.ra_rightDown, 
                        via.la_rightDown )
                    
                    # print("Left To Right")
                    startTime = time.time()
                    state = 0

if(__name__ == "__main__"):
    main()