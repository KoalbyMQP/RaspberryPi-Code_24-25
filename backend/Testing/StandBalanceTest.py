import sys, time, math

import numpy as np 
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid import trajPlannerPose
from backend.KoalbyHumanoid.Config import Joints
import matplotlib.pyplot as plt
from backend.KoalbyHumanoid.Plotter import Plotter

# Edit to declare if you are testing the sim or the real robot
is_real = False
robot = Robot(is_real)
print("Setup Complete")



def initialize():
    robot.motors[1].target = (math.radians(80), 'P')  # RightShoulderAbductor
    robot.motors[6].target = (math.radians(-80), 'P') # LeftShoulderAbductor

    # Torso
    robot.motors[10].target = (math.radians(3), 'P')
    robot.motors[11].target = (math.radians(0), 'P')
    robot.motors[12].target = (math.radians(0), 'P')
    robot.motors[13].target = (math.radians(0), 'P')
    robot.motors[14].target = (math.radians(0), 'P')

    # Right Leg
    robot.motors[15].target = (0, 'P')
    robot.motors[16].target = (0, 'P')
    robot.motors[17].target = (0, 'P')
    robot.motors[18].target = (0, 'P')
    robot.motors[19].target = (0, 'P')

    # Left Leg
    robot.motors[20].target = (0, 'P')
    robot.motors[21].target = (0, 'P')
    robot.motors[22].target = (0, 'P')
    robot.motors[23].target = (0, 'P')
    robot.motors[24].target = (0, 'P')


    robot.moveAllToTarget()
    print("Initial Pose Done")
    


def main():
    initialize()

    # Set initial balance targets
    imu_data = robot.imu_manager.getAllIMUData()
    right_chest_imu = imu_data[0]
    left_chest_imu = imu_data[1]
    #torso_imu = imu_data["Torso"]
    initial = robot.fuse_imu_data(right_chest_imu, left_chest_imu)
    prevX = initial[0]
    prevY = initial[1]
    prevZ = initial[2]

    # creates trajectory of movements (squatting knees to 80 degrees)
    simStartTime = time.time()
    createSetUpPoints = [[0, 0, 0], [math.radians(-45), math.radians(-80), math.radians(180)]]
    setUpPoints = trajPlannerPose.TrajPlannerPose(createSetUpPoints)
    setUp = setUpPoints.getCubicTraj(1, 100)
    count = 0  # Initialize outside of the stabilization loop for consistent counting


    #stabilizes itself before starting test
    while time.time() - simStartTime < 7:
        time.sleep(0.01)
    print("Initialized")

    count = 0 

    while True:
        newTargetX = robot.IMUBalance(prevX, prevY, prevZ)[0]
        newTargetY = robot.IMUBalance(prevX, prevY, prevZ)[1]
        newTargetZ = robot.IMUBalance(prevX, prevY, prevZ)[2]

        robot.motors[12].target = (newTargetZ, 'P')  # Adjust yaw
        robot.motors[13].target = (newTargetY, 'P')  # Adjust pitch
        robot.motors[10].target = (-newTargetX, 'P')  # Adjust roll

        # Right Leg
        robot.motors[15].target = (0, 'P')
        robot.motors[16].target = (0, 'P')
        robot.motors[17].target = (0, 'P')
        robot.motors[18].target = (0, 'P')
        robot.motors[19].target = (0, 'P')

        # Left Leg
        robot.motors[20].target = (0, 'P')
        robot.motors[21].target = (0, 'P')
        robot.motors[22].target = (0, 'P') 
        robot.motors[23].target = (0, 'P') 
        robot.motors[24].target = (0, 'P')


        robot.moveAllToTarget()


        count = count + 1 # keeps track of how many trajectory points it has reached
        # print(count, " / ", len(wave))

if(__name__ == "__main__"):
    main()