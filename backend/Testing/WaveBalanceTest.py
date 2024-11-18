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


def main():
    robot.motors[1].target = (math.radians(0), 'P')  # RightShoulderAbductor
    robot.motors[6].target = (math.radians(0), 'P') # LeftShoulderAbductor
    robot.moveAllToTarget()
    print("Initial Pose Done")

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

    # Set initial balance targets
    imu_data = robot.imu_manager.getAllIMUData()
    right_chest_imu = imu_data["RightChest"]
    left_chest_imu = imu_data["LeftChest"]
    torso_imu = imu_data["Torso"]
    initial = robot.fuse_imu_data(right_chest_imu, left_chest_imu, torso_imu)
    prevX = initial[0]
    prevY = initial[1]
    prevZ = initial[2]

    for point in setUp:
        #tells robot trajectory is specifically for arms
        robot.motors[1].target = (point[1], 'P') # for right arm
        robot.motors[6].target = (point[2], 'P') # for left arm
        robot.motors[2].target = (point[3], 'P')
        robot.moveAllToTarget()        
        


    createWavePoints = [[math.radians(-45)], [math.radians(-70)], [math.radians(-45)]]
    wavePoints = trajPlannerPose.TrajPlannerPose(createWavePoints)
    wave = wavePoints.getCubicTraj(0.05, 100)
    count = 0  # Initialize outside of the stabilization loop for consistent counting
    while True:
        for point in wave:
            #tells robot trajectory is specifically for arms
            robot.motors[1].target = (point[1], 'P') # for right arm
            robot.moveAllToTarget()

            robot.IMUBalance(prevX, prevY, prevZ)

            count = count + 1 # keeps track of how many trajectory points it has reached
            print(count, " / ", len(wave))
        count = 0  # Initialize outside of the stabilization loop for consistent counting






if(__name__ == "__main__"):
    main()