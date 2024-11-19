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
    robot.motors[10].target = (math.radians(2.5), 'P') # HipsFront2Back
    robot.motors[11].target = (math.radians(0), 'P') # TorsoSide2Side
    robot.motors[12].target = (math.radians(0), 'P') # HipsRotate
    robot.motors[13].target = (math.radians(0), 'P') # HipsSide2Side
    robot.motors[14].target = (math.radians(0), 'P') # TorsoFront2Back
    robot.moveAllToTarget()
    print("Initial Pose Done")

    # creates trajectory of movements (squatting knees to 80 degrees)
    simStartTime = time.time()
    createSetUpPoints = [[0, 0, 0], [math.radians(-45), math.radians(-80), math.radians(180)]]
    setUpPoints = trajPlannerPose.TrajPlannerPose(createSetUpPoints)
    setUp = setUpPoints.getCubicTraj(1, 100)


    #stabilizes itself before starting test
    while time.time() - simStartTime < 7:
        time.sleep(0.01)
    print("Initialized")

    # Set initial balance targets
    initial = robot.fuse_imu_data()
    prevX = initial[0]
    prevY = initial[1]
    prevZ = initial[2]

    for point in setUp:
        #tells robot trajectory is specifically for arms
        robot.motors[1].target = (point[1], 'P') # for right arm
        robot.motors[6].target = (point[2], 'P') # for left arm
        robot.motors[2].target = (point[3], 'P')
        robot.moveAllToTarget()
        robot.IMUBalance(prevX, prevY, prevZ)


    createWavePoints = [[math.radians(-45)], [math.radians(-70)], [math.radians(-45)]]
    wavePoints = trajPlannerPose.TrajPlannerPose(createWavePoints)
    wave = wavePoints.getCubicTraj(0.05, 100)
    while True:
        for point in wave:
            #tells robot trajectory is specifically for arms
            robot.motors[1].target = (point[1], 'P') # for right arm
            robot.moveAllToTarget()

            robot.IMUBalance(prevX, prevY, prevZ)
        print("Next loop starting")






if(__name__ == "__main__"):
    main()