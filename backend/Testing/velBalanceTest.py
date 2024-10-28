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

    # moves arms down from T-pose
    robot.motors[1].target = (math.radians(80), 'P') # for RightShoulderAbductor
    robot.motors[6].target = (math.radians(-80), 'P') # for LeftShoulderAbductor
    robot.moveAllToTarget()
    print("Initial Pose Done")

    # Initial IMU data
    imu_data_initial = robot.imu_manager.getAllIMUData()
    print("Initial IMU Readings:", imu_data_initial)
    prevIMU = imu_data_initial

    # creates trajectory of movements (squatting knees to 80 degrees)
    simStartTime = time.time()
    prevCoM = [0,0,0]
    setPoints = [[0,  0], [math.radians(80), math.radians(-80)], [math.radians(0), math.radians(0)]]
    tj = trajPlannerPose.TrajPlannerPose(setPoints)
    traj = tj.getCubicTraj(10, 100)
    notFalling = True
    count = 0  # Initialize outside of the stabilization loop for consistent counting


    #stabilizes itself before starting test
    while time.time() - simStartTime < 10:
        time.sleep(0.01)
        robot.updateRobotCoM()
        #prevCoP = robot.updateCoP()
        prevIMU = robot.imu_manager.getAllIMUData()
    print("Initialized")
    print("PrevIMU: ", prevIMU)
    # moves knees to each traj point until it falls
    while notFalling:
        for point in traj:
            #tells robot trajectory is specifically for arms
            robot.motors[0].target = (point[1], 'P') # for right arm
            robot.motors[5].target = (point[2], 'P') # for left arm
            robot.moveAllToTarget()

            time.sleep(0.01) 
            robot.IMUBalance(prevIMU) #where PID is used

            #robot.CoPBalance(prevCoP)
            
            imu_data = robot.imu_manager.getAllIMUData()
            print("IMU Readings at step {}: {}".format(count, imu_data))


            if abs(robot.CoM[0] - prevCoM[0]) > 15:  # Adjust threshold if needed
                robot.IMUBalance(prevIMU[0], prevIMU[2])
                print("Trying to Fix it")
                notFalling = False
                break
            
            #prevCoP = robot.feetCoP
            prevIMU = imu_data
            count = count + 1 # keeps track of how many trajectory points it has reached
            print("Count: ", count)
    print("Dead")
    print(count, " / ", len(traj)) # prints percentage of completion of balance test if falling was detected


if(__name__ == "__main__"):
    main()