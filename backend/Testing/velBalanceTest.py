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

    # moves arms down
    robot.motors[1].target = (math.radians(80), 'P') # for RightShoulderAbductor
    robot.motors[6].target = (math.radians(-80), 'P') # for LeftShoulderAbductor

    robot.moveAllToTarget()
    print("Initial Pose Done")

    # creates trajectory of movements
    simStartTime = time.time()
    prevCoM = [0,0,0]
    setPoints = [[0,  0], [math.radians(80), math.radians(-80)], [math.radians(0), math.radians(0)]]
    tj = trajPlannerPose.TrajPlannerPose(setPoints)
    traj = tj.getCubicTraj(10, 100)
    notFalling = True
    
    #stabilizes itself before starting test
    while time.time() - simStartTime < 10:
        time.sleep(0.01)
        robot.updateRobotCoM()
        prevCoM = robot.CoM
        count = 0
    print("Initialized")
    # moves arm to each traj point until it falls
    while notFalling:
        for point in traj:
            #tells robot trajectory is specifically for arms
            robot.motors[0].target = (point[1], 'P') # for shoulder rotators
            robot.motors[5].target = (point[2], 'P') # for shoulder rotators
            robot.moveAllToTarget()

            time.sleep(0.01)
            robot.updateRobotCoM()
            print("CoM: ", robot.CoM)
            robot.VelBalance(prevCoM)

            if abs(robot.CoM[0] - prevCoM[0]) > 10 or abs(robot.CoM[1] - prevCoM[1]) > 10:
                print("CoM: ", robot.CoM)
                print("FALLING")
                notFalling = False
                break
            prevCoM = robot.CoM
            count = count + 1
                #abs(robot.CoM[0] - prevCoM[0]) > 10 or 
    print("Dead")
    print(count, " / ", len(traj))
    time.sleep(1)
    robot.updateRobotCoM()
    print("CoM: ", robot.CoM)


if(__name__ == "__main__"):
    main()