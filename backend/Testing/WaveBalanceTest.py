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

    prevCoP = robot.updateCoP()
    print("CoP", prevCoP)

    # creates trajectory of movements (squatting knees to 80 degrees)
    simStartTime = time.time()
    createSetUpPoints = [[0, 0, 0], [math.radians(-45), math.radians(-80), math.radians(180)]]
    setUpPoints = trajPlannerPose.TrajPlannerPose(createSetUpPoints)
    setUp = setUpPoints.getCubicTraj(1, 100)
    count = 0  # Initialize outside of the stabilization loop for consistent counting


    #stabilizes itself before starting test
    while time.time() - simStartTime < 10:
        time.sleep(0.01)
        prevCoP = robot.updateCoP()
    print("Initialized")

    for point in setUp:
        #tells robot trajectory is specifically for arms
        robot.motors[1].target = (point[1], 'P') # for right arm
        robot.motors[6].target = (point[2], 'P') # for left arm
        robot.motors[2].target = (point[3], 'P')
        robot.moveAllToTarget()

        time.sleep(0.01) 
        # robot.IMUBalance(prevIMU) #where PID is used
        robot.CoPBalance(prevCoP)
        
        pressureSensors = robot.updateCoP()
        
        prevCoP = pressureSensors
        count = count + 1 # keeps track of how many trajectory points it has reached
        print("Count: ", count)
    print(count, " / ", len(setUp)) # prints percentage of completion of balance test if falling was detected

    createWavePoints = [[math.radians(-45)], [math.radians(-70)], [math.radians(-45)]]
    wavePoints = trajPlannerPose.TrajPlannerPose(createWavePoints)
    wave = wavePoints.getCubicTraj(0.5, 100)
    count = 0  # Initialize outside of the stabilization loop for consistent counting
    while True:
        for point in wave:
            #tells robot trajectory is specifically for arms
            robot.motors[1].target = (point[1], 'P') # for right arm
            robot.moveAllToTarget()

            time.sleep(0.01) 
            # robot.IMUBalance(prevIMU) #where PID is used
            robot.CoPBalance(prevCoP)
            
            pressureSensors = robot.updateCoP()
            
            prevCoP = pressureSensors
            count = count + 1 # keeps track of how many trajectory points it has reached
            print("Count: ", count)
        print(count, " / ", len(setUp))





if(__name__ == "__main__"):
    main()