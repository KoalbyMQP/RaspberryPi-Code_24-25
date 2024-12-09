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
    robot.motors[1].target = (math.radians(80), 'P')  # RightShoulderAbductor
    robot.motors[6].target = (math.radians(-80), 'P') # LeftShoulderAbductor
    robot.motors[10].target = (math.radians(2.5), 'P') # HipsFront2Back
    robot.motors[11].target = (math.radians(0), 'P') # TorsoSide2Side
    robot.motors[12].target = (math.radians(0), 'P') # HipsRotate
    robot.motors[13].target = (math.radians(0), 'P') # HipsSide2Side
    robot.motors[14].target = (math.radians(0), 'P') # TorsoFront2Back
    robot.moveAllToTarget()
    print("Initial Pose Done")

    simStartTime = time.time()

    #stabilizes itself before starting test
    while time.time() - simStartTime < 7:
        time.sleep(0.01)
    print("Initialized")

    # Set initial balance targets
    initial = robot.fuse_imu_data()
    prevX = initial[0]
    prevY = initial[1]
    prevZ = initial[2]

    while True:
        robot.IMUBalance(prevX, prevY, prevZ)



if(__name__ == "__main__"):
    main()