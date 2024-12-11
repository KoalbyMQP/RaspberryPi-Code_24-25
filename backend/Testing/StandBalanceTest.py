import sys, time, math
import numpy as np
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid import trajPlannerPose
from backend.KoalbyHumanoid.Config import Joints
import matplotlib.pyplot as plt
from backend.KoalbyHumanoid.Plotter import Plotter

def initialize(robot):
    robot.motors[1].target = (math.radians(-70), 'P')  # RightShoulderAbductor
    robot.motors[6].target = (math.radians(70), 'P') # LeftShoulderAbductor
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
    # Edit to declare if you are testing the sim or the real robot
    is_real = True
    robot = Robot(is_real)
    print("Setup Complete")
    initialize(robot)
    scale_factor_front_to_back = 2
    imu_data = robot.imu_manager.getAllIMUData()
    right_chest_imu = imu_data[0]
    left_chest_imu = imu_data[1]
    initial = robot.fuse_imu_data(right_chest_imu, left_chest_imu)
    prevX = initial[0]
    prevY = initial[1]
    prevZ = initial[2]
    # creates trajectory of movements (squatting knees to 80 degrees)
    startTime = time.time()

    #stabilizes itself before starting test
    while time.time() - startTime < 10:
        time.sleep(0.1)
    print("Initialized")
    count = 0
    while True:
        newTargetX = robot.IMUBalance(prevX, prevY, prevZ)[0]
        newTargetY = robot.IMUBalance(prevX, prevY, prevZ)[1]
        newTargetZ = robot.IMUBalance(prevX, prevY, prevZ)[2]

        print("X value: ", newTargetX, "    Y value: ", newTargetY, "   Z value:", newTargetZ)

        robot.motors[10].target = (-math.radians(newTargetZ), 'P')  # Adjust yaw
        robot.motors[13].target = ((math.radians(newTargetY) * scale_factor_front_to_back), 'P')  # Adjust pitch
        robot.motors[12].target = (math.radians(newTargetX), 'P')  # Adjust roll

        # robot.motors[10].target = (0, 'P')  # Adjust yaw
        # robot.motors[13].target = (0, 'P')  # Adjust pitch
        # robot.motors[12].target = (0, 'P')  # Adjust roll
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
        #count = count + 1 # keeps track of how many trajectory points it has reached
        # print(count, " / ", len(wave))
if(__name__ == "__main__"):
    main()