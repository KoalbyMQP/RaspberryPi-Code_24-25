import sys, time, math
import numpy as np
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid import trajPlannerPose
from backend.KoalbyHumanoid.Config import Joints
import matplotlib.pyplot as plt
from backend.KoalbyHumanoid.Plotter import Plotter

def main():
    # Edit to declare if you are testing the sim or the real robot
    is_real = True
    robot = Robot(is_real)
    print("Setup Complete")

    # imu_data = robot.imu_manager.getAllIMUData()
    # right_chest_imu = imu_data[0]
    # left_chest_imu = imu_data[1]
    # initial = robot.fuse_imu_data(right_chest_imu, left_chest_imu)
    # prevX = initial[0]
    # prevY = initial[1]
    # prevZ = initial[2]

    while True:
        imu_data = robot.imu_manager.getAllIMUData()
        right_chest_imu = imu_data[0]
        left_chest_imu = imu_data[1]
        fused_data = robot.fuse_imu_data(right_chest_imu, left_chest_imu)
        # Use the fused data for balance calculations
        # Xerror = prevX - fused_data[0]
        # Yerror = prevY - fused_data[1]
        # Zerror = prevZ - fused_data[2] 
        print("X value: ", fused_data[0], "    Y value: ", fused_data[1], "   Z value:", fused_data[2])
if(__name__ == "__main__"):
    main()