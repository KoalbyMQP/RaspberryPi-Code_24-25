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

    initial = robot.fuse_imu_data()
    prevX = initial[0]
    prevY = initial[1]
    prevZ = initial[2]
    startTime = time.time()

    while True:
        fused_data = robot.fuse_imu_data()
        # Use the fused data for balance calculations
        Xerror = prevX - fused_data[0]
        Yerror = prevY - fused_data[1]
        Zerror = prevZ - fused_data[2] 
if(__name__ == "__main__"):
    main()