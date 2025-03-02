import sys, time, math
import numpy as np
# sys.path.append("D:/Documents/College/Humanoid MQP Project/RaspberryPi-Code_23-24")
# sys.path.append("C:/Users/Gabriel/AppData/Local/Programs/Python/Python312/Lib/site-packages")
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid import trajPlannerPose
import matplotlib.pyplot as plt

is_real = False
robot = Robot(is_real)
print("Setup Complete")

def main():
    robot.motors[1].target = (math.radians(80), 'P')  # RightShoulderAbductor
    robot.motors[6].target = (math.radians(-80), 'P') # LeftShoulderAbductor
    robot.moveAllToTarget()
    print("Initial Pose Done")

    simStartTime = time.time()
    setPoints = [[0, 0], [math.radians(-80), math.radians(80)], [math.radians(0), math.radians(0)]]
    tj = trajPlannerPose.TrajPlannerPose(setPoints)
    traj = tj.getCubicTraj(10, 100)
    notFalling = True
    count = 0

    # Stabilize for 10 seconds
    while time.time() - simStartTime < 5:
        time.sleep(0.01)
    print("Initialized")

    # Set initial balance targets
    imu_data = robot.imu_manager.getAllIMUData()
    right_chest_imu = imu_data["RightChest"]
    left_chest_imu = imu_data["LeftChest"]
    torso_imu = imu_data["Torso"]
    initial = robot.fuse_imu_data(right_chest_imu, left_chest_imu, torso_imu)

    #set initial CoP
    inital_data = []
    inital_data.append(robot.updateCoP()[0])
    inital_data.append(robot.updateCoP()[1])
    print("Initial CoP: ", inital_data)
   
    motor_data = []
    count = 0

    while notFalling:
        for point in traj:
            # Move to trajectory points
            robot.motors[18].target = (point[1], 'P')  # right knee
            robot.motors[23].target = (point[2], 'P')  # left knee
            robot.motors[19].target = (point[1]/2, 'P')  # right ankle
            robot.motors[24].target = (point[2]/2, 'P')  # left ankle

            newTargetsForce = robot.CoPBalance(inital_data)
            motor_data.append(newTargetsForce)

            print("Motor Targets: ", newTargetsForce)
            
            robot.motors[17].target = (newTargetsForce[1], 'P') #for right kick
            robot.motors[22].target = (-newTargetsForce[1], 'P') #for left kick

            robot.moveAllToTarget()
            time.sleep(0.005)

            if(count == 60):
                motorRotate, motorFront2Back = zip(*motor_data)  # Unpacking x and y forces
                plt.plot(range(len(motor_data)), motorFront2Back, label="Motor Targets")
                plt.xlabel("Time step")
                plt.ylabel("Radians")
                plt.legend()
                plt.title("Motor Targets over Time")
                plt.grid()
                plt.show()
            
            count += 1

if __name__ == "__main__":
    main()
