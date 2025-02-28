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
    prevX = initial[0]
    prevZ = initial[2]

    #set initial CoP
    #force_data = robot.updateCoP()
    force_data = [0,0]
    print(str(force_data))

    while notFalling:
        for point in traj:
            # Move to trajectory points
            robot.motors[18].target = (point[1], 'P')  # right knee
            robot.motors[23].target = (point[2], 'P')  # left knee
            robot.moveAllToTarget()
            time.sleep(0.005)

            #robot.IMUBalance(prevX, prevZ)
            newTargetsForce = robot.CoPBalance(force_data)
            # print("Force error X:" + str(newTargetsForce[0]))
            # print("Force error Y:" + str(newTargetsForce[1]))
            #print(robot.fused_imu)
            if robot.fused_imu[0] > 15 or robot.fused_imu[1] > 15 or robot.fused_imu[2] > 15:  
                print("FALLING")
                notFalling = False
                break
            count += 1
            print(count, "/", len(traj))
            
    
    print("Dead")
    print(count, "/", len(traj))

if __name__ == "__main__":
    main()
