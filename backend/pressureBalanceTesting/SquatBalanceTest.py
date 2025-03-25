import sys, time, math
import numpy as np
# sys.path.append("D:/Documents/College/Humanoid MQP Project/RaspberryPi-Code_23-24")
# sys.path.append("C:/Users/Gabriel/AppData/Local/Programs/Python/Python312/Lib/site-packages")
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid import trajPlannerPose
import matplotlib.pyplot as plt
from ikpy.chain import Chain

is_real = False
robot = Robot(is_real)
print("Setup Complete")

def initialize(): 
    robot.motors[1].target = (math.radians(80), 'P')  # RightShoulderAbductor
    robot.motors[6].target = (math.radians(-80), 'P') # LeftShoulderAbductor
    robot.moveAllToTarget()
    print("Initial Pose Done")


    simStartTime = time.time()
    setPoints = [[0, 0], [math.radians(0), math.radians(90)], [math.radians(0), math.radians(0)]]
    tj = trajPlannerPose.TrajPlannerPose(setPoints)
    traj = tj.getCubicTraj(10, 100)
    notFalling = True
    count = 0

    # Stabilize for 10 seconds
    while time.time() - simStartTime < 5:
        time.sleep(0.01)
    print("Initialized")

    return notFalling,traj

def calculate_kick_angles(leg_chain, kick_angle, knee_angle, ankle_angle):
        leg_joint_angles = [0.0, kick_angle, knee_angle, ankle_angle]
        leg_fk = leg_chain.forward_kinematics(leg_joint_angles)
        foot_to_kick = np.linalg.inv(leg_fk)
        R32 = foot_to_kick[2][1]
        R33 = foot_to_kick[2][2]
        kick_angle = np.arctan2(R32, R33)
        return kick_angle

def main():

    notFalling,traj = initialize()

    left_leg_chain = Chain.from_urdf_file(
            "backend/Testing/robotChain.urdf",
            base_elements=['LeftHip2', 'LeftKick']
        )
    right_leg_chain = Chain.from_urdf_file(
            "backend/Testing/robotChain.urdf",
            base_elements=['RightHip2', 'RightKick']
        )

    #set initial CoP
    inital_data = []
    inital_data.append(robot.updateCoP()[0])
    inital_data.append(robot.updateCoP()[1])
    print("Initial CoP: ", inital_data)
    motor_data = []
    count = 0
    while notFalling:
        for point in traj:

            right_kick_angle = 0
            right_knee_angle = point[1]
            right_ankle_angle = point[1]/2

            left_kick_angle = 0
            left_knee_angle = point[2]
            left_ankle_angle = point[2]/2

            # robot.motors[18].target = (right_knee_angle, 'P')  # right knee
            # robot.motors[23].target = (left_knee_angle, 'P')  # left knee
            # robot.motors[19].target = (right_ankle_angle, 'P')  # right ankle
            # robot.motors[24].target = (left_ankle_angle, 'P')  # left ankle

            right_kick_angle = calculate_kick_angles(right_leg_chain, right_kick_angle, right_knee_angle, right_ankle_angle)
            left_kick_angle = calculate_kick_angles(left_leg_chain, left_kick_angle, left_knee_angle, left_ankle_angle)

            # print("right kick angle: ", right_kick_angle,  "\n")
            # print("left kick angle: ", left_kick_angle,  "\n")

            newTargetsForce = robot.CoPBalance(inital_data)
            # print("Motor Targets: ", newTargetsForce)
            # print("Pressure sensor values: ",newTargetsForce)
            motor_data.append(newTargetsForce)

            # robot.motors[17].target = (newTargetsForce[1] - right_kick_angle, 'P') #for right kick
            # robot.motors[22].target = (-(newTargetsForce[1] - left_kick_angle), 'P') #for left kick

            robot.motors[1].target = (math.radians(80), 'P')  # RightShoulderAbductor
            robot.motors[6].target = (math.radians(-80), 'P') # LeftShoulderAbductor

            robot.motors[10].target = (point[1]/2, 'P') #for hips front to back 
            robot.motors[13].target = (point[1]/2, 'P') #for hips side to side 

            robot.moveAllToTarget()
            time.sleep(0.005)

            # if(count == 65):
            #     motorRotate, motorFront2Back = zip(*motor_data)  # Unpacking x and y forces
            #     plt.plot(range(len(motor_data)), motorFront2Back, label=" Kick Motor Targets")
            #     plt.plot(range(len(motor_data)), motorRotate, label="Side to Side Motor Targets")
            #     plt.xlabel("Time step")
            #     plt.ylabel("Radians")
            #     plt.legend()
            #     plt.title("Motor Targets over Time")
            #     plt.grid()
            #     plt.show()

            # if count == 65:
            #     plt.figure(figsize=(10, 5))
            #     motor_data = np.array(motor_data)  # Shape: (timesteps, 8 sensors)
            #     for sensor_id in range(4):
            #         plt.plot(range(len(motor_data)), motor_data[:, sensor_id], label=f"Right Sensor {sensor_id+1}")
            #     plt.xlabel("Time Step")
            #     plt.ylabel("Pressure / Force")
            #     plt.legend()
            #     plt.title("Pressure Sensor Readings Over Time")
            #     plt.grid()
            #     # plt.show()

            #     plt.figure(figsize=(10, 5))
            #     for sensor_id in range(4):
            #         plt.plot(range(len(motor_data)), motor_data[:, sensor_id+4], label=f"Left Sensor {sensor_id+1}")
            #     plt.xlabel("Time Step")
            #     plt.ylabel("Pressure / Force")
            #     plt.legend()
            #     plt.title("Pressure Sensor Readings Over Time")
            #     plt.grid()
            #     plt.show()
            
            count += 1

if __name__ == "__main__":
    main()
