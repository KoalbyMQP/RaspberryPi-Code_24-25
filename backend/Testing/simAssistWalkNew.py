import sys, time, math
import numpy as np 
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils
from scipy.interpolate import CubicSpline

sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient as sim
from backend.KoalbyHumanoid.Config import Joints

def setup_robot(is_real):
    robot = Robot(is_real)
    print("Setup Complete")
    return robot

def update_trajectory_and_control(robot, right_points, left_points, start_time):
    currentTime = time.time() - start_time

    # Torso
    robot.motors[10].target = (math.radians(-30), 'P')

    # Right arm
    robot.motors[0].target = (math.radians(10), 'P')
    robot.motors[1].target = (math.radians(-90), 'P') 
    # robot.motors[2].target = (math.radians())
    robot.motors[3].target = (math.radians(100), 'P') #pos
    robot.motors[4].target = (math.radians(0), 'P')

    # Left Arm
    robot.motors[5].target = (math.radians(-10), 'P')
    robot.motors[6].target = (math.radians(90), 'P')
    robot.motors[8].target = (math.radians(-100), 'P')#neg
    robot.motors[9].target = (math.radians(0), 'P') 

    # Right Leg
    robot.motors[15].target = (right_points[0], 'P')
    robot.motors[16].target = (right_points[1], 'P')
    robot.motors[17].target = (right_points[2], 'P')
    robot.motors[18].target = (right_points[3], 'P')
    robot.motors[19].target = (right_points[4], 'P')

    # Left Leg
    robot.motors[20].target = (left_points[1], 'P')
    robot.motors[21].target = (left_points[1], 'P')
    robot.motors[22].target = (left_points[2], 'P')
    robot.motors[23].target = (left_points[3], 'P')
    robot.motors[24].target = (left_points[4], 'P')

    robot.moveAllToTarget()

def initSimWalk(robot):

    # Right arm
    robot.motors[0].target = (math.radians(10), 'P')
    robot.motors[1].target = (math.radians(-90), 'P') 
    robot.motors[3].target = (math.radians(100), 'P') #pos
    robot.motors[4].target = (math.radians(5), 'P')

    # Left Arm
    robot.motors[5].target = (math.radians(-10), 'P')
    robot.motors[6].target = (math.radians(90), 'P')
    robot.motors[8].target = (math.radians(-100), 'P')#neg
    robot.motors[9].target = (math.radians(0), 'P') 

    # Right Leg
    robot.motors[15].target = (math.radians(0), 'P')
    robot.motors[16].target = (math.radians(0), 'P')
    robot.motors[17].target = (math.radians(0), 'P')
    robot.motors[18].target = (math.radians(0), 'P')
    robot.motors[19].target = (math.radians(0), 'P')

    # Left Leg
    robot.motors[20].target = (math.radians(0), 'P')
    robot.motors[21].target = (math.radians(0), 'P')
    robot.motors[22].target = (math.radians(0), 'P')
    robot.motors[23].target = (math.radians(0), 'P')
    robot.motors[24].target = (math.radians(0), 'P')

    robot.moveAllToTarget()


def main():
    is_real = True
    robot = setup_robot(is_real)

    initSimWalk(robot)

    time.sleep(30)


    left_leg_chain = Chain.from_urdf_file(
        "backend/Testing/robotChain.urdf",
        base_elements=['LeftHip', 'LeftLegRotator']
    )

    right_leg_chain = Chain.from_urdf_file(
        "backend/Testing/robotChain.urdf",
        base_elements=['RightHip', 'RightLegRotator']
    )
    
    x_base = 0.004
    y_base = -0.4
    z_base = -0.05

    left_base = [x_base, y_base, z_base]
    left_back_flat = [x_base, y_base, z_base - 0.1]
    left_back_not_flat = [x_base, y_base, z_base - 0.1]
    left_center_top = [x_base, y_base + 0.1, z_base]
    left_forward_top = [x_base, y_base + 0.1, z_base + 0.1]
    left_almost_center_top = [x_base, y_base, z_base]

    right_base = [x_base, y_base, z_base]
    right_back_flat = [x_base, y_base, z_base - 0.1]
    right_back_not_flat = [x_base, y_base, z_base - 0.1]
    right_center_top = [x_base, y_base + 0.1, z_base]
    right_forward_top = [x_base, y_base + 0.1, z_base + 0.1]
    right_almost_center_top = [x_base, y_base, z_base]

    target_orientation_left = [0, 0, 1]
    target_orientation_right = [0, 0, 1]

    initial_position_left = [0, 0, 0, 0, 0]
    initial_position_right = [0, 0, 0, 0, 0]

    all_trajectories = []

    #RLF = Right Leg First 
    #LLF = Left Leg First
    #RLP = Right Leg Positions
    #LLP = Left Leg Positions
    RLF_RLP = [right_center_top, right_forward_top, right_almost_center_top, right_base, right_base]
    RLF_LLP = [left_base, left_base, left_back_flat, left_back_not_flat, left_center_top]
    LLF_RLP = [right_base, right_base, right_back_flat, right_back_not_flat, right_center_top]
    LLF_LLP = [left_center_top, left_forward_top, left_almost_center_top, left_base, left_base]

    total_time = []
    total_vel = []
    total_acc = []
    step_time = 2
    for i in range(len(RLF_LLP)):
        total_time.append([i * step_time, i * step_time, i * step_time])
        total_vel.append([0, 0, 0])
        total_acc.append([0, 0, 0])

    RLF_LLP_traj = TrajPlannerTime(total_time, RLF_LLP, total_vel, total_acc)
    RLF_RLP_traj = TrajPlannerTime(total_time, RLF_RLP, total_vel, total_acc)
    all_trajectories.append([RLF_LLP_traj,RLF_RLP_traj])

    LLF_LLP_traj = TrajPlannerTime(total_time, LLF_LLP, total_vel, total_acc)
    LLF_RLP_traj = TrajPlannerTime(total_time, LLF_RLP, total_vel, total_acc)
    all_trajectories.append([LLF_LLP_traj,LLF_RLP_traj])

    traj_number=0
    while True:
        start_time = time.time()
        while time.time() - start_time < step_time*(len(total_time)-1):
                left_quintic = all_trajectories[traj_number][0].getQuinticPositions(time.time() - start_time)
                right_quintic = all_trajectories[traj_number][1].getQuinticPositions(time.time() - start_time)

                target_position_left = left_quintic
                ik_solution_left = left_leg_chain.inverse_kinematics(
                    target_position_left,
                    initial_position=initial_position_left,
                    target_orientation=target_orientation_left,
                    orientation_mode='Z' 
                )
                initial_position_left = ik_solution_left
                   
                target_position_right = right_quintic
                ik_solution_right = right_leg_chain.inverse_kinematics(
                    target_position_right,
                    initial_position=initial_position_right,
                    target_orientation=target_orientation_right,
                    orientation_mode='Z'
                )
                initial_position_right = ik_solution_right
                
                update_trajectory_and_control(robot, ik_solution_right, ik_solution_left, start_time)
        if(traj_number < (len(all_trajectories)-1)):
            traj_number = traj_number + 1
        else:
            traj_number = 0

if __name__ == "__main__":
    main()

