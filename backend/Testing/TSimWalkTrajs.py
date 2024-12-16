import sys, time, math
import numpy as np 
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils
from matplotlib.animation import FuncAnimation
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

        # Right arm
    robot.motors[0].target = (math.radians(-20), 'P')
    robot.motors[1].target = (math.radians(90), 'P') 
    robot.motors[3].target = (math.radians(90), 'P') 
    robot.motors[4].target = (math.radians(-15), 'P')

    # Left Arm
    robot.motors[5].target = (math.radians(20), 'P')
    robot.motors[6].target = (math.radians(-90), 'P')
    robot.motors[8].target = (math.radians(-90), 'P')
    robot.motors[9].target = (math.radians(15), 'P') 

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

def main():
    is_real = False
    robot = setup_robot(is_real)

    left_leg_chain = Chain.from_urdf_file(
        "backend/Testing/robotChain.urdf",
        base_elements=['LeftHip', 'LeftLegRotator']
    )

    right_leg_chain = Chain.from_urdf_file(
        "backend/Testing/robotChain.urdf",
        base_elements=['RightHip', 'RightLegRotator']
    )
    
    x_base = 0.004
    y_base = -0.385
    z_base = 0.0

    left_base = [x_base, y_base, z_base]
    left_back_flat = [x_base, y_base + 0.085, z_base - 0.241]
    left_back_not_flat = [x_base, y_base, z_base - 0.241]
    left_center_top = [x_base, y_base + 0.1, z_base]
    left_forward_top = [x_base, y_base + 0.1, z_base + 0.2]
    left_almost_center_top = [x_base, y_base + 0.085, z_base]

    right_base = [x_base, y_base, z_base]
    right_back_flat = [x_base, y_base + 0.085, z_base - 0.241]
    right_back_not_flat = [x_base, y_base, z_base - 0.241]
    right_center_top = [x_base, y_base + 0.1, z_base]
    right_forward_top = [x_base, y_base + 0.1, z_base + 0.2]
    right_almost_center_top = [x_base, y_base + 0.085, z_base]

    target_orientation_left = [0, 0, 1]
    target_orientation_right = [0, 0, 1]

    initial_position_left = [0, 0, 0, 0, 0]
    initial_position_right = [0, 0, 0, 0, 0]

    all_trajectories = []

    #RLF = Right Leg First 
    #LLF = Left Leg First
    #RLP = Right Leg Positions
    #LLP = Left Leg Positions
    RLF_RLP = [right_base, right_center_top, right_forward_top, right_almost_center_top, right_base, right_base, right_base]
    RLF_LLP = [left_base, left_base, left_base, left_back_flat, left_back_not_flat, left_center_top, left_base]
    LLF_RLP = [right_base, right_base, right_base, right_back_flat, right_back_not_flat, right_center_top, right_base]
    LLF_LLP = [left_base, left_center_top, left_forward_top, left_almost_center_top, left_base, left_base, left_base]

    total_time = []
    total_vel = []
    total_acc = []
    step_time = 1
    for i in range(len(RLF_LLP)):
        total_time.append([i * step_time, i * step_time, i * step_time])
        total_vel.append([0, 0, 0])
        total_acc.append([0, 0, 0])

    print(total_time, RLF_LLP, total_vel, total_acc)

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
                print(left_quintic)

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


#------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------
# Jai Code
#------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------

# import sys, time, math
# import numpy as np 
# import matplotlib.pyplot as plt
# from ikpy.chain import Chain
# from ikpy.utils import plot as plot_utils
# from matplotlib.animation import FuncAnimation
# from scipy.interpolate import CubicSpline

# sys.path.append("./")
# from backend.KoalbyHumanoid.Robot import Robot
# from coppeliasim_zmqremoteapi_client import RemoteAPIClient as sim
# from backend.KoalbyHumanoid.Config import Joints

# def setup_robot(is_real):
#     robot = Robot(is_real)
#     print("Setup Complete")
#     return robot

# def update_trajectory_and_control(robot, n_points, right_points, left_points, start_time):
#     currentTime = time.time() - start_time

#     # Right Leg
#     robot.motors[15].target = (0, 'P')
#     robot.motors[16].target = (right_points[0], 'P')
#     robot.motors[17].target = (right_points[1], 'P')
#     robot.motors[18].target = (right_points[2], 'P')
#     robot.motors[19].target = (right_points[3], 'P')

#     # Left Leg
#     robot.motors[20].target = (0, 'P')
#     robot.motors[21].target = (left_points[0], 'P')
#     robot.motors[22].target = (left_points[1], 'P')
#     robot.motors[23].target = (left_points[2], 'P')
#     robot.motors[24].target = (left_points[3], 'P')

#     robot.moveAllToTarget()

# def main():
#     is_real = False
#     robot = setup_robot(is_real)

#     left_leg_chain = Chain.from_urdf_file(
#         "backend/Testing/robotChain.urdf",
#         base_elements=['LeftHip', 'LeftLegRotator']
#     )

#     right_leg_chain = Chain.from_urdf_file(
#         "backend/Testing/robotChain.urdf",
#         base_elements=['RightHip', 'RightLegRotator']
#     )

#     y_avg_position = -0.385
#     z_avg_position = 0.0
#     y_scale = 2
#     z_scale = 1 
#     target_positions_left = np.array([
#         [0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.0],
#         [0.04,  y_avg_position + y_scale*0.050 , z_avg_position + z_scale*0.1],
#         [0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.1],
#         [0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.0]
#     ])
#     target_positions_right = np.array([
#         [-0.04,  y_avg_position + y_scale*0.050 , z_avg_position + z_scale*0.1],
#         [-0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.1],
#         [-0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.0],
#         [-0.04,  y_avg_position + y_scale*0.050 , z_avg_position + z_scale*0.1]
#     ])

#     n_points = 100
#     t_left = np.linspace(0, 1, len(target_positions_left))
#     splines_left = [CubicSpline(t_left, target_positions_left[:, dim]) for dim in range(3)]
#     trajectory_left = np.vstack([spline(np.linspace(0, 1, n_points)) for spline in splines_left]).T

#     t_right = np.linspace(0, 1, len(target_positions_right))
#     splines_right = [CubicSpline(t_right, target_positions_right[:, dim]) for dim in range(3)]
#     trajectory_right = np.vstack([spline(np.linspace(0, 1, n_points)) for spline in splines_right]).T

#     left_points = []
#     right_points = []

#     target_orientation = [0, 0, 0.05]

#     start_time = time.time()
#     state = 0

#     while True:
#         for frame in range(n_points):
#             target_position_left = trajectory_left[frame]
#             ik_solution_left = left_leg_chain.inverse_kinematics(
#                 target_position=target_position_left,
#                 target_orientation=target_orientation,
#                 orientation_mode="Z"
#             )
#             print(target_position_left)
            
#             if len(ik_solution_left) >= 6:
#                 left_points[:] = [ik_solution_left[1], ik_solution_left[2], ik_solution_left[3], ik_solution_left[4], ik_solution_left[5]]
#             else:
#                 left_points[:] = ik_solution_left[1:]

#             target_position_right = trajectory_right[frame]
#             ik_solution_right = right_leg_chain.inverse_kinematics(
#                 target_position=target_position_right,
#                 target_orientation=target_orientation,
#                 orientation_mode="Z"
#             )
            
#             if len(ik_solution_right) >= 6:
#                 right_points[:] = [ik_solution_right[1], ik_solution_right[2], ik_solution_right[3], ik_solution_right[4], ik_solution_right[5]]
#             else:
#                 right_points[:] = ik_solution_right[1:]

#             update_trajectory_and_control(robot, n_points, right_points, left_points, start_time)

#         # if time.time() - start_time >= 3:
#         #     state = (state + 1) % 2  
#         #     start_time = time.time()

# if __name__ == "__main__":
#     main()