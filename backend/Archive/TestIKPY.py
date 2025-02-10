# from ikpy.chain import Chain
# from ikpy.link import OriginLink, URDFLink

# robot_chain = Chain.from_urdf_file("/Users/sahilmirani/MQP/RaspberryPi-Code_24-25/backend/Testing/FullAssem11_24_23(1).urdf", base_elements=["LeftHip2","LeftLegRotator"])

# for link in enumerate(robot_chain.links):
#     print(link)
# print("Number of joints:")

# import numpy as np
# import matplotlib.pyplot as plt
# from ikpy.chain import Chain
# from ikpy.utils import plot as plot_utils

# left_leg_chain = Chain.from_urdf_file(
#     "/Users/sahilmirani/MQP/RaspberryPi-Code_24-25/backend/Testing/FullAssem11_24_23(1).urdf", 
#     base_elements=['LeftHip', 'LeftLegRotator']
# )

# Test Code
# target_positions = [0.1, -0.01, -0.45]

# ik_solution = left_leg_chain.inverse_kinematics(target_positions)
# print("Joint angles:", ik_solution)

# real_frame = left_leg_chain.forward_kinematics(ik_solution)

# fig, ax = plot_utils.init_3d_figure()

# left_leg_chain.plot(ik_solution, ax)

# ax.set_xlim([-1, 1])
# ax.set_ylim([-1, 1])
# ax.set_zlim([-1, 1])

# plt.show()

## Jai code

# import numpy as np
# import matplotlib.pyplot as plt
# from ikpy.chain import Chain
# from ikpy.utils import plot as plot_utils
# from matplotlib.animation import FuncAnimation
# from scipy.interpolate import CubicSpline
# left_leg_chain = Chain.from_urdf_file(
#     "/Users/sahilmirani/MQP/RaspberryPi-Code_24-25/backend/Testing/FullAssem11_24_23(1).urdf",
#     base_elements=['LeftHip', 'LeftLegRotator']
# )
# right_leg_chain = Chain.from_urdf_file(
#     "/Users/sahilmirani/MQP/RaspberryPi-Code_24-25/backend/Testing/FullAssem11_24_23(1).urdf",
#     base_elements=['RightHip', 'RightLegRotator']
# )
# target_positions_left = np.array([
#     [0.05, 0.0, -0.35],
#     [0.05, 0.1, -0.325],
#     [0.05, 0.1, -0.3],
#     [0.05, 0.0, -0.3],
#     [0.05, -0.1, -0.3],
#     [0.05, -0.1, -0.325],
#     [0.05, 0.0, -0.35]
# ])
# target_positions_right = np.array([
#     [-0.05, 0.0, -0.3],
#     [-0.05, -0.1, -0.3],
#     [-0.05, -0.1, -0.325],
#     [-0.05, 0.0, -0.35],
#     [-0.05, 0.1, -0.325],
#     [-0.05, 0.1, -0.3],
#     [-0.05, 0.0, -0.3]
# ])
# n_points = 100
# t_left = np.linspace(0, 1, len(target_positions_left))
# splines_left = [CubicSpline(t_left, target_positions_left[:, dim]) for dim in range(3)]
# trajectory_left = np.vstack([spline(np.linspace(0, 1, n_points)) for spline in splines_left]).T
# t_right = np.linspace(0, 1, len(target_positions_right))
# splines_right = [CubicSpline(t_right, target_positions_right[:, dim]) for dim in range(3)]
# trajectory_right = np.vstack([spline(np.linspace(0, 1, n_points)) for spline in splines_right]).T
# fig, ax = plot_utils.init_3d_figure()
# ax.set_xlim([-0.3, 0.3])
# ax.set_ylim([-0.75, 0.75])
# ax.set_zlim([-0.6, 0.2])
# def update_plot(frame):
#     target_position_left = trajectory_left[frame]
#     ik_solution_left = left_leg_chain.inverse_kinematics(target_position_left)
#     target_position_right = trajectory_right[frame]
#     ik_solution_right = right_leg_chain.inverse_kinematics(target_position_right)
#     print("Left IK Solution:", ik_solution_left)
#     print("Right IK Solution:", ik_solution_right)
#     ax.cla()
#     left_leg_chain.plot(ik_solution_left, ax)
#     right_leg_chain.plot(ik_solution_right, ax)
#     ax.set_xlim([-0.3, 0.3])
#     ax.set_ylim([-0.75, 0.75])
#     ax.set_zlim([-0.6, 0.2])
#     return ax
# ani = FuncAnimation(fig, update_plot, frames=n_points, interval=50, blit=False)
# plt.show()

###################################################################
# Quintic Trajectory with intervals

# import numpy as np
# import matplotlib.pyplot as plt
# from ikpy.chain import Chain
# from ikpy.utils import plot as plot_utils
# from matplotlib.animation import FuncAnimation
# from scipy.interpolate import make_interp_spline
# import time

# # Load the chains from the URDF file
# left_leg_chain = Chain.from_urdf_file(
#     "/Users/sahilmirani/MQP/RaspberryPi-Code_24-25/backend/Testing/FullAssembly_updated.urdf",
#     base_elements=['LeftHip', 'LeftLegRotator']
# )
# right_leg_chain = Chain.from_urdf_file(
#     "/Users/sahilmirani/MQP/RaspberryPi-Code_24-25/backend/Testing/FullAssembly_updated.urdf",
#     base_elements=['RightHip', 'RightLegRotator']
# )

# # Original waypoints
# target_positions_left = np.array([
#     [0.05, 0.0, -0.35],
#     [0.05, 0.1, -0.325],
#     [0.05, 0.1, -0.3],
#     [0.05, 0.0, -0.3],
#     [0.05, -0.1, -0.3],
#     [0.05, -0.1, -0.325],
#     [0.05, 0.0, -0.35]
# ])
# target_positions_right = np.array([
#     [-0.05, 0.0, -0.3],
#     [-0.05, -0.1, -0.3],
#     [-0.05, -0.1, -0.325],
#     [-0.05, 0.0, -0.35],
#     [-0.05, 0.1, -0.325],
#     [-0.05, 0.1, -0.3],
#     [-0.05, 0.0, -0.3]
# ])

# # Parameters for quintic spline
# initial_velocity = [0, 0, 0]
# final_velocity = [0, 0, 0]    
# initial_acceleration = [0, 0, 0]  
# final_acceleration = [0, 0, 0]    

# # Increase the number of points between each waypoint
# num_interpolated_points = 50  # Number of points between each original waypoint

# def interpolate_points(waypoints, num_points):
#     # Create a new set of points between each pair of original waypoints
#     interpolated_points = []
#     for i in range(len(waypoints) - 1):
#         interp_points = np.linspace(waypoints[i], waypoints[i + 1], num_points, endpoint=False)
#         interpolated_points.extend(interp_points)
#     interpolated_points.append(waypoints[-1])  # Add the last waypoint
#     return np.array(interpolated_points)

# # Interpolate to create more waypoints
# interpolated_positions_left = interpolate_points(target_positions_left, num_interpolated_points)
# interpolated_positions_right = interpolate_points(target_positions_right, num_interpolated_points)

# # Create a quintic spline using the new interpolated waypoints
# t_left = np.linspace(0, 1, len(interpolated_positions_left))
# splines_left = [make_interp_spline(t_left, interpolated_positions_left[:, dim], k=5,
#                                    bc_type=([(1, initial_velocity[dim]), (2, initial_acceleration[dim])],
#                                             [(1, final_velocity[dim]), (2, final_acceleration[dim])]))
#                 for dim in range(3)]

# t_right = np.linspace(0, 1, len(interpolated_positions_right))
# splines_right = [make_interp_spline(t_right, interpolated_positions_right[:, dim], k=5,
#                                     bc_type=([(1, initial_velocity[dim]), (2, initial_acceleration[dim])],
#                                              [(1, final_velocity[dim]), (2, final_acceleration[dim])]))
#                  for dim in range(3)]

# fig, ax = plot_utils.init_3d_figure()
# ax.set_xlim([-0.3, 0.3])
# ax.set_ylim([-0.75, 0.75])
# ax.set_zlim([-0.6, 0.2])

# start_time = time.time()
# total_duration = 5.0  # Duration of one cycle of the trajectory in seconds
# speed_factor = 1.0    # Increase this value to make the motion faster, decrease to make it slower

# def update_plot(frame):
#     elapsed_time = time.time() - start_time
#     t = ((elapsed_time * speed_factor) % total_duration) / total_duration
#     target_position_left = np.array([spline(t) for spline in splines_left])
#     target_position_right = np.array([spline(t) for spline in splines_right])
#     ik_solution_left = left_leg_chain.inverse_kinematics(target_position_left)
#     ik_solution_right = right_leg_chain.inverse_kinematics(target_position_right)
#     print("Left IK Solution:", ik_solution_left)
#     print("Right IK Solution:", ik_solution_right)
#     ax.cla()
#     left_leg_chain.plot(ik_solution_left, ax)
#     right_leg_chain.plot(ik_solution_right, ax)
#     ax.set_xlim([-0.3, 0.3])
#     ax.set_ylim([-0.75, 0.75])
#     ax.set_zlim([-0.6, 0.2])

#     return ax

# ani = FuncAnimation(fig, update_plot, frames=None, interval=50, blit=False)

# plt.show()


#########################################################
#Quintic spline for trajectories with way point

# import numpy as np
# import matplotlib.pyplot as plt
# from ikpy.chain import Chain
# from ikpy.utils import plot as plot_utils
# from matplotlib.animation import FuncAnimation
# from scipy.interpolate import make_interp_spline
# import time

# # Load the chains from the URDF file
# left_leg_chain = Chain.from_urdf_file(
#     "/Users/sahilmirani/MQP/RaspberryPi-Code_24-25/backend/Testing/FullAssembly_updated.urdf",
#     base_elements=['LeftHip', 'LeftLegRotator']
# )
# right_leg_chain = Chain.from_urdf_file(
#     "/Users/sahilmirani/MQP/RaspberryPi-Code_24-25/backend/Testing/FullAssembly_updated.urdf",
#     base_elements=['RightHip', 'RightLegRotator']
# )

# target_positions_left = np.array([
#     [0.05, 0.0, -0.35],
#     [0.05, 0.1, -0.325],
#     [0.05, 0.1, -0.3],
#     [0.05, 0.0, -0.3],
#     [0.05, -0.1, -0.3],
#     [0.05, -0.1, -0.325],
#     [0.05, 0.0, -0.35]
# ])
# target_positions_right = np.array([
#     [-0.05, 0.0, -0.3],
#     [-0.05, -0.1, -0.3],
#     [-0.05, -0.1, -0.325],
#     [-0.05, 0.0, -0.35],
#     [-0.05, 0.1, -0.325],
#     [-0.05, 0.1, -0.3],
#     [-0.05, 0.0, -0.3]
# ])

# initial_velocity = [0, 0, 0]
# final_velocity = [0, 0, 0]    
# initial_acceleration = [0, 0, 0]  
# final_acceleration = [0, 0, 0]    

# t_left = np.linspace(0, 1, len(target_positions_left))
# splines_left = [make_interp_spline(t_left, target_positions_left[:, dim], k=5,
#                                    bc_type=([(1, initial_velocity[dim]), (2, initial_acceleration[dim])],
#                                             [(1, final_velocity[dim]), (2, final_acceleration[dim])]))
#                 for dim in range(3)]

# t_right = np.linspace(0, 1, len(target_positions_right))
# splines_right = [make_interp_spline(t_right, target_positions_right[:, dim], k=5,
#                                     bc_type=([(1, initial_velocity[dim]), (2, initial_acceleration[dim])],
#                                              [(1, final_velocity[dim]), (2, final_acceleration[dim])]))
#                  for dim in range(3)]

# fig, ax = plot_utils.init_3d_figure()
# ax.set_xlim([-0.3, 0.3])
# ax.set_ylim([-0.75, 0.75])
# ax.set_zlim([-0.6, 0.2])

# start_time = time.time()
# total_duration = 5.0  # Duration of one cycle of the trajectory in seconds
# speed_factor = 1.0    # Increase this value to make the motion faster, decrease to make it slower

# def update_plot(frame):
#     elapsed_time = time.time() - start_time
#     t = ((elapsed_time * speed_factor) % total_duration) / total_duration
#     target_position_left = np.array([spline(t) for spline in splines_left])
#     target_position_right = np.array([spline(t) for spline in splines_right])
#     ik_solution_left = left_leg_chain.inverse_kinematics(target_position_left)
#     ik_solution_right = right_leg_chain.inverse_kinematics(target_position_right)
#     print("Left IK Solution:", ik_solution_left)
#     print("Right IK Solution:", ik_solution_right)
#     ax.cla()
#     left_leg_chain.plot(ik_solution_left, ax)
#     right_leg_chain.plot(ik_solution_right, ax)
#     ax.set_xlim([-0.3, 0.3])
#     ax.set_ylim([-0.75, 0.75])
#     ax.set_zlim([-0.6, 0.2])

#     return ax

# ani = FuncAnimation(fig, update_plot, frames=None, interval=50, blit=False)

# plt.show()

#####################################################################
# #Time based polynomial quintic code
import numpy as np
from ikpy.chain import Chain
import sys, time, math 
sys.path.append("./")
import copy
# from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
from backend.Testing import assistWalkViaPoints as via
# from coppeliasim_zmqremoteapi_client import RemoteAPIClient as sim
# from backend.KoalbyHumanoid.Config import Joints

left_leg_chain = Chain.from_urdf_file(
    "/Users/sahilmirani/MQP/RaspberryPi-Code_24-25/backend/Testing/robotChain.urdf",
    base_elements=['Pelvis', 'LeftLegAbductor']
)

right_leg_chain = Chain.from_urdf_file(
    "/Users/sahilmirani/MQP/RaspberryPi-Code_24-25/backend/Testing/robotChain.urdf",
    base_elements=['Pelvis', 'RightLegAbductor']
)



# for link in enumerate(left_leg_chain.links):
#     print(link)

def findEndEffectorPos(right_leg_chain, left_leg_chain):
    # Create arrays to store the results
    rightEEpose_Even2Right = []
    rightEEpose_Right2Left = []
    rightEEpose_Left2Right = []

    leftEEpose_Even2Right = []
    leftEEpose_Right2Left = []
    leftEEpose_Left2Right = []


    # Iterate through the "Even2Right" poses for right leg
    for i in range(4):  # Assuming there are 2 positions (start and final)
            # Assuming the joint vector has 5 values
        joint_vector = via.rf_Even2Right[1][i]

        # Prepend 0 at the start of the joint vector
        joint_vector = np.insert(joint_vector,0, len(joint_vector)-1 ) 
        rightEEpose_Even2Right.append(right_leg_chain.forward_kinematics(joint_vector)[:3, 3])

    # # Iterate through the "Right2Left" poses for right leg
    # for i in range(2):  # Assuming there are 2 positions (start and final)
    #     rightEEpose_Right2Left.append(right_leg_chain.forward_kinematics(via.rf_Right2Left[1][i])[:3, 3])

    # # Iterate through the "Left2Right" poses for right leg
    # for i in range(4):  # Assuming there are 4 positions
    #     rightEEpose_Left2Right.append(right_leg_chain.forward_kinematics(via.rf_Left2Right[1][i])[:3, 3])

    # # Iterate through the "Even2Right" poses for left leg
    # for i in range(2):  # Assuming there are 2 positions (start and final)
    #     leftEEpose_Even2Right.append(left_leg_chain.forward_kinematics(via.lf_Even2Right[1][i])[:3, 3])

    # # Iterate through the "Right2Left" poses for left leg
    # for i in range(4):  # Assuming there are 4 positions (start and final)
    #     leftEEpose_Right2Left.append(left_leg_chain.forward_kinematics(via.lf_Right2Left[1][i])[:3, 3])

    # # Iterate through the "Left2Right" poses for left leg
    # for i in range(2):  # Assuming there are 2 positions
    #     leftEEpose_Left2Right.append(left_leg_chain.forward_kinematics(via.lf_Left2Right[1][i])[:3, 3])

    # Return all computed poses as a dictionary for better structure
    return {
        'rightEEpose_Even2Right': rightEEpose_Even2Right
        # 'rightEEpose_Right2Left': rightEEpose_Right2Left,
        # 'rightEEpose_Left2Right': rightEEpose_Left2Right,
        # 'leftEEpose_Even2Right': leftEEpose_Even2Right,
        # 'leftEEpose_Right2Left': leftEEpose_Right2Left,
        # 'leftEEpose_Left2Right': leftEEpose_Left2Right
    }


# print(left_leg_chain.forward_kinematics(via.lf_Left2Right[1][0]))
# print(left_leg_chain.forward_kinematics(via.lf_Left2Right[1][1]))

poses = findEndEffectorPos(right_leg_chain, left_leg_chain)
rightPose1 = poses['rightEEpose_Even2Right']
# rightPose2 = poses['rightEEpose_Right2Left']
# rightPose3 = poses['rightEEpose_Left2Right']
# leftPose1 = poses['leftEEpose_Even2Right']
# leftPose2 = poses['leftEEpose_Right2Left']
# leftPose3 = poses['leftEEpose_Left2Right']

# print(via.rf_Even2Right[1], sep='\n\n')
# print(rightPose1, sep='\n\n')

kitten = np.array([1.80111086e-14 ,-1.49795680e-01 ,-2.51464650e-01 , 5.95714182e-01 ,-1.50951099e-02,  0.00000000e+00])
# print(via.rf_Even2Right[1][0])
print(rightPose1[0])
# print(right_leg_chain.inverse_kinematics(rightPose1[0], initial_position=via.rf_Even2Right[1][0]), right_leg_chain.inverse_kinematics(rightPose1[1]), right_leg_chain.inverse_kinematics(rightPose1[2]), right_leg_chain.inverse_kinematics(rightPose1[3]), sep='\n\n')


# # Define initial and final positions
# # Left Leg Trajectories
# initial_position_left = np.array([0.05, 0.0, -0.35])
# final_position_left = np.array([0.05, 0.1, -0.325])

# initial_position_left_2 = np.array([0.05, 0.1, -0.325])
# final_position_left_2 = np.array([0.05, 0.1, -0.3])

# initial_position_left_3 = np.array([0.05, 0.1, -0.3])
# final_position_left_3 = np.array([0.05, 0.0, -0.3])

# initial_position_left_3 = np.array([0.05, 0.0, -0.3])
# final_position_left_3 = np.array([0.05, -0.1, -0.3])

# initial_position_left_4 = np.array([0.05, -0.1, -0.3])
# final_position_left_4 = np.array([0.05, -0.1, -0.325])

# initial_position_left_5 = np.array([0.05, -0.1, -0.325])
# final_position_left_5 = np.array([0.05, 0.0, -0.35])


# # Right Leg Trajectories
# initial_position_right = np.array([-0.05, 0.0, -0.3])
# final_position_right = np.array( [-0.05, -0.1, -0.3])

# initial_position_right_2 = np.array([-0.05, -0.1, -0.3])
# final_position_right_2 = np.array([-0.05, -0.1, -0.325])

# initial_position_right_3 = np.array([-0.05, -0.1, -0.325])
# final_position_right_3 = np.array([-0.05, 0.0, -0.35])

# initial_position_right_3 = np.array([-0.05, 0.0, -0.35])
# final_position_right_3 = np.array([-0.05, 0.1, -0.325])

# initial_position_right_4 = np.array([-0.05, 0.1, -0.325])
# final_position_right_4 = np.array([-0.05, 0.1, -0.3])

# initial_position_right_5 = np.array([-0.05, 0.1, -0.3])
# final_position_right_5 = np.array([-0.05, 0.0, -0.3])

# # Define initial and final velocities and accelerations
# initial_velocity = np.array([0, 0, 0])
# final_velocity = np.array([0, 0, 0])
# initial_acceleration = np.array([0, 0, 0])
# final_acceleration = np.array([0, 0, 0])

# def compute_quintic_coefficients(p0, pf, v0, vf, a0, af, T):
#     # Construct the matrix and vector for boundary conditions
#     M = np.array([
#         [1, 0, 0, 0, 0, 0],
#         [0, 1, 0, 0, 0, 0],
#         [0, 0, 2, 0, 0, 0],
#         [1, T, T**2, T**3, T**4, T**5],
#         [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
#         [0, 0, 2, 6*T, 12*T**2, 20*T**3]
#     ])
    
#     b = np.array([p0, v0, a0, pf, vf, af])
    
#     # Solve for the coefficients
#     coefficients = np.linalg.solve(M, b)
#     return coefficients

# def quintic_trajectory(coefficients, t):
#     # Compute the position using the quintic polynomial
#     a0, a1, a2, a3, a4, a5 = coefficients
#     return a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5

# # Calculate coefficients for the left and right leg
# T = 1  # Normalized total duration
# coeffs_left_1 = [compute_quintic_coefficients(initial_position_left[i], final_position_left[i],
#                                             initial_velocity[i], final_velocity[i],
#                                             initial_acceleration[i], final_acceleration[i], T)
#                for i in range(3)]

# coeffs_left_2 = [compute_quintic_coefficients(initial_position_left_2[i], final_position_left_2[i],
#                                             initial_velocity[i], final_velocity[i],
#                                             initial_acceleration[i], final_acceleration[i], T)
#                for i in range(3)]

# coeffs_left_3 = [compute_quintic_coefficients(initial_position_left_3[i], final_position_left_3[i],
#                                             initial_velocity[i], final_velocity[i],
#                                             initial_acceleration[i], final_acceleration[i], T)
#                for i in range(3)]

# coeffs_left_4 = [compute_quintic_coefficients(initial_position_left_4[i], final_position_left_4[i],
#                                             initial_velocity[i], final_velocity[i],
#                                             initial_acceleration[i], final_acceleration[i], T)
#                for i in range(3)]

# coeffs_left_5 = [compute_quintic_coefficients(initial_position_left_5[i], final_position_left_5[i],
#                                             initial_velocity[i], final_velocity[i],
#                                             initial_acceleration[i], final_acceleration[i], T)
#                for i in range(3)]


# # Right Coefficients
# coeffs_right_1 = [compute_quintic_coefficients(initial_position_right[i], final_position_right[i],
#                                              initial_velocity[i], final_velocity[i],
#                                              initial_acceleration[i], final_acceleration[i], T)
#                 for i in range(3)]

# coeffs_right_2 = [compute_quintic_coefficients(initial_position_right_2[i], final_position_right_2[i],
#                                              initial_velocity[i], final_velocity[i],
#                                              initial_acceleration[i], final_acceleration[i], T)
#                 for i in range(3)]

# coeffs_right_3 = [compute_quintic_coefficients(initial_position_right_3[i], final_position_right_3[i],
#                                              initial_velocity[i], final_velocity[i],
#                                              initial_acceleration[i], final_acceleration[i], T)
#                 for i in range(3)]

# coeffs_right_4 = [compute_quintic_coefficients(initial_position_right_4[i], final_position_right_4[i],
#                                              initial_velocity[i], final_velocity[i],
#                                              initial_acceleration[i], final_acceleration[i], T)
#                 for i in range(3)]

# coeffs_right_5 = [compute_quintic_coefficients(initial_position_right_5[i], final_position_right_5[i],
#                                              initial_velocity[i], final_velocity[i],
#                                              initial_acceleration[i], final_acceleration[i], T)
#                 for i in range(3)]

# # Initialize the plot
# fig, ax = plot_utils.init_3d_figure()
# ax.set_xlim([-0.3, 0.3])
# ax.set_ylim([-0.75, 0.75])
# ax.set_zlim([-0.6, 0.2])

# # Store the start time for real-time calculation
# start_time = time.time()
# total_duration = 5.0  # Duration of one cycle of the trajectory in seconds
# speed_factor = 1.0    # Adjust this to control the speed

# def update_plot(frame):
#     elapsed_time = time.time() - start_time
#     t_normalized = (elapsed_time * speed_factor) % total_duration
    
#     # Determine which segment we're in (assuming equal duration for each segment)
#     segment_duration = total_duration / 5.0
#     segment_index = int(t_normalized // segment_duration)
#     t_segment = (t_normalized % segment_duration) / segment_duration

#     # Select the appropriate coefficients based on the segment index
#     if segment_index == 0:
#         coeffs_left = coeffs_left_1
#         coeffs_right = coeffs_right_1
#     elif segment_index == 1:
#         coeffs_left = coeffs_left_2
#         coeffs_right = coeffs_right_2
#     elif segment_index == 2:
#         coeffs_left = coeffs_left_3
#         coeffs_right = coeffs_right_3
#     elif segment_index == 3:
#         coeffs_left = coeffs_left_4
#         coeffs_right = coeffs_right_4
#     elif segment_index == 4:
#         coeffs_left = coeffs_left_5
#         coeffs_right = coeffs_right_5

#     # Compute the positions using the quintic polynomial for the current segment
#     target_position_left = np.array([quintic_trajectory(coeffs_left[i], t_segment) for i in range(3)])
#     target_position_right = np.array([quintic_trajectory(coeffs_right[i], t_segment) for i in range(3)])
    
#     # Calculate inverse kinematics
#     ik_solution_left = left_leg_chain.inverse_kinematics(target_position_left)
#     ik_solution_right = right_leg_chain.inverse_kinematics(target_position_right)
    
#     print("Left IK Solution:", ik_solution_left)
#     print("Right IK Solution:", ik_solution_right)
    
#     ax.cla()
#     left_leg_chain.plot(ik_solution_left, ax)
#     right_leg_chain.plot(ik_solution_right, ax)
#     ax.set_xlim([-0.3, 0.3])
#     ax.set_ylim([-0.75, 0.75])
#     ax.set_zlim([-0.6, 0.2])

#     return ax

# ani = FuncAnimation(fig, update_plot, frames=None, interval=100, blit=False)

# plt.show()


