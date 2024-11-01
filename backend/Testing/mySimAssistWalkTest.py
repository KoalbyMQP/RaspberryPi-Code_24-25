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

def update_trajectory_and_control(robot, n_points, right_points, left_points, start_time):
    currentTime = time.time() - start_time

    # Right Leg
    robot.motors[15].target = (0, 'P')
    robot.motors[16].target = (right_points[1], 'P')
    robot.motors[17].target = (right_points[2], 'P')
    robot.motors[18].target = (right_points[3], 'P')
    robot.motors[19].target = (right_points[4], 'P')

    # Left Leg
    robot.motors[20].target = (0, 'P')
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

    y_avg_position = -0.385
    z_avg_position = 0.0
    y_scale = 2
    z_scale = 1
    # target_positions_left = np.array([
    #     [0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.0],
    #     [0.04,  -0.285 , -0.05],
    #     [0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.1],
    #     [0.04,  -0.385 , -0.1]
    # ])
    # target_positions_right = np.array([
    #     [0.04,  y_avg_position + y_scale*0.050 , z_avg_position + z_scale*0.1],
    #     [0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.1],
    #     [0.04,  y_avg_position + y_scale*0.100 , z_avg_position + z_scale*0.2],
    #     [0.04,  y_avg_position + y_scale*0.050 , z_avg_position + z_scale*0.1]
    # ])

    target_positions_left = np.array([
        [0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.0],
        [0.04,  -0.285 , -0.05],
        [0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.1],
        [0.04,  -0.385 , -0.1]
    ])
    target_positions_right = np.array([
        [0.04,  y_avg_position + y_scale*0.050 , z_avg_position + z_scale*0.1],
        [0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.1],
        [0.04,  y_avg_position + y_scale*0.100 , z_avg_position + z_scale*0.2],
        [0.04,  y_avg_position + y_scale*0.050 , z_avg_position + z_scale*0.1]
    ])

    left_base = [0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.0]
    left_center_top = [0.04, y_avg_position + y_scale * 0.05, z_avg_position + z_scale*0.1]
    left_forward_top = [0.04,  y_avg_position + y_scale*0.1 , z_avg_position + z_scale*0.2]
    right_back_flat = [0.04,  y_avg_position + y_scale*0.1 , z_avg_position + z_scale*0.2] #Doesn't work, the leg peaks, if z goes below 0

    right_base = [0.04,  y_avg_position + y_scale*0.050 , z_avg_position + z_scale*0.1]
    right_center_top = [0.04,  y_avg_position + y_scale*0.000 , z_avg_position + z_scale*0.1]
    right_forward_top = [0.04,  y_avg_position + y_scale*0.1 , z_avg_position + z_scale*0.2]
    # left_back_flat -> Wouldn't work cause leg peaks, if z goes negative


    trajectory_left = []
    trajectory_right = []

    trajectory_left = TrajPlannerTime([[0,0,0], [5,5,5], [10,10,10], [15, 15,15]], target_positions_left, [[0,0,0],[0,0,0],[0,0,0],[0,0,0]], [[0,0,0],[0,0,0],[0,0,0],[0,0,0]])
    trajectory_right = TrajPlannerTime([[0,0,0], [5,5,5], [10,10,10], [15,15,15]], target_positions_right, [[0,0,0],[0,0,0],[0,0,0],[0,0,0]], [[0,0,0],[0,0,0],[0,0,0],[0,0,0]])

    n_points = 100
    t_left = np.linspace(0, 1, len(target_positions_left))
    splines_left = [CubicSpline(t_left, target_positions_left[:, dim]) for dim in range(3)]
    # trajectory_left = np.vstack([spline(np.linspace(0, 1, n_points)) for spline in splines_left]).T

    t_right = np.linspace(0, 1, len(target_positions_right))
    splines_right = [CubicSpline(t_right, target_positions_right[:, dim]) for dim in range(3)]
    # trajectory_right = np.vstack([spline(np.linspace(0, 1, n_points)) for spline in splines_right]).T

    left_points = []
    right_points = []

    target_orientation = [0, 0, 0.05]

    state = 0
    initial_position_left = [-0.9574527745422127, -6.677834432572358, 3.362498794795048, 0, 0]

    initial_position_right = [-0.8818984203844015, 2.090247231413886, -7.376601873229693e-06, 0, 0]


    while True:
        start_time = time.time()
        count = 1
        while time.time() - start_time < 15:
                left_quintic = trajectory_left.getQuinticPositions(time.time() - start_time)
                right_quintic = trajectory_right.getQuinticPositions(time.time() - start_time)
            
                target_position_left = left_quintic
                ik_solution_left = left_leg_chain.inverse_kinematics(
                    target_position_left,
                    target_orientation=target_orientation,
                    orientation_mode='Z' 
                )
                # print(ik_solution_left)
                   
                target_position_right = right_quintic
                ik_solution_right = right_leg_chain.inverse_kinematics(
                    target_position_right,
                    target_orientation=target_orientation,
                    orientation_mode='Z'
                )
                print(left_quintic)
                print(ik_solution_left)
                
                update_trajectory_and_control(robot, n_points, ik_solution_right, ik_solution_left, start_time)

                count += 1

            # if time.time() - start_time >= 3:
            #     state = (state + 1) % 2  
            #     start_time = time.time()

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