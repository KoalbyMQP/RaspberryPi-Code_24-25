import numpy as np
import matplotlib.pyplot as plt




from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils


import sys, time, math, array


import numpy as np
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime

from backend.Testing import finlyViaPoints as via



# Creating URDF chain for left arm 
# left_arm_chain = Chain.from_urdf_file(
#     "backend/Testing/FinleyJNEWARMS_2024_straight_4.urdf",
#     base_elements=['shoulder1_left', 'shoulder1_left'],
#     active_links_mask=[False, True, True, True, True, True, True]  
# )

# # creating URDF chain for camera
# camera = Chain.from_urdf_file(
#     "backend/Testing/\FinleyJNEWARMS_2024_straight_4.urdf",
#     base_elements=['neck', 'neck']   
# )

#forward kinematics for camera chain
# camera_angles=np.array([0,0,0,0])
# camera_frame_transformation=camera.forward_kinematics(camera_angles)


# Edit to declare if you are testing the sim or the real robot
is_real = True
robot = Robot(is_real)
print("Setup Complete")


# positions


final_position=np.array([0.07089,  -0.26614,  .68187])



#Starting Angles

# robot.motors[25].target = (math.radians(30), 'P')
# robot.motors[26].target = (math.radians(10), 'P')


robot.motors[27].target = (math.radians(-60), 'P')
robot.motors[5].target = (math.radians(0), 'P')
robot.motors[6].target = (math.radians(0), 'P')
robot.motors[7].target = (math.radians(0), 'P')
robot.motors[8].target = (math.radians(0), 'P')
robot.motors[9].target = (math.radians(0), 'P')
robot.motors[10].target = (math.radians(0), 'P')



ik_solution_2=np.array([0,0,0,0,0,0,0])


# centering all angles to zero
prevTime = time.time()
simStartTime = time.time()
while time.time() - simStartTime < 2:
    time.sleep(0.01)
    #robot.IMUBalance(0,0)
    robot.moveAllToTarget()


# conversion of final points from camera coordinate systm to rorbot coordinate system 
# final_points=np.array([0, .3, 0])
# B=np.array([[final_points[0]],[final_points[1]],[final_points[2]],[1]])
# A= camera_frame_transformation
# final_points=np.array([0,0, 0])
# C = np.dot(A, B)




leftArmTraj = [
    [[0,0,0], [20,20,20]],
    [[.49076,  -.08197, .76541],
   [C[0],  C[1], C[2]] ],
    [[0,0,0], [0,0,0]],
    [[0,0,0], [0,0,0]]
]

# final_position=left_arm_chain.forward_kinematics(ik_solution_2)

# lArm_tj_joint = TrajPlannerTime(leftArmTraj[0], leftArmTraj[1], leftArmTraj[2], leftArmTraj[3])




startTime = time.time()

# while time.time() - startTime < 20:
        
     
#         target_position_task = lArm_tj_joint.getQuinticPositions(time.time() - startTime)
#         target_position_2 = np.array([(target_position_task[0]), (target_position_task[1]), (target_position_task[2])])
#         ik_solution = left_arm_chain.inverse_kinematics(target_position_2, initial_position=ik_solution_2 )
#         ik_solution_2=ik_solution
#         motor_angle_task=ik_solution

#         robot.motors[5].target = (motor_angle_task[1], 'P')
#         robot.motors[6].target = (motor_angle_task[2], 'P')
#         robot.motors[7].target = (motor_angle_task[3], 'P')
#         robot.motors[8].target = (motor_angle_task[4], 'P')
#         robot.motors[9].target = (motor_angle_task[5], 'P')
        
#         print(motor_angle_task)

#         robot.IMUBalance(0, 0)
#         robot.moveAllToTarget()
 