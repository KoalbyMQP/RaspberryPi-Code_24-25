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




left_leg_chain = Chain.from_urdf_file(
    "backend/Testing/FullAssemFin.urdf",
    base_elements=['left_shoulder1', 'left_shoulder_twist']
)


# Edit to declare if you are testing the sim or the real robot
is_real = False


robot = Robot(is_real)


print("Setup Complete")


# positions


final_position=np.array([0.07089,  -0.26614,  .68187])


motor_angle_joint=np.array([0,0,0,0,0])


#Starting Agnles

robot.motors[5].target = (math.radians(90), 'P')
robot.motors[6].target = (math.radians(90), 'P')
robot.motors[7].target = (math.radians(0), 'P')
robot.motors[8].target = (math.radians(0), 'P')
robot.motors[9].target = (math.radians(0), 'P')
robot.motors[10].target = (math.radians(0), 'P')



ik_solution_2=np.array([0,0,0,0,0,0,0])
prevTime = time.time()


simStartTime = time.time()


while time.time() - simStartTime < 2:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()



leftArmTraj = [
    [[0,0,0], [20,20,20]],
    [[.49513,  -.04901, .28036],
     [.12938,  -.03073, .60344]],
    [[0,0,0], [0,0,0]],
    [[0,0,0], [0,0,0]]
]

# get=np.array([.41263 +.02042,  -.01513 +.02647, .26667 -.02361])
# print(get)
final_position=left_leg_chain.forward_kinematics(ik_solution_2)
print(final_position)

# arred=np.array([ -3.29889867e-14,  1.33762582e+00 , 5.24790953e-01 ,-2.16606497e-02, 1.35522250e-02, -2.52929247e+00 , 0.00000000e+00])
# help=left_leg_chain.forward_kinematics(arred)
# print(help)

## EVEN TO RIGHT FOOT FORWARD
# rArm_tj = TrajPlannerTime(via.ra_grabCart[0], via.ra_grabCart[1], via.ra_grabCart[2], via.ra_grabCart[3])
lArm_tj_joint = TrajPlannerTime(leftArmTraj[0], leftArmTraj[1], leftArmTraj[2], leftArmTraj[3])


state = 0


startTime = time.time()


print("phase 2")

while time.time() - startTime < 20:
     
        target_position_task = lArm_tj_joint.getQuinticPositions(time.time() - startTime)
        target_position_2 = np.array([(target_position_task[0]), (target_position_task[1]), (target_position_task[2])])
        ik_solution = left_leg_chain.inverse_kinematics(target_position_2, initial_position=ik_solution_2 )
        ik_solution_2=ik_solution
        motor_angle_task=ik_solution

        robot.motors[5].target = (motor_angle_task[1], 'P')
        robot.motors[6].target = (motor_angle_task[2], 'P')
        robot.motors[7].target = (motor_angle_task[3], 'P')
        robot.motors[8].target = (motor_angle_task[4], 'P')
        robot.motors[9].target = (motor_angle_task[5], 'P')
        
        print(motor_angle_task)

        robot.IMUBalance(0, 0)
        robot.moveAllToTarget()
 