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
    "backend/Testing/FullAssem11_24_23ch.urdf",
    base_elements=['LeftShoulder2', 'LeftShoulderRotator']
)


# Edit to declare if you are testing the sim or the real robot
is_real = False


robot = Robot(is_real)


print("Setup Complete")


# positions


final_position=np.array([0.07089,  -0.26614,  .68187-.66334])


motor_angle_joint=np.array([0,0,0,0,0])


#Starting Agnles


robot.motors[5].target = (math.radians(90), 'P')
robot.motors[6].target = (math.radians(90), 'P')
robot.motors[7].target = (math.radians(90), 'P')
robot.motors[8].target = (math.radians(90), 'P')
robot.motors[9].target = (math.radians(90), 'P')

ik_solution_2=np.array([0,0,0,0,0,0])
prevTime = time.time()


simStartTime = time.time()


while time.time() - simStartTime < 2:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()

"""
final_position_joint=left_leg_chain.inverse_kinematics(final_position)






arm_joint = [
    [[0,0,0,0,0], [20,20,20,20,20]],
    [[0.000000, 0, 0.000000, 0, 0.000000],
        [final_position_joint[1], final_position_joint[2], final_position_joint[3],final_position_joint[4],final_position_joint[5]]],
    [ [0,0,0,0,0], [0,0,0,0,0]],
    [[0,0,0,0,0], [0,0,0,0,0]]
]
## EVEN TO RIGHT FOOT FORWARD
# rArm_tj = TrajPlannerTime(via.ra_grabCart[0], via.ra_grabCart[1], via.ra_grabCart[2], via.ra_grabCart[3])
lArm_tj_joint = TrajPlannerTime(arm_joint[0], arm_joint[1], arm_joint[2], arm_joint[3])


state = 0


startTime = time.time()




while time.time() - startTime < 20:
    target_position_joint = lArm_tj_joint.getQuinticPositions(time.time() - startTime)
    motor_angle_joint=target_position_joint
    robot.motors[5].target = (motor_angle_joint[0], 'P')
    robot.motors[6].target = (motor_angle_joint[1], 'P')
    robot.motors[7].target = (motor_angle_joint[2], 'P')
    robot.motors[8].target = (motor_angle_joint[3], 'P')
    robot.motors[9].target = (motor_angle_joint[4], 'P')
    robot.IMUBalance(0, 0)
    robot.moveAllToTarget()
    pain=left_leg_chain.forward_kinematics(np.concatenate(([0], motor_angle_joint)))
    print(pain)






    if abs(motor_angle_joint[0]-final_position_joint[1])<.2 and abs(motor_angle_joint[1]-final_position_joint[2])<.2 and state == 0:
          current_time=time.time() - startTime
          startTime=-21
          print("got here")




motor_angle_joint=np.concatenate(([0], motor_angle_joint))
ik_solution_2=motor_angle_joint
target_position_fk = left_leg_chain.forward_kinematics(motor_angle_joint)
arm_task= [[[0,0,0], [3,3,3]],
            [[target_position_fk[0][3], target_position_fk[1][3], target_position_fk[2][3]],
            [final_position[0], final_position[1],final_position[2]]],
            [[0,0,0], [0,0,0]],
            [[0,0,0], [0,0,0]]]
lArm_tj_task = TrajPlannerTime(arm_task[0], arm_task[1], arm_task[2], arm_task[3])
startTime = time.time()


while time.time() - startTime < 3:
     
        target_position_task = lArm_tj_task.getQuinticPositions(time.time() - startTime)
        target_position_2 = np.array([(target_position_task[0] ), (target_position_task[1]), (target_position_task[2])])
        ik_solution = left_leg_chain.inverse_kinematics(target_position_2, initial_position=ik_solution_2)
        ik_solution_2=ik_solution
        motor_angle_task=ik_solution
        robot.motors[5].target = (motor_angle_task[1], 'P')
        robot.motors[6].target = (motor_angle_task[2], 'P')
        robot.motors[7].target = (motor_angle_task[3], 'P')
        robot.motors[8].target = (motor_angle_task[4], 'P')
        robot.motors[9].target = (motor_angle_task[5], 'P')
        print(target_position_2)


        robot.IMUBalance(0, 0)
        robot.moveAllToTarget()
        """