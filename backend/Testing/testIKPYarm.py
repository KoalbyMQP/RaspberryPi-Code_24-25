import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils
import sys, time, math, array
import numpy as np
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
from backend.DemoScripts import finlyViaPoints as via
left_leg_chain = Chain.from_urdf_file(
    "backend/Testing/FullAssem11_24_23(1).urdf",
    base_elements=['LeftShoulder1', 'LeftShoulderRotator']
)
# Edit to declare if you are testing the sim or the real robot
is_real = False
robot = Robot(is_real)
print("Setup Complete")
##Joint angles: [-2.57202856e-15  6.37722176e-02  8.34481233e-01  3.97048734e-01
 ## 9.51953940e-02  0.00000000e+00]
#Starting Agnles
##robot.motors[0].target = (math.radians(-20), 'P')
##robot.motors[1].target = (math.radians(90), 'P')
##robot.motors[3].target = (math.radians(110), 'P')
##robot.motors[5].target = (math.radians(10), 'P')
##robot.motors[6].target = (math.radians(-90), 'P')
robot.motors[5].target = (-2.57202856e-15, 'P')
robot.motors[6].target = (6.37722176e-02, 'P')
robot.motors[7].target = (8.34481233e-01, 'P')
robot.motors[8].target = (math.radians(-90), 'P')
robot.motors[9].target = (9.51953940e-02, 'P')
prevTime = time.time()
simStartTime = time.time()
while time.time() - simStartTime < 2:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()
## EVEN TO RIGHT FOOT FORWARD
# rArm_tj = TrajPlannerTime(via.ra_grabCart[0], via.ra_grabCart[1], via.ra_grabCart[2], via.ra_grabCart[3])
lArm_tj = TrajPlannerTime(via.leftArmTraj[0], via.leftArmTraj[1], via.leftArmTraj[2], via.leftArmTraj[3])
state = 0
startTime = time.time()
print("State = 0")
##l_points = lArm_tj.getQuinticPositions(time.time() - startTime)
## Grabbing cart
##l_points=np.array([.1, 0 ,0, 0, 0,  0])
while time.time() - startTime < 8:
    target_positions = lArm_tj.getQuinticPositions(time.time() - startTime)
  ##  l_points=array([-0.45110797,-1.570796,0.,-1.133303,-0.01361496])
    ik_solution = left_leg_chain.inverse_kinematics(target_positions)
    print(ik_solution)
    robot.motors[5].target = (ik_solution[0], 'P')
    robot.motors[6].target = (ik_solution[1], 'P')
    robot.motors[7].target = (ik_solution[2], 'P')
    robot.motors[8].target = (ik_solution[3], 'P')
    robot.motors[9].target = (ik_solution[4], 'P')
robot.IMUBalance(0, 0)
robot.moveAllToTarget()