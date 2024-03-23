import sys, time, math

import numpy as np 
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.trajPlannerPose import TrajPlannerPose
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
from backend.KoalbyHumanoid.Config import Joints
import matplotlib.pyplot as plt
from backend.KoalbyHumanoid.Plotter import Plotter

# Edit to declare if you are testing the sim or the real robot
is_real = False

robot = Robot(is_real)

print("Setup Complete")

robot.motors[0].target = (math.radians(-20), 'P')
robot.motors[1].target = (math.radians(90), 'P')
robot.motors[3].target = (math.radians(110), 'P')

robot.motors[5].target = (math.radians(20), 'P')
robot.motors[6].target = (math.radians(-90), 'P')
robot.motors[8].target = (math.radians(-110), 'P')

robot.motors[17].target = (math.radians(20), 'P')
robot.motors[18].target = (math.radians(-40), 'P')
robot.motors[19].target = (math.radians(-20), 'P')
robot.motors[22].target = (math.radians(-20), 'P')
robot.motors[23].target = (math.radians(40), 'P')
robot.motors[24].target = (math.radians(20), 'P')


prevTime = time.time()
simStartTime = time.time()

while time.time() - simStartTime < 8:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()

## EVEN TO RIGHT FOOT FORWARD
right_setPointTimes = [[0,0,0], [1,1,1], [2,2,2], [3,3,3], [4,4,4]]
right_angles = [[math.radians(20),math.radians(-40), math.radians(-20)], 
                [0.543488, -1.045386, -0.501898], 
                [0.721075, -1.229439, -0.508364], 
                [0.675096, -1.033531, -0.358434], 
                [0.467840, -0.682081, -0.214241]]
right_vels = [[0,0,0],[0.05, -0.1, -0.05], [0.05, 0.05, 0], [-0.05, 0.1, 0.05],[0,0,0]]
right_accels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

left_setPointTimes = [[0,0,0],[4,4,4]]
left_angles = [[math.radians(-20),math.radians(40), math.radians(20)],
               [-0.295607, 0.682080, 0.386473]]
left_vels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
left_accels = [[0,0,0],[0,0,0],[0,0,0]]

right_tj = TrajPlannerTime(right_setPointTimes, right_angles, right_vels, right_accels)
left_tj = TrajPlannerTime(left_setPointTimes, left_angles, left_vels, left_accels)
state = 0
startTime = time.time()
lastTime = startTime
print("State = 0")
while True:
    right_points = right_tj.getQuinticPositions(time.time() - startTime)
    left_points = left_tj.getQuinticPositions(time.time() - startTime)
    robot.motors[17].target = (right_points[0], 'P')
    robot.motors[18].target = (right_points[1], 'P')
    robot.motors[19].target = (right_points[2], 'P')
    robot.motors[22].target = (left_points[0], 'P')
    robot.motors[23].target = (left_points[1], 'P')
    robot.motors[24].target = (left_points[2], 'P')
    robot.IMUBalance(0, 0)
    robot.moveAllToTarget()
    match(state):
        case 0: 
            if(time.time() - startTime >= 4):
                ##ANGLES TO GO RIGHT TO LEFT
                right_setPointTimes = [[0,0,0], [1,1,1], [2,2,2], [3,3,3], [4,4,4]]
                right_angles =  [[0.467840, -0.682081, -0.214241], 
                                [0.415031, -0.701593, -0.286562], 
                                [0.349066, -0.698132, -0.349066], 
                                [0.270305, -0.671400, -0.401095], 
                                [0.177871, -0.618870, -0.440999]]
                right_vels = [[0,0,0],[-0.1, 0, 0], [-0.1, 0.1, 0] ,[-0.1, 0, 0],[0,0,0]]
                right_accels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

                left_setPointTimes = [[0,0,0], [1,1,1], [2,2,2], [3,3,3], [4,4,4]]
                left_angles = [[-0.295607, 0.682080, 0.386473], 
                                [-0.488609, 1.033534, 0.544925], 
                                [-0.741970, 1.227072, 0.485102], 
                                [-0.737818, 0.988358, 0.250540], 
                                [-0.514302, 0.618868, 0.104566]]
                left_vels = [[0,0,0],[-0.05, 0.1, 0.05],[-0.1, -0.1, 0],[0.05, -0.1, -0.05],[0,0,0]]
                left_accels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

                right_tj = TrajPlannerTime(right_setPointTimes, right_angles, right_vels, right_accels)
                left_tj = TrajPlannerTime(left_setPointTimes, left_angles, left_vels, left_accels)
                startTime = time.time()
                print("Right To Left")
                state = 1
            
        case 1: ##Right to Left
            if(time.time() - startTime >= 4):
                ##ANGLES TO GO LEFT TO RIGHT
                right_setPointTimes = [[0,0,0], [1,1,1], [2,2,2], [3,3,3], [4,4,4]]
                right_angles =  [[0.177871, -0.618870, -0.440999], 
                                [0.373971, -0.988358, -0.614387], 
                                [0.646288, -1.227072, -0.580784], 
                                [0.675096, -1.033531, -0.358434], 
                                [0.467840, -0.682081, -0.214241]]
                right_vels = [[0,0,0],[0.05, -0.1, -0.05],[0.1, 0.1, 0],[-0.05, 0.1, 0.05],[0,0,0]]
                right_accels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

                left_setPointTimes = [[0,0,0], [1,1,1], [2,2,2], [3,3,3], [4,4,4]]
                left_angles = [[-0.514302, 0.618868, 0.104566], 
                                [-0.481090, 0.671404, 0.190314], 
                                [-0.432509, 0.698132, 0.265623], 
                                [-0.370454, 0.701594, 0.331139], 
                                [-0.295607, 0.682080, 0.386473]]
                left_vels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
                left_accels = [[0,0,0],[0,0,0],[0.1, -0.1, 0],[0,0,0],[0,0,0]]
                right_tj = TrajPlannerTime(right_setPointTimes, right_angles, right_vels, right_accels)
                left_tj = TrajPlannerTime(left_setPointTimes, left_angles, left_vels, left_accels)
                print("Left To Right")
                startTime = time.time()
                state = 0