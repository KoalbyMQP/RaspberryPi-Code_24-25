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

plotter = Plotter(10, False)

robot.motors[1].target = (math.radians(80), 'P')
robot.motors[6].target = (math.radians(-80), 'P')


robot.motors[17].target = (math.radians(20), 'P')
robot.motors[18].target = (math.radians(-40), 'P')
robot.motors[19].target = (math.radians(-20), 'P')
robot.motors[22].target = (math.radians(-20), 'P')
robot.motors[23].target = (math.radians(40), 'P')
robot.motors[24].target = (math.radians(20), 'P')
robot.motors[3].target = (math.radians(90), 'P')
robot.motors[8].target = (math.radians(-90), 'P')
robot.motors[14].target = (0, 'P')

prevTime = time.time()
simStartTime = time.time()

while time.time() - simStartTime < 2:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()


startTime = time.time()
setPointTimes = [[0,0],[5,5],[10,10]]
angles = [[0,0],[math.radians(90),math.radians(-90)],[0,0]]
vels = [[0,0],[0,0],[0,0]]
accels = [[0,0],[0,0],[0,0]]

tj = TrajPlannerTime(setPointTimes,angles,vels,accels)

while time.time() - startTime < 30:
    time.sleep(0.01)
    robot.updateRobotCoM()
    robot.updateBalancePoint()
    robot.IMUBalance(0,0)
    points = tj.getQuinticPositions(time.time() - startTime)

    robot.motors[0].target = (points[1], 'P')
    robot.motors[5].target = (points[2], 'P')

    robot.moveAllToTarget()


#State Machine to move through walking gait

state = 0
while True:
    robot.moveAllToTarget()
    robot.IMUBalance(0,0)
    match(state):
        case 0:
            points = tj.getQuinticPositions(time.time() - startTime)
            robot.motors[0].target = (points[1], 'P')
            robot.motors[5].target = (points[2], 'P')
            robot.moveAllToTarget()
    