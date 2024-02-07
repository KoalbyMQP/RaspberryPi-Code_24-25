import sys, time, math

import numpy as np 
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
<<<<<<< Updated upstream
from backend.KoalbyHumanoid import trajPlanner
=======
from backend.KoalbyHumanoid.trajPlannerPose import TrajPlannerPose
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
>>>>>>> Stashed changes
from backend.KoalbyHumanoid.Config import Joints
import matplotlib.pyplot as plt
from backend.KoalbyHumanoid.Plotter import Plotter

# Edit to declare if you are testing the sim or the real robot
is_real = False

robot = Robot(is_real)

print("Setup Complete")

<<<<<<< Updated upstream
setPoints = [[math.radians(20), math.radians(-40), math.radians(-20)], [math.radians(36.9495), math.radians(-56.3311), math.radians(-19.3816)], [math.radians(30.1578), math.radians(-32.3075), math.radians(-2.1497)]]
tj = trajPlanner.TrajPlannerNew(setPoints)
traj = tj.getCubicTraj(10, 10)
plotter = Plotter(10, False)

robot.motors[1].target = (math.radians(80), 'P')
robot.motors[3].target = (math.radians(90), 'P')
robot.motors[6].target = (math.radians(-80), 'P')


robot.motors[17].target = (math.radians(20), 'P')
robot.motors[18].target = (math.radians(-40), 'P')
robot.motors[19].target = (math.radians(-20), 'P')

robot.motors[22].target = (math.radians(-20), 'P')
robot.motors[23].target = (math.radians(40), 'P')
robot.motors[24].target = (math.radians(20), 'P')

=======
# startStepKickRight = math.radians(20) # deg
# startStepKneeRight = math.radians(-40)
# startStepAnkleRight = math.radians(-20)
# startStepKickLeft = math.radians(-20) # deg
# startStepKneeLeft = math.radians(40)
# startStepAnkleLeft = math.radians(20)

# endStepKickRight = math.radians(20) # deg
# endStepKneeRight = math.radians(-40)
# endStepAnkleRight = math.radians(-20)
# endStepKickLeft = math.radians(-20) # deg
# endStepKneeLeft = math.radians(40)
# endStepAnkleLeft = math.radians(20)

# setPointTimes = [[0, 0, 0], [5, 15, 8], [9, 20, 10]]
# angles = [[0, 5, 2], [20, 9, 12], [5, 0, 6]]
# vels = [[0, 2, 1], [5,5,7], [2,0,1]]
# accels = [[0,0,0], [0,0,0], [0,0,0]]

setPointTimes = [[0,0],[5,5],[10,10]]
angles = [[0,0],[math.radians(90),math.radians(-90)],[0,0]]
vels = [[0,0],[0,0],[0,0]]
accels = [[0,0],[0,0],[0,0]]

tj = TrajPlannerTime(setPointTimes,angles,vels,accels)

plotter = Plotter(10, False)

# robot.motors[1].target = (math.radians(80), 'P')
# robot.motors[6].target = (math.radians(-80), 'P')


# robot.motors[17].target = (math.radians(20), 'P')
# robot.motors[18].target = (math.radians(-40), 'P')
# robot.motors[19].target = (math.radians(-20), 'P')
# robot.motors[22].target = (math.radians(-20), 'P')
# robot.motors[23].target = (math.radians(40), 'P')
# robot.motors[24].target = (math.radians(20), 'P')
# robot.motors[3].target = math.radians(90)
# robot.motors[8].target = math.radians(90)
# robot.motors[14].target = (0, 'P')
# robot.motors[17].target = math.radians(90)
# robot.motors[22].target = math.radians(90)

#robot.motors[17].target = math.radians(-45)
#robot.motors[22].target = math.radians(45)
prevTime = time.time()

#robot.motors[0].target = -math.radians(90)
>>>>>>> Stashed changes

simStartTime = time.time()

while time.time() - simStartTime < 5:
    time.sleep(0.01)
<<<<<<< Updated upstream
    # robot.updateRobotCoM()
    # robot.updateBalancePoint()
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()
    
while True:
    startTime = time.time()
    for point in traj:
        time.sleep(0.01)
        robot.updateRobotCoM()
        
        robot.motors[17].target = (point[1], 'P')
        robot.motors[18].target = (point[2], 'P')
        robot.motors[19].target = (point[3], 'P')

        robot.moveAllToTarget()
        while time.time() - startTime < point[0]:
            time.sleep(0.01)
            robot.updateRobotCoM() 
            robot.moveAllToTarget()
=======
    robot.updateRobotCoM()
    robot.updateBalancePoint()
    # robot.IMUBalance(0,0)
    # print(robot.balancePoint - robot.CoM, robot.VelBalance())
    robot.moveAllToTarget()

startTime = time.time()
while time.time() - startTime < 30:
    # errorData = []
    # timeData = []
    time.sleep(0.01)
    # print(robot.locatePolygon())
    robot.updateRobotCoM()
    # plotting stuff
    robot.updateBalancePoint()
    robot.IMUBalance(0,0)
    points = tj.getQuinticPositions(time.time() - startTime)

    robot.motors[0].target = (points[1], 'P')
    robot.motors[5].target = (points[2], 'P')

    robot.moveAllToTarget()
>>>>>>> Stashed changes
