import sys, time, math 
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid import trajPlanner
from backend.KoalbyHumanoid.Config import Joints
import matplotlib.pyplot as plt
from backend.KoalbyHumanoid.Plotter import Plotter

# Edit to declare if you are testing the sim or the real robot
is_real = False

robot = Robot(is_real)

print("Setup Complete")

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


simStartTime = time.time()

while time.time() - simStartTime < 5:
    time.sleep(0.01)
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