import sys, time, math 
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid import trajPlannerPose

is_real = False

robot = Robot(is_real)

print("Setup Complete")

setPoints = [[0, 0, 0], [math.radians(50), math.radians(-100), math.radians(-50)], [0, 0, 0]]
tj = trajPlannerPose.TrajPlannerPose(setPoints)
traj = tj.getCubicTraj(5, 100)

robot.motors[1].target = (math.radians(80), 'P')
robot.motors[6].target = (math.radians(-80), 'P')

simStartTime = time.time()
while time.time() - simStartTime < 5:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()

while True:
    startTime = time.time()
    for point in traj:
        time.sleep(0.01)
        robot.IMUBalance(0,0)
        robot.motors[17].target = (point[1], 'P')
        robot.motors[18].target = (point[2], 'P')
        robot.motors[19].target = (point[3], 'P')

        robot.motors[22].target = (-point[1], 'P')
        robot.motors[23].target = (-point[2], 'P')
        robot.motors[24].target = (-point[3], 'P')

        robot.moveAllToTarget()
        while time.time() - startTime < point[0]:
            time.sleep(0.01)
            robot.IMUBalance(0,0)
            robot.moveAllToTarget()