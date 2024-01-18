import sys, time, math 
sys.path.append("./")
from backend.Testing import initSim, initRobot
from backend.KoalbyHumanoid import trajPlanner
from backend.KoalbyHumanoid.Robot import Joints

tj = trajPlanner.TrajPlannerNew([[4, 28, -10, 0], [0, 0, 0, 0], [10, 10, 10, 10]])
fulltraj = tj.getNTraj(13.5, 10, 5)
print("fulltraj")
print(fulltraj)