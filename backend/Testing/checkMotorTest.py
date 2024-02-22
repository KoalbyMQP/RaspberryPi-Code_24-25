
import sys
sys.path.append("./")
import math
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.Config import Joints

robot = Robot(True)

robot.checkMotors()
