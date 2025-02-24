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

# Edit to declare if you are testing the sim or the real robot
is_real = True
robot = Robot(is_real)
print("Setup Complete")

robot.motors[27].target = (math.radians(0), 'P')
robot.moveAllToTarget()
