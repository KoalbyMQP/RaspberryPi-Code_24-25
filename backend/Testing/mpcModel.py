import numpy as np
import scipy as scipy
from scipy import signal
import backend.KoalbyHumanoid.Robot as Robot

# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
import os
rel_do_mpc_path = os.path.join('..','..')
sys.path.append(rel_do_mpc_path)

# Import do_mpc package:
import do_mpc

robot = Robot
model_type = 'continuous' # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)

#system constants 
g = 9.8 #gravity 
z_c = robot.CoM[3] #Height of COM of the Robot


#setting up the model
#creating state space model 
a = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]])
b = np.array([[0], [0], [1]])
c = np.array([[1, 0, (-z_c/g)], [1, 0, 0], [0, 1, 0]])
d = np.array([[0], [0], [0]])

lip_x = signal.StateSpace(a, b, c, d)
lip_y = signal.StateSpace(a, b, c, d)



