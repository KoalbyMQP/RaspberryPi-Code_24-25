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

#setting up states
# x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
# xd = model.set_variable(var_type='_x', var_name='xd', shape=(1,1))
# xdd = model.set_variable(var_type='_x', var_name='xdd', shape=(1,1))
lip_x = model.set_variable(var_type='_x', var_name='lip_x', shape=(3, 1))
# y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
# yd = model.set_variable(var_type='_x', var_name='yd', shape=(1,1))
# ydd = model.set_variable(var_type='_x', var_name='ydd', shape=(1,1))
lip_y = model.set_variable(var_type='_x', var_name='lip_y', shape=(3,1))

#inputs 
xddd = model.set_variable(var_type='_u', var_name='xddd', shape=(1,1))
yddd = model.set_variable(var_type= '_u', var_name='yddd', shape=(1,1))




