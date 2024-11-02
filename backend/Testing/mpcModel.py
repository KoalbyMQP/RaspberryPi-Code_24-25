import numpy as np
import scipy as scipy
from scipy import signal
from casadi import SX
from backend.KoalbyHumanoid.Robot import Robot

# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
import os
rel_do_mpc_path = os.path.join('..','..')
sys.path.append(rel_do_mpc_path)

# Import do_mpc package:
import do_mpc

def template_model(symvar_type='SX'):
    robot = Robot
    model_type = 'continuous' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    #system constants 
    g = 9.8 #gravity 
    z_c = robot.updateRobotCoMCoM[3] #Height of COM of the Robot
    ts = .5 # time step 
    x = 10 #this is the position of the xCOM
    px = 5 #I think this is the wanted position of xCOM
    y = 10 #position of yCOM
    py = 5 #wanted position of yCOM

    #setting up the model

    #creating state space model 
    a = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]])
    b = np.array([[0], [0], [1]])
    c = np.array([[1, 0, (-z_c/g)], [1, 0, 0], [0, 1, 0]])
    d = np.array([[0], [0], [0]])

    lip_x = signal.StateSpace(a, b, c, d)
    lip_y = signal.StateSpace(a, b, c, d)

    #setting up states
    x = model.set_variable(var_type='_x', var_name='x', shape=(3,1))
    xdd = model.set_variable(var_type='_x', var_name='xdd', shape=(1,1))

    y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
    ydd = model.set_variable(var_type='_x', var_name='ydd', shape=(1,1))

    #inputs 
    xddd = model.set_variable(var_type='_u', var_name='xddd', shape=(1,1))
    yddd = model.set_variable(var_type= '_u', var_name='yddd', shape=(1,1))

    A = np.array([[1, ts, ts**2/2], [0, 1, ts], [0, 0, 1]])
    B = np.array([[ts**3/6], [ts**2/2], [ts]])
    C = np.array([1, 0, -z_c/g])

    nextStepx = A*x+B*lip_x
    nextStepy = A*y+B*lip_y
    
    model.set_rhs('xddd', nextStepx)
    model.set_rhs('yddd', nextStepy)
    
    #need to track position 
    model.setup()

    return model



