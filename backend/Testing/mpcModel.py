import numpy as np
import scipy as scipy
import casadi as ca
from scipy import signal

#from backend.KoalbyHumanoid.Robot import Robot

# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
import os
rel_do_mpc_path = os.path.join('..','..')
sys.path.append(rel_do_mpc_path)

# Import do_mpc package:
import do_mpc

def mpc_model(symvar_type='SX'):
    # robot = Robot
    model_type = 'continuous' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    #system constants 
    g = 9.8 #gravity 
    z_c = 15 #robot.updateRobotCoMCoM[3] #Height of COM of the Robot
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

    # these are the continuous state space models 
    lip_x = signal.StateSpace(a, b, c, d)
    lip_y = signal.StateSpace(a, b, c, d)
    
    #making matrixes into casadi SX type
    A_sx = ca.SX(a)
    B_sx = ca.SX(b)
    C_sx = ca.SX(c)
    D_sx = ca.SX(d)

    #setting up states
    x = model.set_variable(var_type='_x', var_name='x', shape=(3,1))
    y = model.set_variable(var_type='_x', var_name='y', shape=(3,1))
    x_dot = model.set_variable(var_type='_x', var_name='x_dot', shape =(1,1))
    y_dot = model.set_variable(var_type='_x', var_name='y_dot', shape=(1, 1))
    
    #inputs 
    xdd = model.set_variable(var_type='_u', var_name='xddd', shape=(3,1))
    ydd = model.set_variable(var_type= '_u', var_name='yddd', shape=(3,1))
    
    """
    the following comment out code is to try something new, and I dont think it works. 
    the goal was that it would make the state space system discrete for the next iteration
    """
    # A = np.array([[1, ts, ts**2/2], [0, 1, ts], [0, 0, 1]])
    # B = np.array([[ts**3/6], [ts**2/2], [ts]])
    # #B = MX(ts**3/6, ts**2/2, ts)
    # C = np.array([1, 0, -z_c/g])
    # b2 = np.transpose(B)
    # # print(B)
    # print("lip_x", lip_x)
    
    #making a discrete system 
    nextStepx = ca.mtimes(A_sx, x) + ca.mtimes(B_sx.T, xdd)
    nextStepy = ca.mtimes(A_sx, y) + ca.mtimes(B_sx.T, ydd)
    
    model.set_rhs('x_dot', nextStepx)
    model.set_rhs('y_dot', nextStepy)
    
    #need to track position 
    model.setup()

    return model


