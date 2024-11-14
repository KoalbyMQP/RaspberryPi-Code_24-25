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
    mass = 3.5

    #setting up the model

    # #creating state space model 
    # a = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]])
    # b = np.array([[0], [0], [1]])
    # c = np.array([[1, 0, (-z_c/g)], [1, 0, 0], [0, 1, 0]])
    # d = np.array([[0], [0], [0]])

    # # these are the continuous state space models 
    # lip_x = signal.StateSpace(a, b, c, d)
    # lip_y = signal.StateSpace(a, b, c, d)
    
    # # #making matrixes into casadi SX type
    # # A_sx = ca.SX(a)
    # # B_sx = ca.SX(b)
    # # C_sx = ca.SX(c)
    # # D_sx = ca.SX(d)

    # #setting up states
    # x = model.set_variable(var_type='_x', var_name='x', shape=(1,1))
    # y = model.set_variable(var_type='_x', var_name='y', shape=(1,1))
    # xd = model.set_variable(var_type='_x', var_name='xd', shape =(1,1))
    # yd = model.set_variable(var_type='_x', var_name='yd', shape=(1, 1))
    xdd = model.set_variable(var_type='_x', var_name='xdd', shape =(1,1))
    ydd = model.set_variable(var_type='_x', var_name='ydd', shape =(1,1))

    
    #inputs 
    ur = model.set_variable(var_type='_u', var_name='xddd', shape=(1,1))
    up = model.set_variable(var_type= '_u', var_name='yddd', shape=(1,1))
    
    model.set_rhs('xdd', (g/z_c)*x - (1/mass)*ur)
    model.set_rhs('ydd', (g/z_c)*y - (1/mass)*up)
    
    # """
    # the following comment out code is to try something new, and I dont think it works. 
    # the goal was that it would make the state space system discrete for the next iteration
    # """
    # A = np.array([[1, ts, ts**2/2], [0, 1, ts], [0, 0, 1]])
    # B = np.array([[ts**3/6], [ts**2/2], [ts]])
    # #B = MX(ts**3/6, ts**2/2, ts)
    # C = np.array([1, 0, -z_c/g])
    # b2 = np.transpose(B)
    # # print(B)
    # # print("lip_x", lip_x)

    
    # #making a discrete system 
    # nextStepx = ca.mtimes(A, x) + ca.mtimes(B.T, xdd)
    # nextStepy = ca.mtimes(A, y) + ca.mtimes(B.T, ydd)
    
    # model.set_rhs('x', lip_x)
    # model.set_rhs('y', lip_y)
    # model.set_rhs('x_dot', nextStepx)
    # model.set_rhs('y_dot', nextStepy)
    
    
    
    #need to track position 
    model.setup()

    return model


