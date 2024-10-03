# imported stuff from the example 
import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
import os
rel_do_mpc_path = os.path.join('..','..')
sys.path.append(rel_do_mpc_path)
import do_mpc

def ip_model(obstacles, symvar_type='SX'): #takes in obstacles(in this case will be bounds of foot), and type of model
    """
    --------------------------------------------------------------------------
    template_model: Variables / RHS / AUX
    --------------------------------------------------------------------------
    """
    model_type = 'continuous' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type, symvar_type)
    
    #set parameters 
    m0 = 3.255 #mass of whole robot in kg
    L0= .4572 #arbitrary m (18 in), distance from COM to ground, In the future I see this as 
    l0 =L0/2
    j0= (m0*l0**2) /3 #inertia
    
    m0 = model.set_variable('_p', 'm0')
    g = 9.80665 # m/s^2, gravity
    
    # Setpoint x:
    pos_set = model.set_variable('_tvp', 'pos_set')


    # States struct (optimization variables):
    pos = model.set_variable('_x',  'pos')
    theta = model.set_variable('_x',  'theta', (2,1))
    dpos = model.set_variable('_x',  'dpos')
    dtheta = model.set_variable('_x',  'dtheta', (2,1))
    # Algebraic states:
    ddpos = model.set_variable('_z', 'ddpos')
    ddtheta = model.set_variable('_z', 'ddtheta', (2,1))

    # Input struct (optimization variables):
    u = model.set_variable('_u',  'force')

    # Differential equations
    model.set_rhs('pos', dpos)
    model.set_rhs('theta', dtheta)
    model.set_rhs('dpos', ddpos)
    model.set_rhs('dtheta', ddtheta)
    #Im not sure if I need the position vector right now as I am just trying to control the swing of the IP
    #next steps is have the dynamic equations and then the cost function 
    
    
    
    
    # Build the model
    model.setup()

    return model
    