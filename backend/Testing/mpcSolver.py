import do_mpc
import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
import os
rel_do_mpc_path = os.path.join('..','..')
sys.path.append(rel_do_mpc_path)

def solve_mpc(model, silence_solver = False):
    """
    --------------------------------------------------------------------------
    template_mpc: tuning parameters
    I honestly havent touched this. Not sure how much needs to be added or adjusted 
    --------------------------------------------------------------------------
    """
    mpc = do_mpc.controller.MPC(model)

    mpc.settings.n_robust = 0 #optimization problem grow exponentially with this setting
    mpc.settings.n_horizon = 2 #number of future estimations
    mpc.settings.t_step = 5 #how long it takes to make a step in seconds
    mpc.settings.store_full_solution =True  #stores solution 

    if silence_solver:
        mpc.settings.supress_ipopt_output()


    mterm = model.aux['cost'] #terminal cost 
    lterm = model.aux['cost'] #stage cost
    
    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(xddd=1e-4, yddd=1e-4)

    # max_x = np.array([[4.0], [10.0], [4.0], [10.0]]) #needs to be replaced with size of feet 

    # # Set lower and upper bounds for each state variable
    # for i, bound in enumerate(max_x.flatten()):
    #     mpc.bounds['lower', '_x', i] = -bound
    #     mpc.bounds['upper', '_x', i] = bound
    # Define bounds for each state variable explicitly based on their names
    state_bounds = {'x': 4.0, 'y': 10.0, 'xd': 4.0, 'yd': 10.0}

    for state_name, bound in state_bounds.items():
        mpc.bounds['lower', '_x', state_name] = -bound
        mpc.bounds['upper', '_x', state_name] = bound


    mpc.bounds['lower', '_u', 'xddd'] = -0.5
    mpc.bounds['upper', '_u', 'xddd'] = 0.5

    mpc.bounds['lower', '_u', 'yddd'] = -0.5
    mpc.bounds['upper', '_u', 'yddd'] = 0.5



    mpc.setup()

    return mpc
