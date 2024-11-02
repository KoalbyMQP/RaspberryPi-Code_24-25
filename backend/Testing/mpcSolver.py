import do_mpc
import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
import os
rel_do_mpc_path = os.path.join('..','..')
sys.path.append(rel_do_mpc_path)

def template_mpc(model, silence_solver = False):
    """
    --------------------------------------------------------------------------
    template_mpc: tuning parameters
    --------------------------------------------------------------------------
    """
    mpc = do_mpc.controller.MPC(model)

    mpc.settings.n_robust = 0 #optimization problem grow exponentially with this setting
    mpc.settings.n_horizon = 2 #number of future estimations
    mpc.settings.t_step = 0.5 #how long it takes to make a step 
    mpc.settings.store_full_solution =True  #stores solution 

    if silence_solver:
        mpc.settings.supress_ipopt_output()


    mterm = model.aux['cost'] #terminal cost 
    lterm = model.aux['cost'] #stage cost
    
    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(u=1e-4)

    max_x = np.array([[4.0], [10.0], [4.0], [10.0]])

    mpc.bounds['lower','_x','x'] = -max_x
    mpc.bounds['upper','_x','x'] =  max_x

    mpc.bounds['lower','_u','u'] = -0.5
    mpc.bounds['upper','_u','u'] =  0.5


    mpc.setup()

    return mpc
