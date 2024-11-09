import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from casadi.tools import *
import pdb
import sys
import os
rel_do_mpc_path = os.path.join('..','..')
sys.path.append(rel_do_mpc_path)
import do_mpc

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Circle
from matplotlib import rcParams
from matplotlib.animation import FuncAnimation, FFMpegWriter, ImageMagickWriter

# Plot settings
rcParams['text.usetex'] = False
rcParams['axes.grid'] = True
rcParams['lines.linewidth'] = 2.0
rcParams['axes.labelsize'] = 'xx-large'
rcParams['xtick.labelsize'] = 'xx-large'
rcParams['ytick.labelsize'] = 'xx-large'

from mpcModel import mpc_model
from mpcSolver import solve_mpc
import time

#settings
show_animation = True
store_animation = False
store_results = False
model = mpc_model()
mpc = solve_mpc(model)
estimator = do_mpc.estimator.StateFeedback(model)

# set initial state 
# this is where it will connect to the walking file 
# need to set x0 -> initial iteration 
mpc.set_initial_guess()

# set up graphs 
L1 = 12 #cm, height of CoM
def pendulum(x):
    line_x = np.array([
        x[0],
        x[0]+l1*np.sin(x[1])
    ])
    line_y = np.array([
        0, 
        l1*np.cos(x[1])
    ])
    
    line = np.stack((line_x, line_y))
    
    return line

mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
