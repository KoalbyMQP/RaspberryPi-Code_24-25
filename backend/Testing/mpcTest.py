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
# import matplotlib.gridspec as gridspec
# from matplotlib.patches import Circle
# from matplotlib import rcParams
# from matplotlib.animation import FuncAnimation, FFMpegWriter, ImageMagickWriter

# Plot settings
# rcParams['text.usetex'] = False
# rcParams['axes.grid'] = True
# rcParams['lines.linewidth'] = 2.0
# rcParams['axes.labelsize'] = 'xx-large'
# rcParams['xtick.labelsize'] = 'xx-large'
# rcParams['ytick.labelsize'] = 'xx-large'

# from mpcModel import mpc_model
# from mpcSolver import solve_mpc
# import time

# #settings
# show_animation = True
# store_animation = False
# store_results = False
# model = mpc_model()
# mpc = solve_mpc(model)
# estimator = do_mpc.estimator.StateFeedback(model)

# # set initial state 
# # this is where it will connect to the walking file 
# # need to set x0 -> initial iteration 
# mpc.set_initial_guess()

# # set up graphs 
# L1 = 12 #cm, height of CoM
# def pendulum(x):
#     line_x = np.array([
#         x[0],
#         x[0]+l1*np.sin(x[1])
#     ])
#     line_y = np.array([
#         0, 
#         l1*np.cos(x[1])
#     ])
    
#     line = np.stack((line_x, line_y))
    
#     return line

# mpc_graphics = do_mpc.graphics.Graphics(mpc.data)


"""
Creating 3d plot of the motion
"""
# creating 2d plot in 3d 
ax = plt.figure().add_subplot(projection='3d')

#defining IP
x = np.linspace(0, 1, 100)
y = 10*x
ax.plot(x, y, zs=10, zdir='y', label='curve in (x, y)')

#creating ball at the top of the line
top_x = x[-1]
top_y = y[-1]
ax.scatter(top_x, top_y, zs=10, zdir='y', color='blue', label='Top Point')


# Make legend, set axes limits and labels
ax.legend()
ax.set_xlim(0, 20)
ax.set_ylim(0, 20)
ax.set_zlim(0, 20)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()