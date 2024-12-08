

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


from mpcModel import mpc_model
from mpcSolver import solve_mpc
import time

# RK4 Integration Function
def rk4_step(f, x, u, dt):
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

# Dynamics for integration
def dynamics(x, u):
    # Ensure u is a flat 1D array (extracting values if u is a 2D array)
    u = u.flatten() if len(u.shape) > 1 else u
    return np.array([
        x[2],  # xd
        x[3],  # yd
        (9.8 / 15) * x[0] - (5 / 3.5) * u[0],  # xdd
        (9.8 / 15) * x[1] - (5 / 3.5) * u[1]   # ydd
    ])

# Initialize Model and MPC
model = mpc_model()
mpc = solve_mpc(model)
mpc.set_initial_guess()
x0 = np.array([2, 2, 0, 0])  # Start closer to the desired position
trajectory = [x0]

# Simulation Loop
for i in range(200):
    u0 = mpc.make_step(x0)
    x0 = rk4_step(dynamics, x0, u0, mpc.settings.t_step)
    trajectory.append(x0)

trajectory = np.array(trajectory)

# Pendulum Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
origin = 0

def pendulum(x, origin):
    line_x = np.array([origin, x[0]])
    line_y = np.array([origin, x[1]])
    line_z = np.array([0, 15])  # z_c remains constant
    return line_x, line_y, line_z

for state in trajectory:
    if(state[1] >= 4.9):
        origin = 5
    x, y, z = pendulum(state, origin)
    ax.plot(x, y, z, color='blue')
    ax.scatter(x[1], y[1], z[1], color='red')

# Configure the plot
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(0, 20)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Pendulum Motion with LIPM Dynamics")

plt.show()
