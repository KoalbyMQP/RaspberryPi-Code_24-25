import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from casadi.tools import *
import pdb
import sys
import os
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

rel_do_mpc_path = os.path.join('..','..')
sys.path.append(rel_do_mpc_path)
import do_mpc

from mpcModel import mpc_model
from mpcSolver import solve_mpc

def rk4_step(f, x, u, dt):
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

def dynamics(x, u):
    u = u.flatten() if len(u.shape) > 1 else u
    return np.array([
        x[2],  # xd
        x[3],  # yd
        (9.8 / 15) * x[0] - (5 / 3.5) * u[0], 
        (9.8 / 15) * x[1] - (5 / 3.5) * u[1] 
    ])

model = mpc_model()
mpc = solve_mpc(model)
mpc.set_initial_guess()

# Initial state and simulate
x0 = np.array([0, 0, 0, 0]) 
trajectory = [x0]

for i in range(40):
    u0 = mpc.make_step(x0)
    x0 = rk4_step(dynamics, x0, u0, mpc.settings.t_step)
    trajectory.append(x0)

trajectory = np.array(trajectory)

def pendulum(x, origin):
    if x[1]>15:
        line_x = np.array([5, x[0]])
    elif x[1]>10:
        line_x = np.array([0, x[0]])
    elif x[1]>5:
        line_x = np.array([5, x[0]])
    else:
        line_x = np.array([0, x[0]])

    line_y = np.array([origin, x[1]])
    line_z = np.array([0, 15])  # z_c remains constant
    return line_x, line_y, line_z

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(-10, 10)
ax.set_ylim(0, 20)
ax.set_zlim(0, 20)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Pendulum Motion with LIPM Dynamics")

line, = ax.plot([], [], [], color='blue')
point = ax.scatter([], [], [], color='red')

origin = 0
step = 5

def init():
    line.set_data([], [])
    line.set_3d_properties([])
    point._offsets3d = ([], [], [])
    return line, point

def update(frame):
    global origin, step
    state = trajectory[frame]
    
    if state[1] >= step:
        origin = step
        step += 5

    if (state[1] == 0):
        origin = 0
        step = 0

    x, y, z = pendulum(state, origin)

    line.set_data(x, y)
    line.set_3d_properties(z)

    point._offsets3d = (np.array([x[1]]), np.array([y[1]]), np.array([z[1]]))
    return line, point

anim = FuncAnimation(fig, update, frames=len(trajectory), init_func=init, blit=False, interval=200)

plt.show()