#this is my restart of creating the model. Not using the do-mpc template because of the errors gotten from the obstacle parameter
import numpy as np
import sys
from casadi import *
# Add do_mpc to path. This is not necessary if it was installed via pip
# import os
# rel_do_mpc_path = os.path.join('..','..','..')
# sys.path.append(rel_do_mpc_path)
# Import do_mpc package:
import do_mpc

#this may need to be in a function, not sure yet
model_type = 'continuous' # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)

#define the parameters of the system 
m0 = 3.255 #mass of whole robot in kg
m_com = 2.101 #mass of chest, arms, head in kg
m_leg = 1.149 #mass of legs kg
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

euler_lagrange = vertcat(
    # #1
    # (m0 + m_com)*ddpos + m_com*(ddtheta*cos(theta)-(ddtheta**2)* sin(theta)),
    # #2
    # m_com*g*l0*sin(theta)+m_com*l0*ddpos*cos(theta)+m_com*l0**2*ddtheta
    
    # eq for just the IP 
    m_com*(-dpos*l0*dtheta*sin(theta)+(l0**2*ddtheta)) + m_com*dpos*l0*dtheta*sin(theta)
    )
model.set_alg('euler_lagrange', euler_lagrange)
    
# Expressions for kinetic and potential energy
E_kin_robot = 1 / 2 * m0 * dpos**2
E_kin_p1 = 1 / 2 * m_com * (
    (dpos + l0 * dtheta[0] * cos(theta[0]))**2 +
    (l0 * dtheta[0] * sin(theta[0]))**2) + 1 / 2 * j0 * dtheta[0]**2

E_kin = E_kin_robot +E_kin_p1
E_pot = m0 * g * l0 * cos(theta[0]) + m_com * g * (l0 * cos(theta[0]))
    
model.set_expression('E_kin', E_kin)
model.set_expression('E_pot', E_pot)

# Build the model
model.setup()


#Setting up Controller 
mpc = do_mpc.controller.MPC(model)

setup_mpc = {
    'n_horizon': 100,
    'n_robust': 0,
    'open_loop': 0,
    't_step': 0.04,
    'state_discretization': 'collocation',
    'collocation_type': 'radau',
    'collocation_deg': 3,
    'collocation_ni': 1,
    'store_full_solution': True,
    # Use MA27 linear solver in ipopt for faster calculations:
    'nlpsol_opts': {'ipopt.linear_solver': 'mumps'}
}

mpc.set_param(**setup_mpc)
mterm = model.aux['E_kin'] - model.aux['E_pot'] # terminal cost
lterm = model.aux['E_kin'] - model.aux['E_pot'] # stage cost

mpc.set_objective(mterm=mterm, lterm=lterm)
# Input force is implicitly restricted through the objective.
mpc.set_rterm(force=0.1)

#setting constraints
mpc.bounds['lower','_u','force'] = -4
mpc.bounds['upper','_u','force'] = 4

mpc.setup()


