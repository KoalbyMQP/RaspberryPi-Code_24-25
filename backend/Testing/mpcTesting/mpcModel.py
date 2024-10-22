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
    #Im not sure if I need the position vector right now as I am just trying to control the swing of the IP
    #next steps is have the dynamic equations and then the cost function 
    euler_lagrange = vertcat(
        # #1
        # (m0 + m_com)*ddpos + m_com*(ddtheta*cos(theta)-(ddtheta**2)* sin(theta)),
        # #2
        # m_com*g*l0*sin(theta)+m_com*l0*ddpos*cos(theta)+m_com*l0**2*ddtheta
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
    
     # Coordinates of the nodes:
    node0_x = model.x['pos']
    node0_y = np.array([0])

    node1_x = node0_x+l0*sin(model.x['theta',0])
    node1_y = node0_y+l0*cos(model.x['theta',0])


    obstacle_distance = []

    for obs in obstacles:
        d0 = sqrt((node0_x-obs['x'])**2+(node0_y-obs['y'])**2)-obs['r']*1.05
        d1 = sqrt((node1_x-obs['x'])**2+(node1_y-obs['y'])**2)-obs['r']*1.05
        obstacle_distance.extend([d0, d1])


    model.set_expression('obstacle_distance',vertcat(*obstacle_distance))
    model.set_expression('tvp', pos_set)
    
    # Build the model
    model.setup()
    
    # print(model._getvar('_x'))

    return model
    
