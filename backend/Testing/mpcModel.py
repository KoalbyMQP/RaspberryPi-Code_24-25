import numpy as np
import casadi as ca
import do_mpc

def mpc_model(symvar_type='SX'):
    model_type = 'continuous'
    model = do_mpc.model.Model(model_type)

    # System constants
    g = 9.8  # gravity
    z_c = 15  # Height of CoM of the Robot
    mass = 3.5
    damping = 0.1 

    # States
    x = model.set_variable(var_type='_x', var_name='x')
    y = model.set_variable(var_type='_x', var_name='y')
    xd = model.set_variable(var_type='_x', var_name='xd')
    yd = model.set_variable(var_type='_x', var_name='yd')

    # Inputs
    ur = model.set_variable(var_type='_u', var_name='xddd')
    up = model.set_variable(var_type='_u', var_name='yddd')

    # Dynamics
    xdd = (g / z_c) * x - (5 / mass) * ur - damping * xd
    ydd = (g / z_c) * y - (5 / mass) * up - damping * yd


    # set_alg for xdd and ydd
    model.set_rhs('x', xd)
    model.set_rhs('xd', xdd)
    model.set_rhs('y', yd)
    model.set_rhs('yd', ydd)

    # Cost function
    px, py = 5, 5  # Desired positions
    cost_x = 1 * (x - px) ** 2
    cost_y = 1 * (y - py) ** 2
    cost_x_vel = 1 * xd ** 2
    cost_y_vel = 1 * yd ** 2
    total_cost = cost_x + cost_y #+ cost_x_vel + cost_y_vel
    model.set_expression(expr_name='cost', expr=total_cost)

    # Final setup
    model.setup()
    return model
