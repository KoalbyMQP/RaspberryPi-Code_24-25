import numpy as np
import casadi as ca
import do_mpc

def mpc_model(symvar_type='SX'):
    model_type = 'continuous'
    model = do_mpc.model.Model(model_type)

    # System constants
    g = 9.8  
    z_c = 15  
    mass = 3.5

    # States
    x = model.set_variable(var_type='_x', var_name='x')
    y = model.set_variable(var_type='_x', var_name='y')
    xd = model.set_variable(var_type='_x', var_name='xd')
    yd = model.set_variable(var_type='_x', var_name='yd')

    ur = model.set_variable(var_type='_u', var_name='xddd')
    up = model.set_variable(var_type='_u', var_name='yddd')

    px = model.set_variable(var_type='_tvp', var_name='px')
    py = model.set_variable(var_type='_tvp', var_name='py')

    xdd = (g / z_c) * x - (5 / mass) * ur
    ydd = (g / z_c) * y - (5 / mass) * up

    model.set_rhs('x', xd)
    model.set_rhs('xd', xdd)
    model.set_rhs('y', yd)
    model.set_rhs('yd', ydd)

    cost_x = 1 * (x - px) ** 2
    cost_y = 1 * (y - py) ** 2
    total_cost = cost_x + cost_y
    model.set_expression(expr_name='cost', expr=total_cost)

    model.setup()
    return model
