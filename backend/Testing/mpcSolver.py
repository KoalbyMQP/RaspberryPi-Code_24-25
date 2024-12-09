import do_mpc

def solve_mpc(model, silence_solver=False):
    mpc = do_mpc.controller.MPC(model)

    mpc.settings.n_robust = 0
    mpc.settings.n_horizon = 10
    mpc.settings.t_step = 0.1
    mpc.settings.store_full_solution = True

    if silence_solver:
        mpc.settings.supress_ipopt_output()

    mterm = model.aux['cost']
    lterm = model.aux['cost']
    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(xddd=1e-2, yddd=1e-2)

    state_bounds = {'x': 5.0, 'y': 5.0, 'xd': 5.0, 'yd': 5.0}
    for state_name, bound in state_bounds.items():
        mpc.bounds['lower', '_x', state_name] = -bound
        mpc.bounds['upper', '_x', state_name] = bound

    mpc.bounds['lower', '_u', 'xddd'] = -20
    mpc.bounds['upper', '_u', 'xddd'] = 20
    mpc.bounds['lower', '_u', 'yddd'] = -20
    mpc.bounds['upper', '_u', 'yddd'] = 20

    mpc.setup()
    return mpc
