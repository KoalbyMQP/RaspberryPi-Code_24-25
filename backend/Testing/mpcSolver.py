import do_mpc
import numpy as np

def solve_mpc(model, silence_solver=False):
    mpc = do_mpc.controller.MPC(model)

    # MPC settings remain the same
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

    # State and input constraints as before
    state_bounds = {'x': 20.0, 'y': 20.0, 'xd': 20.0, 'yd': 20.0}
    for state_name, bound in state_bounds.items():
        mpc.bounds['lower', '_x', state_name] = -bound
        mpc.bounds['upper', '_x', state_name] = bound

    mpc.bounds['lower', '_u', 'xddd'] = -20
    mpc.bounds['upper', '_u', 'xddd'] = 20
    mpc.bounds['lower', '_u', 'yddd'] = -20
    mpc.bounds['upper', '_u', 'yddd'] = 20

    # Create a template for TVPs
    tvp_template = mpc.get_tvp_template()
    print(tvp_template)
    
    def tvp_fun(t_now):

        if isinstance(t_now, np.ndarray):
            # If t_now is something like array([0.]), convert it to a float
            t_now = t_now.item()  


        # Here we define how px, py change over time
        # For example, move to (5,5) for first 25 steps, then to (10,5), etc.
        step = int(round(t_now/mpc.settings.t_step))

        # Set default
        px_ref = 5
        py_ref = 5

        # After 50 steps (which is 5.0 s if dt=0.1)
        if step > 10:
            px_ref = 0
            py_ref = 10

        # After 75 steps (7.5 s)
        if step > 20:
            px_ref = 5
            py_ref = 15

        if step > 30:
            px_ref = 0
            py_ref = 20

        for k in range(mpc.settings.n_horizon+1):
            tvp_template['_tvp', k, 'px'] = px_ref
            tvp_template['_tvp', k, 'py'] = py_ref
        return tvp_template

    # Set the TVP function
    mpc.set_tvp_fun(tvp_fun)

    mpc.setup()
    return mpc
