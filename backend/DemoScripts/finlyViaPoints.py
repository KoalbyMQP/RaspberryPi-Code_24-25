"""
FOR DESIGN TEAM

This file is to log the via points of the trajectories for the demo in effort to clean up the main code.
Angles are in radians and can be calculated using the MATLAB scripts found in backend>KoalbyHumanoid>MATLAB Scripts.

Format of each list goes as follows:
    [0]: list of t_f for each via point
    [1]: list of joint angles (rad)
    [2]: list of joint velocities through each via_point (v_f)
    [3]: list of joint accelerations through each via point (a_f)
"""

leftArmTraj = [
    [[0,0,0,0,0], [4,4,4,4,4], [8,8,8,8,8]],
    [[0.349066, -1.570796, 0.000000, -1.919862, 0.000000],
     [-0.634948, -1.570796, -0.000000, -0.952591, -0.016743],
     [-0.555547, -0.613355, -0.467164, -1.344703, -1.030752]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]
]