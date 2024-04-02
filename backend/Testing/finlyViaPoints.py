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

##first movement to grab the candy
leftArmTraj1 = [
    [[0,0,0,0,0], [3,3,3,3,3], [6,6,6,6,6],[9,9,9,9,9]],
    [[0.349066, -1.570796, 0.000000, -1.919862, 0.000000],
     [0.467484, -0.709233, 0.380303, -1.543701, -0.919356],
     [0.485506, -0.694350, 0.398759, -1.558736, -0.938553],
     [0.503996, -0.679529, 0.418190, -1.573259, -0.957807]],
     [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]],
     [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]]

leftArmTraj2 = [
    [[0,0,0,0,0],[3,3,3,3,3],[6,6,6,6,6],[9,9,9,9,9]]
     [[0.414209, -0.645689, 0.343485, -1.524787, -0.969501],
     [0.287245, -1.447992, 0.035687, -1.906203, -0.127452],
     [0.349066, -1.570796, 0.000000, -1.919862, 0.000000]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0],],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0],]]
##second movement to drop the candy
