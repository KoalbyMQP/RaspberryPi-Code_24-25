"""
This file is to log the via points of the trajectories for the assisted walking demo in effort to clean up the main code.
Angles are in radians and can be calculated using the MATLAB scripts found in backend>KoalbyHumanoid>MATLAB Scripts.

Format of each list goes as follows:
    [0]: list of t_f for each via point
    [1]: list of joint angles (rad)
    [2]: list of joint velocities through each via_point (v_f)
    [3]: list of joint accelerations through each via point (a_f)
"""

import math

leftAbd = -0
rightAbd = 0

lAnkleOffset = 0
rAnkleOffset = -0

## LEFT FOOT (lf) #############################################################

# lf_Even2Right = [
#     [[0,0,0,0,0], [3,3,3,3,3]],
#     [[math.radians(leftAbd), 0.000000, -0.349125, -0.349173, 0.349113 + math.radians(lAnkleOffset)], # 0 0 0
#                [math.radians(leftAbd), -0.000000, -0.337710, -0.415672, 0.427027 + math.radians(lAnkleOffset)]], # 0 0 -20
#     [[0,0,0,0,0],[0,0,0,0,0]],
#     [[0,0,0,0,0],[0,0,0,0,0]]
# ]

# lf_Right2Left = [
#     [[0,0,0,0,0], [1,1,1,1,1], [2,2,2,2,2], [3,3,3,3,3]],
#     [[math.radians(leftAbd), -0.000000, -0.267421, -0.466078, 0.547723 + math.radians(lAnkleOffset)], # 0 0 -60
#                [math.radians(leftAbd), -0.000000, -0.718043, -1.357030, 0.988053 + math.radians(lAnkleOffset)], # 0 80 -80
#                [math.radians(leftAbd), -0.000000, -0.936060, -1.312802, 0.725808 + math.radians(lAnkleOffset)], # 0 80 0
#                [math.radians(leftAbd), 0.000000, -0.349125, -0.349173, 0.349113 + math.radians(lAnkleOffset)]], # 0 0 0
#     [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]],
#     [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
# ]

# lf_Left2Right = [
#     [[0,0,0,0,0], [3,3,3,3,3]],
#     [[math.radians(leftAbd), 0.000000, -0.349125, -0.349173, 0.349113 + math.radians(lAnkleOffset)], # 0 0 0
#                [math.radians(leftAbd), -0.000000,  -0.309161, -0.452410, 0.492314 + math.radians(lAnkleOffset)]], # 0 0 -40
#     [[0,0,0,0,0],[0,0,0,0,0]],
#     [[0,0,0,0,0],[0,0,0,0,0]]
# ]
## NEW LEFT FOOT
lf_Even2Right = [
    [[0,0,0,0,0], [3,3,3,3,3]],
    [[math.radians(leftAbd), -0.000000, -0.349066, -0.349066, 0.349066 + math.radians(lAnkleOffset)], # 0 0 0
               [math.radians(leftAbd), -0.000000, -0.337710, -0.415672, 0.427027 + math.radians(lAnkleOffset)]], # 0 0 -20
    [[0,0,0,0,0],[0,0,0,0,0]],
    [[0,0,0,0,0],[0,0,0,0,0]]
]

lf_Left2Right = [
    [[0,0,0,0,0], [3,3,3,3,3]],
    [[math.radians(leftAbd), -0.000000, -0.349066, -0.349066, 0.349066 + math.radians(lAnkleOffset)], # 0 0 0
               [math.radians(leftAbd), -0.000000, -0.337710, -0.415672, 0.427027 + math.radians(lAnkleOffset)]], # 0 0 -20
    [[0,0,0,0,0],[0,0,0,0,0]],
    [[0,0,0,0,0],[0,0,0,0,0]]
]
lf_Right2Left = [
    [[0,0,0,0,0], [1,1,1,1,1], [2,2,2,2,2], [3,3,3,3,3]],
    [[math.radians(leftAbd), -0.000000, -0.337710, -0.415672, 0.427027 + math.radians(lAnkleOffset)], # 0 0 -20
               [math.radians(leftAbd), -0.000000, -0.636551, -1.286623, 0.650072+ math.radians(lAnkleOffset)], # 0 80 -80
               [math.radians(leftAbd), -0.000000, -0.936060, -1.312802, 0.725808 + math.radians(lAnkleOffset)], # 0 80 0
               [math.radians(leftAbd), -0.000000, -0.349066, -0.349066, 0.349066 + math.radians(lAnkleOffset)]], # 0 0 -0
    [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]],
    [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
]
## RIGHT FOOT (rf) #############################################################

rf_Even2Right = [
    [[0,0,0,0,0], [1,1,1,1,1], [2,2,2,2,2], [3,3,3,3,3]],
    [[math.radians(rightAbd), -0.000000, 0.349066, 0.349066, -0.349066 + math.radians(rAnkleOffset)], # 0 0 0
                [math.radians(rightAbd), 0.000000, 0.755823, 1.094676, -0.338853 + math.radians(rAnkleOffset)],  # 0 60 0
                [math.radians(rightAbd), -0.000000, 0.755823, 1.094676, -0.338853 + math.radians(rAnkleOffset)], # 0 60 0
                [math.radians(rightAbd), -0.000000, 0.349066, 0.349066, -0.349066 + math.radians(rAnkleOffset)]], # 0 0 0
    [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]],
    [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
]

rf_Right2Left = [
    [[0,0,0,0,0], [3,3,3,3,3]],
    [[math.radians(rightAbd), -0.000000, 0.349066, 0.349066, -0.349066 + math.radians(rAnkleOffset)], # 0 0 0
               [math.radians(rightAbd), -0.000000, 0.337710, 0.415672, -0.427027 + math.radians(rAnkleOffset)]], # 0 0 -20
    [[0,0,0,0,0],[0,0,0,0,0]],
    [[0,0,0,0,0],[0,0,0,0,0]]
]

rf_Left2Right = [
    [[0,0,0,0,0], [1,1,1,1,1], [2,2,2,2,2], [3,3,3,3,3]],
    [[math.radians(rightAbd), -0.000000,0.337710, 0.415672, -0.427027 + math.radians(rAnkleOffset)], # 0 0 -60
               [math.radians(rightAbd), -0.000000, 0.636551, 1.286623, -0.650072+ math.radians(rAnkleOffset)], # 0 80 -80
               [math.radians(rightAbd), -0.000000, 0.936060, 1.312802, -0.725808 + math.radians(rAnkleOffset)], # 0 80 0
               [math.radians(rightAbd), -0.000000, 0.349066, 0.349066, -0.349066 + math.radians(rAnkleOffset)]], # 00 -0
    [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]],
    [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
]

## Right Arm (ra) #############################################################

ra_grabCart = [
    [[0,0,0,0,0], [1,1,1,1,1], [2,2,2,2,2], [3,3,3,3,3]], 
    [[-0.349065, 1.570797, -0.000001, 1.919868, 0.000007], 
        [-0.120985, 1.570796, 0.000000, 2.001527, 0.309746],
        [0.330818, 1.570795, 0.000002, 1.583480, 0.343501],
        [0.311330, 1.570796, 0.000001, 1.147939, -0.111528]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]
]

ra_rightDown = [
    [[0,0,0,0,0], [1,1,1,1,1], [2,2,2,2,2], [3,3,3,3,3]],
    [[0.000000, -1.570796, 0.000000, 1.570796, 0.000000], # 0 0 0
        [-0.059590, -1.570796, 0.000000, 1.145847, -0.365360], # 0 -40 0
        [-0.059590, -1.570796, 0.000000, 1.145847, -0.365360], # 0 -40 0
        [0.000000, -1.570796, 0.000000, 1.570796, 0.000000]], # 0 0 0
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]
]

ra_leftDown = [
    [[0,0,0,0,0], [1,1,1,1,1], [2,2,2,2,2], [3,3,3,3,3]],
    [[0.000000, -1.570796, 0.000000, 1.570796, 0.000000],
        [0.000000, -1.570796, 0.000000, 1.570796, 0.000000],
        [0.000000, -1.570796, 0.000000, 1.570796, 0.000000],
        [0.000000, -1.570796, 0.000000, 1.570796, 0.000000]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]
]

## Left Arm (la) ##############################################################

la_grabCart = [
    [[0,0,0,0,0], [1,1,1,1,1], [2,2,2,2,2], [3,3,3,3,3]],
    [[0.349066, -1.570796, 0.000000, -1.919862, 0.000000],
        [0.051372, -1.570796, -0.000000, -1.956822, -0.334654],
        [-0.330817, -1.570796, -0.000000, -1.583478, -0.343500],
        [-0.311330, -1.570796, -0.000000, -1.147936, 0.111531]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]
]

la_rightDown = [
    [[0,0,0,0,0], [1,1,1,1,1], [2,2,2,2,2], [3,3,3,3,3]],
    [[0.000000, 1.570796, 0.000000, -1.570796, 0.000000],
        [0.000000, 1.570796, 0.000000, -1.570796, 0.000000],
        [0.000000, 1.570796, 0.000000, -1.570796, 0.000000],
        [0.000000, 1.570796, 0.000000, -1.570796, 0.000000]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]
]

la_leftDown = [
    [[0,0,0,0,0], [1,1,1,1,1], [2,2,2,2,2], [3,3,3,3,3]],
    [[0.000000, 1.570796, 0.000000, -1.570796, 0.000000],
        [0.059590, 1.570796, 0.000000, -1.145847, 0.365360],
        [0.059590, 1.570796, 0.000000, -1.145847, 0.365360],
        [0.000000, 1.570796, 0.000000, -1.570796, 0.000000]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]],
    [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]
]