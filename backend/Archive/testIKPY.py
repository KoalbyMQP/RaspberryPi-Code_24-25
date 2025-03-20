import numpy as np
import math
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils

left_leg_chain = Chain.from_urdf_file(
    "backend/Testing/FinleyJNEWARMS_2024_2.urdf",
    base_elements=['shoulder1_left', 'shoulder1_left'],
    active_links_mask=[False, True, True, True, True, True, True]
    
)

target_positions = np.array([.39076,  -.08197, .76541])

ik_solution = left_leg_chain.inverse_kinematics(target_positions)
ik_solution_2=np.array([0,0,0,0,0,0,0])
print("Joint angles:", ik_solution)

real_frame = left_leg_chain.forward_kinematics(ik_solution_2)
#print("Joint angles:", real_frame)
fig, ax = plot_utils.init_3d_figure()

left_leg_chain.plot(ik_solution, ax)

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

plt.show()