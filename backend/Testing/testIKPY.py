import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils import plot

left_leg_chain = Chain.from_urdf_file(
    "backend/Testing/FullAssem11_24_23frame.urdf", 
    base_elements=['LeftHip', 'LeftLegRotator']
)

start_positions = np.array([0.5, -0.5, -0.5])
target_positions = np.array([0.5, -0.5, -0.5])

ik_solution = left_leg_chain.inverse_kinematics(target_positions)
print("Joint Angles Solution:")
for i, angle in enumerate(ik_solution):
    print(f"Joint {i}: {angle}")
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

plot.plot_chain(left_leg_chain, ik_solution, ax=ax)

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

plt.show()
