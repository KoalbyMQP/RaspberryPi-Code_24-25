import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils
from matplotlib.animation import FuncAnimation
from scipy.interpolate import CubicSpline

left_leg_chain = Chain.from_urdf_file(
    "backend/Testing/FullAssem11_24_23(1).urdf",
    base_elements=['LeftHip', 'LeftLegRotator']
)

# left_leg_chain = Chain.from_urdf_file(
#     "backend/Testing/FullAssem11_24_23frame.urdf",
#     base_elements=['LeftThigh', 'LeftKnee']
# )

right_leg_chain = Chain.from_urdf_file(
    "backend/Testing/FullAssem11_24_23(1).urdf",
    base_elements=['RightHip', 'RightLegRotator']
)

# right_leg_chain = Chain.from_urdf_file(
#     "backend/Testing/FullAssem11_24_23frame.urdf",
#     base_elements=['RightThigh', 'RightKnee']
# )

target_positions_left = np.array([
    [0.05, 0.0, -0.35],
    [0.05, 0.1, -0.325],
    [0.05, 0.1, -0.3],
    [0.05, 0.0, -0.3],
    [0.05, -0.1, -0.3],
    [0.05, -0.1, -0.325],
    [0.05, 0.0, -0.35]
])
target_positions_right = np.array([
    [-0.05, 0.0, -0.3],
    [-0.05, -0.1, -0.3],
    [-0.05, -0.1, -0.325],
    [-0.05, 0.0, -0.35],
    [-0.05, 0.1, -0.325],
    [-0.05, 0.1, -0.3],
    [-0.05, 0.0, -0.3]
])

n_points = 100

t_left = np.linspace(0, 1, len(target_positions_left))
splines_left = [CubicSpline(t_left, target_positions_left[:, dim]) for dim in range(3)]
trajectory_left = np.vstack([spline(np.linspace(0, 1, n_points)) for spline in splines_left]).T

t_right = np.linspace(0, 1, len(target_positions_right))
splines_right = [CubicSpline(t_right, target_positions_right[:, dim]) for dim in range(3)]
trajectory_right = np.vstack([spline(np.linspace(0, 1, n_points)) for spline in splines_right]).T

fig, ax = plot_utils.init_3d_figure()

# Setting axis limits
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# Adding labels to the axes
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

# Plot the initial splines as lines on the 3D plot
spline_line_left, = ax.plot(trajectory_left[:, 0], trajectory_left[:, 1], trajectory_left[:, 2], color='b', label="Left Leg Spline")
spline_line_right, = ax.plot(trajectory_right[:, 0], trajectory_right[:, 1], trajectory_right[:, 2], color='r', label="Right Leg Spline")

def update_plot(frame):
    target_position_left = trajectory_left[frame]
    ik_solution_left = left_leg_chain.inverse_kinematics(target_position_left)

    target_position_right = trajectory_right[frame]
    ik_solution_right = right_leg_chain.inverse_kinematics(target_position_right)

    ax.cla()  # Clear the axis for each frame

    # Plot the current IK solution
    left_leg_chain.plot(ik_solution_left, ax)
    right_leg_chain.plot(ik_solution_right, ax)

    # Redefine axis limits and labels after clearing the plot
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    # Re-plot the full splines
    ax.plot(trajectory_left[:, 0], trajectory_left[:, 1], trajectory_left[:, 2], color='b', label="Left Leg Spline")
    ax.plot(trajectory_right[:, 0], trajectory_right[:, 1], trajectory_right[:, 2], color='r', label="Right Leg Spline")

    return ax

# Create the animation
ani = FuncAnimation(fig, update_plot, frames=n_points, interval=50, blit=False)

# Display the plot
plt.show()
