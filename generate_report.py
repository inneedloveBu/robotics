import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

fig = plt.figure(figsize=(8.27, 11.69))  # A4
fig.patch.set_facecolor('white')

ax = fig.add_axes([0.08, 0.05, 0.84, 0.90])
ax.axis('off')
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)

# Title
ax.text(0.5, 1.0, 'Assignment 1: Robot Localization in Simulation',
        ha='center', va='top', fontsize=15, fontweight='bold')
ax.text(0.5, 0.965, 'Intelligent Robotics — Individual Assignment',
        ha='center', va='top', fontsize=10, style='italic', color='gray')

# 1. Problem Statement
y = 0.925
ax.text(0.0, y, '1. Problem Statement', fontsize=12, fontweight='bold', va='top')
y -= 0.028
problem = (
    "This assignment addresses the localization of a differential-drive mobile robot in a 2D "
    "simulated indoor environment. The robot follows a pre-defined trajectory consisting of a "
    "straight line, a 90-degree turn, and another straight line. The objective is to estimate "
    "the robot's pose (x, y, theta) over time using only noisy odometry measurements, and to "
    "evaluate the accuracy of the estimate against the simulator's ground-truth pose."
)
ax.text(0.0, y, problem, fontsize=9.5, va='top', linespacing=1.6)

# 2. Methods
y -= 0.105
ax.text(0.0, y, '2. Methods', fontsize=12, fontweight='bold', va='top')
y -= 0.028
methods = (
    "Simulation Platform: ROS 2 Jazzy with Gazebo Harmonic; differential-drive vehicle_blue model.\n\n"
    "Localization Method: Dead reckoning with the Euler-integrated motion model:\n"
    "    x(k+1) = x(k) + v*cos(theta(k))*dt\n"
    "    y(k+1) = y(k) + v*sin(theta(k))*dt\n"
    "    theta(k+1) = theta(k) + omega*dt\n\n"
    "with dt = 0.1 s. Uncertainty is modelled by adding Gaussian noise to the commanded velocities: "
    "v_odom = v_cmd + N(0, sigma_v), omega_odom = omega_cmd + N(0, sigma_w), with sigma_v = 0.05 m/s "
    "and sigma_w = 0.02 rad/s.\n\n"
    "Ground Truth: The simulator's true pose is obtained from /model/vehicle_blue/odometry and is "
    "used only for evaluation, never to generate the estimate."
)
ax.text(0.0, y, methods, fontsize=9.5, va='top', linespacing=1.6)

# 3. Results
y -= 0.245
ax.text(0.0, y, '3. Results', fontsize=12, fontweight='bold', va='top')
y -= 0.028
results = (
    "The estimated trajectory closely follows the ground-truth trajectory throughout the experiment. "
    "The final position error after the complete trajectory is 0.167 m.\n\n"
    "Error accumulates gradually due to the integration of noisy velocity measurements. The straight-line "
    "segments show smaller deviations, while the 90-degree turn introduces larger orientation error, "
    "which then propagates into position error during the subsequent straight segment.\n\n"
    "Sources of error include: (1) Gaussian noise added to the velocity commands, (2) discretization "
    "error from Euler integration at dt = 0.1 s, and (3) the idealized motion model which assumes no "
    "wheel slip and perfect differential-drive kinematics.\n\n"
    "The result demonstrates that dead reckoning provides reasonable short-term localization, but drift "
    "will grow unbounded over longer trajectories without external correction such as a particle filter "
    "or Kalman filter."
)
ax.text(0.0, y, results, fontsize=9.5, va='top', linespacing=1.6)

# Footer
ax.text(0.5, 0.02, 'Trajectory plot: results/trajectory_plot.pdf',
        ha='center', va='bottom', fontsize=8, color='gray', style='italic')

with PdfPages('report/report.pdf') as pdf:
    pdf.savefig(fig)
plt.close()
print('Report saved to report/report.pdf')
