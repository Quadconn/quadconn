import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import QuadconnBindings as qb

def link_line_segment(length, angle, xstart=0, ystart=0):
    xend = xstart + length * np.cos(angle)
    yend = ystart + length * np.sin(angle)
    return [xstart, xend], [ystart, yend]

l1, l2 = 3, 4
radius_workspace = l1 + l2
manip = qb.Manipulator(l1, l2)
j = qb.Manipulator.JointAngles()

circle_steps = 100
theta = np.linspace(0, 2*np.pi, circle_steps)
x_targets = radius_workspace * np.cos(theta)
y_targets = radius_workspace * np.sin(theta)

fig, ax = plt.subplots()
dot, = ax.plot(0, 0, 'bo')
link1, = ax.plot([0,0], [0,0], 'r-')
link2, = ax.plot([0,0], [0,0], 'y-')

ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)

def update(frame):
    global radius_workspace

    x_t = radius_workspace * np.cos(theta[frame])
    y_t = radius_workspace * np.sin(theta[frame])

    print(manip.ik(x_t, y_t, j))
    theta1, theta2 = j.theta1_p, j.theta2_p

    dot.set_data([x_t], [y_t])

    link1_xs, link1_ys = link_line_segment(l1, theta1)
    link1.set_data(link1_xs, link1_ys)

    link2_xs, link2_ys = link_line_segment(l2, theta2 + theta1, link1_xs[-1], link1_ys[-1])
    link2.set_data(link2_xs, link2_ys)

    if radius_workspace - 0.05 >= 0.0:
        radius_workspace -= 0.05

    return dot, link1, link2

ani = FuncAnimation(fig, update, frames=len(theta), interval=200, blit=True)
plt.show()
