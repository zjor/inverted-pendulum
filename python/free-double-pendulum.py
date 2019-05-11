"""
Simulation of a free double pendulum

References:
    - (Equation derivation) http://www.math24.ru/%D0%B4%D0%B2%D0%BE%D0%B9%D0%BD%D0%BE%D0%B9-%D0%BC%D0%B0%D1%8F%D1%82%D0%BD%D0%B8%D0%BA.html
	- (Original example)[https://matplotlib.org/gallery/animation/double_pendulum_sgskip.html]
"""

import numpy as np
import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as pp
import scipy.integrate as integrate
import matplotlib.animation as animation

from math import pi
from numpy import sin, cos

# physical constants
g = 9.8
l1 = 0.8
l2 = 0.5
m1 = 1.0
m2 = 0.4

# simulation time
dt = 0.05
Tmax = 20
t = np.arange(0.0, Tmax, dt)

# initial conditions
a0 = pi / 3  # angle of the first pendulum
Y0 = .0      # angular velocity of the first pendulum
b0 = pi / 5  # angle of the second pendulum
Z0 = .0      # angular velocity of the second pendulum

state = np.array([a0, Y0, b0, Z0])

def derivatives(state, t):
    ds = np.zeros_like(state)
    a = state[0]
    Y = state[1]
    b = state[2]
    Z = state[3]

    ds[0] = Y
    ds[1] = (- m2 * cos(a - b) * (l1 * Y * Y * sin(a - b) - g * sin(a - b)) - Z * Z * l2 * m2 * sin(a - b) - (m1 + m2) * g * sin(a)) / (l1 * (m1 + m2 * sin(a - b) * sin(a - b)))
    ds[2] = Z
    
    h = cos(a - b) / (m1 + m2)
    d = (m1 * l2 + m2 * l2 * sin(a - b) * sin(a - b)) / (m1 + m2)
    
    ds[3] = (h * (Z * Z * l2 * m2 * sin(a - b) + (m1 + m2) * g * sin(a)) + l1 * Y * Y * sin(a - b) - g * sin(b)) / d

    return ds

print("Integrating...")
# integrate your ODE using scipy.integrate.
solution = integrate.odeint(derivatives, state, t)
print("Done")

alphas = solution[:, 0]
betas = solution[:, 2]

xs1 = l1 * sin(alphas)
ys1 = -l1 * cos(alphas)
xs2 = l2 * sin(betas) + xs1
ys2 = -l2 * cos(betas) + ys1

fig = pp.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

cart_width = 0.3
cart_height = 0.2

def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text


def animate(i):
    thisx = [0, xs1[i], xs2[i]]
    thisy = [0, ys1[i], ys2[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*dt))
    return line, time_text

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution)),
                              interval=25, blit=True, init_func=init)

pp.show()

# Set up formatting for the movie files
# Writer = animation.writers['ffmpeg']
# writer = Writer(fps=15, metadata=dict(artist='Sergey Royz'), bitrate=1800)
# ani.save('free-cart.mp4', writer=writer)



