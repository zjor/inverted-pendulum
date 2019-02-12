"""
Simulation from a free pendulum with animation.

Equation:
	Th'' = - g / L * sin(Th)

System:
	Th' = Y
	Y' = - g / L * sin(Th)

State:
	[Y, Th]

References:
	- (Original example)[https://matplotlib.org/gallery/animation/double_pendulum_sgskip.html]
"""

import numpy as np
import matplotlib.pyplot as pp
import scipy.integrate as integrate
import matplotlib.animation as animation

from math import pi
from numpy import sin, cos

# physical constants
g = 9.8
L = 1.0

# simulation time
dt = 0.05
Tmax = 20
t = np.arange(0.0, Tmax, dt)

# initial conditions
Y = 0 # angular velocity
Th = pi/6

state = np.array([Th, Y])

def derivatives(state, t):
	ds = np.zeros_like(state)
	ds[0] = state[1]
	ds[1] = -g / L * sin(state[0])
	return ds

# integrate your ODE using scipy.integrate.
solution = integrate.odeint(derivatives, state, t)

x = L * sin(solution[:, 0])
y = - L * cos(solution[:, 0])

fig = pp.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text


def animate(i):
    thisx = [0, x[i]]
    thisy = [0, y[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*dt))
    return line, time_text

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution)),
                              interval=25, blit=True, init_func=init)

pp.show()

