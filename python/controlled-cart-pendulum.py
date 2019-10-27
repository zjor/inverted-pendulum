"""
Simulation of controlling the inverted pendulum on a cart with state controller.

Equations:
	th'' = (g * sin(th) - u * cos(th)) / L,
	u = Kp_th * th + Kd_th * th' + Kp_x * (x - x0) + Kd_x * x'

System:
	th' = Y,
	Y' = (g * sin(th) - u * cos(th)) / L,
	x' = Z,
	Z' = u = Kp_th * th + Kd_th * Y + Kp_x * (x - x0) + Kd_x * Z,

State: 
	[th, Y, x, Z]
"""

import numpy as np

import matplotlib
matplotlib.use('TKAgg')

import matplotlib.pyplot as pp
import scipy.integrate as integrate
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

from math import pi, trunc
from numpy import sin, cos

def trim(x, step):
    d = trunc(x / step)
    return step * d

# physical constants
g = 9.8
L = 1.5
m = 0.5

# simulation time
dt = 0.05
Tmax = 5
t = np.arange(0.0, Tmax, dt)

# initial conditions
Y = .0 		# pendulum angular velocity
th = pi/10	# pendulum angle
x = .0		# cart position
x0 = 0		# desired cart position
Z = .0		# cart velocity

precision = 0.006
k = 1000.0	# Kalman filter coefficient

Kp_th = 50
Kd_th = 15
Kp_x = 3.1
Kd_x = 4.8

state = np.array([th, Y, x, Z, trim(th, precision), .0])

def step(t):
	if t < 5:
		return .0
	elif t >= 5 and t < 10:
		return 1.
	elif t >= 10 and t < 15:
		return -0.5
	else:
		return .0

def derivatives(state, t):
	ds = np.zeros_like(state)

	_th = state[0]
	_Y = state[1]
	_x = state[2]
	_Z = state[3]

	# x0 = step(t)

	u = Kp_th * _th + Kd_th * _Y + Kp_x * (_x - x0) + Kd_x * _Z

	ds[0] = state[1]
	ds[1] = (g * sin(_th) - u * cos(_th)) / L
	ds[2] = state[3]
	ds[3] = u

	return ds

print("Integrating...")
# integrate your ODE using scipy.integrate.
solution = integrate.odeint(derivatives, state, t)
print("Done")

ths = solution[:, 0]
xs = solution[:, 2]

pxs = L * sin(ths) + xs
pys = L * cos(ths)

fig = pp.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-0.5, 2))
ax.set_aspect('equal')
ax.grid()

patch = ax.add_patch(Rectangle((0, 0), 0, 0, linewidth=1, edgecolor='k', facecolor='g'))

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

cart_width = 0.3
cart_height = 0.2

def init():
    line.set_data([], [])
    time_text.set_text('')
    patch.set_xy((-cart_width/2, -cart_height/2))
    patch.set_width(cart_width)
    patch.set_height(cart_height)
    return line, time_text, patch


def animate(i):
    thisx = [xs[i], pxs[i]]
    thisy = [0, pys[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*dt))
    patch.set_x(xs[i] - cart_width/2)
    return line, time_text, patch

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution)),
                              interval=25, blit=True, init_func=init)

pp.show()

# Set up formatting for the movie files
print("Writing video...")
Writer = animation.writers['imagemagick']
writer = Writer(fps=25, metadata=dict(artist='Sergey Royz'), bitrate=1800)
ani.save('controlled-cart.gif', writer=writer)
