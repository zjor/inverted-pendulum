"""
Stabilisation of DC-motor controlled pendulum.
Equations of motion are taken from [1]


References:
	- [1][https://pdfs.semanticscholar.org/dadf/8b96134eb01399f925a6652f23fde66a45a2.pdf]
"""

import numpy as np

import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as pp
import scipy.integrate as integrate
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

from math import pi
from numpy import sin, cos

# physical constants
g = 9.8
l = 0.8
m_p = 0.5
m_k = 0.2
k = 0.5 # motor torque constant
I = 0.1 # rotational inertia
R = 0.5 # motor resistance
r = 0.5   # shaft radius

# simulation time
dt = 0.05
Tmax = 20
t = np.arange(0.0, Tmax, dt)

# initial conditions
x1_0 = 0.3  # x - cart position
x2_0 = 0.0 # v - cart velocity
x3_0 = pi/6 # theta - pendulum angle
x4_0 = 0.0  # angular velocity

state = np.array([x1_0, x2_0, x3_0, x4_0])

M = m_k + m_p
L = (I + m_p * l * l) / (m_p * l)

# control matrix
K = [.0, .0, -0.06, -0.03]

def derivatives(state, t):
    ds = np.zeros_like(state)

    _x1, _x2, _x3, _x4 = state

    u = np.dot(K, state)

    ds[0] = _x2
    ds[1] = (k / (R * r) * u - k * k / (R * r * r) * _x2 - m_p * l * g * cos(_x3) * sin(_x3) + m_p * l * _x4 * _x4 * sin(_x3)) / (M - m_p * l * cos(_x3) * cos(_x3) / L)
    ds[2] = _x4
    ds[3] = (cos(_x3) / M * (k * k / (R * r * r) * _x2 - k / (R * r) * u) + g * sin(_x3) - m_p * l * _x4 * _x4 * cos(_x3) * sin(_x3) / M) / (L - m_p*l*cos(_x3) * cos(_x3) / M)

    return ds

print("Integrating...")
# integrate your ODE using scipy.integrate.
solution = integrate.odeint(derivatives, state, t)
print("Done")

ths = solution[:, 2]
xs = solution[:, 0]

pxs = l * sin(ths) + xs
pys = l * cos(ths)

fig = pp.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
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
    patch.set_xy((-cart_width/2 + x1_0, -cart_height/2))
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
# Writer = animation.writers['ffmpeg']
# writer = Writer(fps=15, metadata=dict(artist='Sergey Royz'), bitrate=1800)
# ani.save('free-cart.mp4', writer=writer)



