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
l = 1.0
m_p = 0.5
m_k = 1.0
m_r = 0.001 # mass of the broom
k = 0.5 # motor torque constant
I = m_r * l * l / 12 # rotational inertia of a rod (m_r * l * l / 12)
R = 6.5 # motor resistance
r = 0.05   # shaft radius

M = m_k + m_p
L = (I + m_p * l * l) / (m_p * l)

# simulation time
dt = 0.01
Tmax = 10
t = np.arange(0.0, Tmax, dt)

# initial conditions
x1_0 = 0.0  # x - cart position
x2_0 = 0.0 # v - cart velocity
x3_0 = pi/12 # theta - pendulum angle
x4_0 = 0.0  # angular velocity

state = np.array([x1_0, x2_0, x3_0, x4_0])

# control matrix
K = [-5.4772255750531889, -23.938795926902721, -93.42203852466433, -28.650581504633493]

def disturbance(t):
    if t > 1.0 and t < 1.2:
        return -10.0
    else:
        return 0.0

def derivatives(state, t):
    ds = np.zeros_like(state)

    x, dx, th, dth = state

    u = -np.dot(K, state)
    # u = 0.0

    F = k * u / (R * r) - k * k / (R * r * r) * dx
    # F = 0

    ds[0] = dx
    ds[1] = (F - m_p * l * g * cos(th) * sin(th) / L + m_p * l * dth * dth * sin(th)) / (M - m_p * l * cos(th) * cos(th) / L)
    ds[2] = dth
    ds[3] = (- cos(th) / M * F + g * sin(th) - m_p * l * dth * dth * cos(th) * sin(th) / M) / (L - m_p * l * cos(th) * cos(th) / M)
    ds[3] = ds[3] + disturbance(t)

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



